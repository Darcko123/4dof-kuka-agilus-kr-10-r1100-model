"""
transport.py - Enlace USB-CDC con el STM32.

Solo se encarga de enviar comandos y recibir líneas ``KR10:``. No sabe nada de
cinemática: el parseo de bloques está en protocol.py y el consumo en robot.py.

El firmware descarta lo que se le envía si el puerto USB no está enumerado
(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED), así que tras abrir el puerto
se espera un PING hasta que el STM32 esté listo.
"""

from __future__ import annotations

import time

import serial
from serial.tools import list_ports

from . import protocol as proto

# Tamaño de lectura del búfer del puerto. Las respuestas más largas del
# firmware son los bloques de estado (4 líneas Q + POSE + OLED).
READ_CHUNK = 512

# Plazos por defecto (s). El movimiento dura SERVO_MOVE_DURATION (1500 ms) y
# la pausa entre poses de la secuencia DELAY_TEST_SEQUENCE (5000 ms).
TIMEOUT_FAST = 1.0
TIMEOUT_MOVE = 5.0
TIMEOUT_SEQ_STEP = 12.0


def list_devices() -> list[str]:
    """Puertos serie que parecen un CDC de STMicroelectronics."""
    found = []
    for port in list_ports.comports():
        vid, pid = port.vid, port.pid
        # 0483:5740 es el VID/PID del CDC de ST; el resto se listan igual
        # porque algunosEnumerator Bridges cambian el PID.
        if (vid, pid) in ((0x0483, 0x5740), (0x0483, 0x5741)) or port.name.startswith(
            ("/dev/ttyACM", "COM")
        ):
            found.append(port.device)
    return found


class Transport:
    """Enlace de líneas con el firmware."""

    def __init__(self, port: str | None = None, baudrate: int = 115200, timeout: float = TIMEOUT_FAST):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: serial.Serial | None = None
        self._buffer = b""

    # --- Ciclo de vida -------------------------------------------------------

    def open(self, wait: float = 10.0) -> None:
        """Abre el puerto y espera a que el firmware responda a un PING."""
        candidates = [self.port] if self.port else list_devices()
        if not candidates:
            raise ConnectionError(
                "No se encontró ningún puerto USB-CDC del STM32.\n"
                "Conecta el prototipo y vuelve a intentarlo."
            )

        last_error: Exception | None = None
        for device in candidates:
            try:
                self._serial = serial.Serial(
                    device,
                    self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                break
            except serial.SerialException as e:  # pragma: no cover - hardware
                last_error = e
                self._serial = None
        else:
            raise ConnectionError(f"No se pudo abrir el puerto: {last_error}")

        self.port = self._serial.name
        self._buffer = b""

        # El firmware pierde lo que se le mande hasta estar configurado por
        # el host, de ahí el ping de espera.
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            try:
                self.send(proto.CMD_PING)
                self.read_block()
                return
            except (proto.TimeoutError_, serial.SerialException, OSError):
                time.sleep(0.25)

        self.close()
        raise ConnectionError(
            f"El dispositivo {self.port} no respondió a PING.\n"
            "Comprueba que el firmware esté grabado y en modo protocolo."
        )

    def close(self) -> None:
        """Cierra el puerto, ignorando errores de un cierre forzado."""
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def __enter__(self) -> "Transport":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- IO ------------------------------------------------------------------

    def send(self, command: str) -> None:
        """Envía un comando terminated en CRLF."""
        if self._serial is None:
            raise ConnectionError("El puerto no está abierto")
        self._serial.write(f"{command}\r\n".encode("ascii"))

    def read_lines(self, timeout: float | None = None) -> list[str]:
        """Lee líneas completas hasta agotar el plazo, sin bloqueo."""
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)

        while time.monotonic() < deadline:
            if proto.END in self._payloads():
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self._serial is None:
                break
            waiting = self._serial.in_waiting
            chunk = self._serial.read(waiting if waiting else 1)
            if chunk:
                self._buffer += chunk

        return self._payloads()

    def read_block(self, timeout: float | None = None) -> list[str]:
        """Lee un bloque completo de respuesta, hasta el token END.

        :raises proto.TimeoutError_: si no llega el END a tiempo.
        """
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        payloads: list[str] = []

        while time.monotonic() < deadline:
            payloads = self._payloads()
            if proto.END in payloads:
                return payloads
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self._serial is None:
                break
            waiting = self._serial.in_waiting
            chunk = self._serial.read(waiting if waiting else 1)
            if chunk:
                self._buffer += chunk

        raise proto.TimeoutError_(
            "El STM32 no respondió a tiempo. "
            "¿Está grabando en el puerto USB y el firmware en modo protocolo?"
        )

    def request(self, command: str, timeout: float | None = None) -> list[str]:
        """Envía un comando y devuelve su bloque de respuesta."""
        self.send(command)
        return self.read_block(timeout)

    def drain(self, seconds: float = 0.05) -> None:
        """Descarta lo pendiente sin interpretar, para partir de limpio."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            payloads = self._payloads()
            if not payloads:
                break
        self._buffer = b""

    # --- Interno -------------------------------------------------------------

    def _payloads(self) -> list[str]:
        """Extrae del búfer las líneas con prefijo KR10: ya completas.

        Las líneas que no llevan prefijo (restos de una sesión anterior en el
        búfer del driver) se descartan, igual que los '\r' sobrantes del CRLF.
        """
        found = []
        while b"\n" in self._buffer:
            line, self._buffer = self._buffer.split(b"\n", 1)
            text = line.strip().decode("ascii", "replace")
            if text.startswith(proto.PREFIX):
                found.append(text[len(proto.PREFIX) :])
        return found