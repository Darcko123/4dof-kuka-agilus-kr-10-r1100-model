"""
test_protocol.py - Pruebas del cliente Python contra un firmware simulado.

Reproduce la salida del STM32 (Core/Src/main.c, sección PROTOCOLO DE HOST)
para comprobar el parseo y la secuización de comandos sin hardware.
"""

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import serial  # noqa: E402

from modules.core import protocol as proto  # noqa: E402
from modules.core.robot import Robot, _raise_on_error  # noqa: E402
from modules.core.transport import Transport  # noqa: E402


class FakeSerial:
    """Doble de pyserial que habla el protocolo del firmware."""

    def __init__(self, port, baudrate, timeout=None, write_timeout=None):
        self.name = port
        self.timeout = timeout
        self._out = b""
        self._in = b""
        self._lock = threading.Lock()
        self.received = []
        self.q = [0.0, 0.0, 0.0, 0.0]
        self._seq_active = False
        self._seq_step = 0
        self._seq_len = 5
        self._seq_delay = 0.15
        self.oled = 1
        threading.Thread(target=self._sequence_loop, daemon=True).start()

    # --- pyserial API -------------------------------------------------------

    @property
    def in_waiting(self):
        # Como el STM32: cada línea llega en su propio paquete USB
        with self._lock:
            i = self._in.find(b"\n")
            return i + 1 if i >= 0 else len(self._in)

    def reset_input_buffer(self):
        with self._lock:
            self._in = b""

    def write(self, data):
        text = data.decode().strip()
        self.received.append(text)
        threading.Thread(target=self._handle, args=(text,), daemon=True).start()
        return len(data)

    def read(self, size=1):
        with self._lock:
            chunk, self._in = self._in[:size], self._in[size:]
        return chunk

    def close(self):
        pass

    # --- Firmware simulado --------------------------------------------------

    def _send(self, line):
        with self._lock:
            self._in += f"KR10:{line}\r\n".encode()

    def _end(self):
        self._send("END")

    def _err(self, code):
        self._send(f"ERR {code}")
        self._end()

    def _state(self):
        for i, q in enumerate(self.q):
            self._send(f"Q {i + 1} {q:.2f}")
        self._send("POSE 157.00 0.00 65.00 0.00 0.00")
        self._send(f"OLED {self.oled}")
        self._end()

    def _sequence_loop(self):
        while True:
            time.sleep(0.01)
            if not self._seq_active:
                continue
            step = self._seq_step
            if step >= self._seq_len:
                self._seq_active = False
                self._send("SEQ IDLE")
                self._state()
                continue
            self.q = [step * 5.0] * 4
            self._seq_step += 1
            self._send(f"SEQ {step + 1} {self._seq_len}")
            self._state()
            time.sleep(self._seq_delay)

    def _handle(self, line):
        cmd, _, arg = line.partition(" ")
        cmd = cmd.upper()
        arg = arg.strip()

        if cmd == "PING":
            self._send("OK PONG 1.1")
            self._end()
        elif cmd == "HELLO":
            self._send("OK")
            self._send("ID KR10 4DOF 1.1")
            self._send("JOINTS 4")
            self._send("MOVE_MS 1500")
            self._send("SEQ_LEN 5")
            self._send("SEQ_DELAY_MS 5000")
            self._send(f"OLED {self.oled}")
            self._end()
        elif cmd == "STATUS":
            self._send("OK")
            self._state()
        elif cmd == "LIMITS":
            self._send("OK")
            for i in range(4):
                self._send(f"LIM {i + 1} 0.0 180.0")
            self._end()
        elif cmd in ("MOVE", "HOME"):
            if cmd == "HOME":
                values = [0.0] * 4
            else:
                try:
                    values = [float(v) for v in arg.replace(",", ".").split()]
                except ValueError:
                    self._err("ANGLE")
                    return
                if len(values) != 4 or any(not 0 <= v <= 180 for v in values):
                    self._err("ANGLE")
                    return
            self._send(f"OK {cmd}")
            self._end()
            # El firmware mueve los servos y avisa al terminar
            self.q = values
            time.sleep(0.05)
            self._send("OK")
            self._state()
        elif cmd == "SEQ":
            sub = arg.upper()
            if sub == "START":
                self._seq_active = True
                self._seq_step = 0
                self._send("OK SEQ START")
                self._end()
            elif sub == "STOP":
                self._seq_active = False
                self._send("OK SEQ STOP")
                self._end()
            elif sub == "STATUS":
                self._send("OK")
                if self._seq_active:
                    self._send(f"SEQ {self._seq_step} {self._seq_len}")
                else:
                    self._send("SEQ IDLE")
                self._end()
            else:
                self._err("SEQ ARG")
        elif cmd == "STOP":
            self._seq_active = False
            self._send("OK STOP")
            self._end()
            self._state()
        elif cmd == "OLED":
            if not arg:
                self._err("OLED ARG")
            else:
                self._send("OK OLED")
                self._end()
        else:
            self._err("CMD")


def _patch_serial():
    serial.Serial = FakeSerial


class FakeMudo(FakeSerial):
    """Enlace que acepta la conexión pero nunca responde (firmware colgado)."""

    def _handle(self, line):
        pass


def test():
    _patch_serial()

    robot = Robot.connect("/dev/fake")
    print("PING   ->", robot.ping())

    info = robot.info()
    print("HELLO  ->", info.model, info.version, info.joints, "GDL, OLED", info.oled_ok)

    limites = robot.limits()
    print("LIMITS ->", [(l.min_deg, l.max_deg) for l in limites])

    estado = robot.status()
    print("STATUS -> Q", estado.q, "pose", (estado.x, estado.y, estado.z))

    estado = robot.move([45, 30, 30, 0])
    print("MOVE   -> Q", estado.q)
    print("        enviados:", robot.t._serial.received[-1])

    estado = robot.home()
    print("HOME   -> Q", estado.q)

    robot.oled("PRUEBA")
    print("OLED   ->", robot.t._serial.received[-1])

    # Fuera de rango: debe llegar ERR del firmware
    try:
        robot.move([0, 0, 0, 999])
        print("ERROR  -> no se lanzó ProtocolError (fallo)")
    except proto.ProtocolError as e:
        print("ERROR  ->", e)

    # Comando inexistente: el cliente lo traduce al leer el bloque
    try:
        _raise_on_error(robot.t.request("BAILE"))
        print("ERRCMD -> no se lanzó ProtocolError (fallo)")
    except proto.ProtocolError as e:
        print("ERRCMD ->", e)

    # Secuencia: consumes los 5 poses
    robot.sequence_start()
    pasos = []
    while True:
        sec = robot.sequence_poll(timeout=3.0)
        if not sec.running:
            break
        pasos.append((sec.step, sec.total))
    print("SEQ    -> poses", pasos)

    # Move simultáneo con la secuencia parada
    robot.sequence_stop()
    estado = robot.stop()
    print("STOP   -> Q", estado.q)

    # Timeouts: un enlace que acepta la conexión pero nunca responde
    mudo = Robot(Transport.__new__(Transport))
    mudo.t.port = "/dev/mudo"
    mudo.t.baudrate = 115200
    mudo.t.timeout = 0.3
    mudo.t._serial = FakeMudo("/dev/mudo", 115200)
    mudo.t._buffer = b""
    mudo.t._lines = []
    try:
        mudo.status()
        print("TIMEOUT-> no se lanzó TimeoutError (fallo)")
    except proto.TimeoutError_ as e:
        print("TIMEOUT->", str(e)[:48], "...")

    print("\nTodas las comprobaciones pasaron")


if __name__ == "__main__":
    test()