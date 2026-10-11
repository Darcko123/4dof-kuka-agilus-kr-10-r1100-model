"""
robot.py - Cliente de control del KUKA Agilus KR10.

Traduce comandos de la interfaz de usuario en líneas de protocolo y las
respuestas del firmware en objetos (ArmState, RobotInfo, JointLimit). No dibuja
nada: la presentación vive en ui.py.
"""

from __future__ import annotations

from . import protocol as proto
from .transport import (
    TIMEOUT_FAST,
    TIMEOUT_MOVE,
    TIMEOUT_SEQ_STEP,
    Transport,
)


class Robot:
    """Envoltura de alto nivel del firmware."""

    def __init__(self, transport: Transport):
        self.t = transport

    # --- Conexión ------------------------------------------------------------

    @classmethod
    def connect(
        cls, port: str | None = None, timeout: float = TIMEOUT_FAST
    ) -> "Robot":
        """Abre el enlace y devuelve un cliente listo para usar."""
        t = Transport(port=port, timeout=timeout)
        t.open()
        return cls(t)

    def close(self) -> None:
        self.t.close()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- Comandos de solo lectura --------------------------------------------

    def ping(self) -> str:
        """Versión de firmware."""
        block = self.t.request(proto.CMD_PING)
        _raise_on_error(block)
        tokens = block[0].split()
        return tokens[2] if len(tokens) > 2 else "unknown"

    def info(self) -> proto.RobotInfo:
        """Identidad y capacidades del robot."""
        block = self.t.request(proto.CMD_HELLO)
        _raise_on_error(block)
        info = proto.RobotInfo()

        for line in block[1:]:
            parts = line.split()
            if not parts:
                continue
            tag = parts[0]
            if tag == "ID":
                # ID KR10 4DOF 1.1
                info.model = parts[1] if len(parts) > 1 else info.model
                info.version = parts[-1] if len(parts) > 2 else info.version
            elif tag == "JOINTS":
                info.joints = _int(parts, 1, info.joints)
            elif tag == "MOVE_MS":
                info.move_ms = _int(parts, 1, info.move_ms)
            elif tag == "SEQ_LEN":
                info.seq_length = _int(parts, 1, info.seq_length)
            elif tag == "SEQ_DELAY_MS":
                info.seq_delay_ms = _int(parts, 1, info.seq_delay_ms)
            elif tag == "OLED":
                info.oled_ok = _int(parts, 1, 1) == 1

        return info

    def status(self) -> proto.ArmState:
        """Cinemática actual: ángulos articulares y pose del efector."""
        block = self.t.request(proto.CMD_STATUS)
        _raise_on_error(block)
        return _parse_state(block)

    def limits(self) -> list[proto.JointLimit]:
        """Límites articulares en grados."""
        block = self.t.request(proto.CMD_LIMITS)
        _raise_on_error(block)

        result = []
        for line in block[1:]:
            parts = line.split()
            if len(parts) >= 4 and parts[0] == "LIM":
                result.append(
                    proto.JointLimit(
                        index=_int(parts, 1, len(result) + 1) - 1,
                        min_deg=_float(parts, 2),
                        max_deg=_float(parts, 3),
                    )
                )
        return result

    def sequence_status(self) -> proto.SequenceState:
        """Estado de la secuencia automática."""
        block = self.t.request(proto.CMD_SEQ_STATUS)
        _raise_on_error(block)

        for line in block[1:]:
            parts = line.split()
            if parts and parts[0] == "SEQ":
                if len(parts) > 1 and parts[1].upper() == "IDLE":
                    return proto.SequenceState(running=False)
                return proto.SequenceState(
                    running=True,
                    step=_int(parts, 1, 1),
                    total=_int(parts, 2, 0),
                )
        return proto.SequenceState(running=False)

    # --- Comandos de movimiento ----------------------------------------------

    def move(self, angles_deg: list[float]) -> proto.ArmState:
        """Mueve el brazo a los ángulos dados (grados) y devuelve el estado final.

        El firmware no bloquea: MOVE devuelve el acuse y el estado se emite al
        terminar el movimiento, así que aquí se encadenan ambos bloques.
        """
        if len(angles_deg) != proto.NUM_JOINTS:
            raise ValueError(
                f"Se esperaban {proto.NUM_JOINTS} ángulos, "
                f"se recibieron {len(angles_deg)}"
            )

        command = " ".join(
            [proto.CMD_MOVE] + [_fmt_angle(a) for a in angles_deg]
        )
        ack = self.t.request(command, timeout=TIMEOUT_MOVE)
        _raise_on_error(ack)

        done = self.t.read_block(timeout=TIMEOUT_MOVE)
        _raise_on_error(done)
        return _parse_state(done)

    def home(self) -> proto.ArmState:
        """Lleva el brazo a la posición HOME."""
        ack = self.t.request(proto.CMD_HOME, timeout=TIMEOUT_MOVE)
        _raise_on_error(ack)

        done = self.t.read_block(timeout=TIMEOUT_MOVE)
        _raise_on_error(done)
        return _parse_state(done)

    def stop(self) -> proto.ArmState:
        """Aborta el movimiento o la secuencia en curso."""
        # El firmware responde OK STOP + END y después el estado + END
        _raise_on_error(self.t.request(proto.CMD_STOP, timeout=TIMEOUT_FAST))
        return _parse_state(self.t.read_block(timeout=TIMEOUT_FAST))

    def oled(self, title: str) -> None:
        """Cambia el título de la barra superior de la OLED."""
        if not title:
            raise ValueError("El título de la OLED no puede estar vacío")
        block = self.t.request(f"{proto.CMD_OLED} {title}")
        _raise_on_error(block)

    # --- Secuencia automática -------------------------------------------------

    def sequence_start(self) -> None:
        """Arranca la secuencia de prueba en el firmware."""
        block = self.t.request(proto.CMD_SEQ_START, timeout=TIMEOUT_FAST)
        _raise_on_error(block)

    def sequence_stop(self) -> proto.SequenceState:
        """Detiene la secuencia de prueba."""
        block = self.t.request(proto.CMD_SEQ_STOP, timeout=TIMEOUT_FAST)
        _raise_on_error(block)
        return self._sequence_from(block)

    def sequence_poll(self, timeout: float = TIMEOUT_SEQ_STEP) -> proto.SequenceState:
        """Espera el bloque de la siguiente pose de la secuencia.

        El firmware emite un bloque por pose (SEQ, Q..., POSE, OLED, END) sin
        acuse previo, así que aquí solo se lee.
        """
        block = self.t.read_block(timeout=timeout)
        _raise_on_error(block)
        return self._sequence_from(block)

    def _sequence_from(self, block: list[str]) -> proto.SequenceState:
        for line in block:
            parts = line.split()
            if parts and parts[0] == "SEQ":
                if len(parts) > 1 and parts[1].upper() == "IDLE":
                    return proto.SequenceState(running=False)
                return proto.SequenceState(
                    running=True,
                    step=_int(parts, 1, 1),
                    total=_int(parts, 2, 0),
                )
        return proto.SequenceState(running=False)


# --- Parseo de bloques -------------------------------------------------------


def _parse_state(block: list[str]) -> proto.ArmState:
    """Convierte un bloque OK/END en ArmState."""
    state = proto.ArmState()
    seen_q = False

    for line in block:
        parts = line.split()
        if not parts:
            continue
        tag = parts[0]

        if tag == "Q" and len(parts) >= 3:
            index = _int(parts, 1, len(state.q) + 1) - 1
            if 0 <= index < len(state.q):
                state.q[index] = _float(parts, 2)
                seen_q = True
        elif tag == "POSE" and len(parts) >= 6:
            state.x, state.y, state.z, state.pitch, state.yaw = (
                _float(parts, 1),
                _float(parts, 2),
                _float(parts, 3),
                _float(parts, 4),
                _float(parts, 5),
            )
        elif tag == "OLED" and len(parts) >= 2:
            state.oled_ok = _int(parts, 1, 1) == 1

    if not seen_q:
        raise proto.ProtocolError("El firmware no devolvió los ángulos articulares")

    return state


def _raise_on_error(block: list[str]) -> None:
    """Convierte un bloque ERR en excepción."""
    for line in block:
        if line.startswith(proto.ERR):
            code = line[len(proto.ERR) :].strip()
            raise proto.ProtocolError(proto.describe_error(code))


def _int(parts: list[str], index: int, default: int = 0) -> int:
    try:
        return int(float(parts[index]))
    except (IndexError, ValueError):
        return default


def _float(parts: list[str], index: int, default: float = 0.0) -> float:
    try:
        return float(parts[index])
    except (IndexError, ValueError):
        return default


def _fmt_angle(value: float) -> str:
    """Formatea un ángulo en grados para el comando MOVE.

    Se fuerza el punto decimal por si el locale cambiara; el parser del
    firmware acepta tanto '.' como ','.
    """
    return f"{value:.2f}".replace(",", ".")