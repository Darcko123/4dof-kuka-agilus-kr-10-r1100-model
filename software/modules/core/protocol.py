"""
protocol.py - Definiciones del protocolo de texto entre Python y el STM32.

El firmware (KR10_FW_VERSION >= 1.1 en Core/Src/main.c) actúa como agente de
control: expone comandos de texto por USB-CDC y devuelve el estado cinemático.
La interfaz de usuario vive aquí, en Python.

Formato de línea
----------------
Todo lo que envía el firmware llega como ``KR10:<payload>\\r\\n``. Los payloads
son tokens separados por espacios; el host envía comandos sin prefijo.

Una respuesta completa es un bloque de líneas que termina en ``KR10:END``::

    KR10:OK STATUS
    KR10:Q 1 0.00
    ...
    KR10:POSE 187.00 0.00 65.00 0.00 0.00
    KR10:OLED 1
    KR10:END

Un movimiento no bloquea el firmware: ``MOVE`` devuelve el acuse de inmediato y
el bloque ``OK`` + estado se emite al terminar el movimiento. La secuencia
automática emite un bloque por pose, sin acuse previo.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# --- Tokens del protocolo ----------------------------------------------------

PREFIX = "KR10:"
END = "END"
OK = "OK"
ERR = "ERR"

# Comandos aceptados por el firmware
CMD_PING = "PING"
CMD_HELLO = "HELLO"
CMD_STATUS = "STATUS"
CMD_LIMITS = "LIMITS"
CMD_MOVE = "MOVE"
CMD_HOME = "HOME"
CMD_SEQ = "SEQ"
CMD_SEQ_START = "SEQ START"
CMD_SEQ_STOP = "SEQ STOP"
CMD_SEQ_STATUS = "SEQ STATUS"
CMD_OLED = "OLED"
CMD_STOP = "STOP"

# Subtokens de argüento de SEQ
SEQ_START = "START"
SEQ_STOP = "STOP"
SEQ_STATUS = "STATUS"

# Códigos de error emitidos por el firmware
ERR_CMD = "CMD"
ERR_ANGLE = "ANGLE"
ERR_RANGE = "RANGE"
ERR_SEQ_ARG = "SEQ ARG"
ERR_OLED_ARG = "OLED ARG"

ERROR_TEXT = {
    ERR_CMD: "comando desconocido",
    ERR_ANGLE: "ángulos inválidos o fuera de rango",
    ERR_RANGE: "movimiento fuera de límites",
    ERR_SEQ_ARG: "subcomando de SEQ inválido (use START, STOP o STATUS)",
    ERR_OLED_ARG: "falta el título para OLED",
}

# Nombres de las articulaciones: los envía el firmware en HELLO como
# JOINT <i> <nombre>, pero se mantienen aquí como respaldo para firmware
# antiguo que no los envíe.
DEFAULT_JOINT_NAMES = ["Base", "Hombro", "Codo", "Muñeca"]

NUM_JOINTS = 4


# --- Objetos de datos --------------------------------------------------------


@dataclass
class ArmState:
    """Cinemática del brazo tal y como la reporta el firmware."""

    q: list[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    oled_ok: bool = True

    @property
    def joints(self) -> int:
        return len(self.q)


@dataclass
class JointLimit:
    """Límite articular en grados, tal y como lo reporta el firmware."""

    index: int
    min_deg: float
    max_deg: float

    def contains(self, value: float) -> bool:
        return self.min_deg <= value <= self.max_deg


@dataclass
class RobotInfo:
    """Identidad y capacidades del robot (respuesta a HELLO)."""

    model: str = "KR10"
    version: str = "unknown"
    joints: int = NUM_JOINTS
    joint_names: list[str] = field(default_factory=lambda: list(DEFAULT_JOINT_NAMES))
    move_ms: int = 1500
    seq_length: int = 5
    seq_delay_ms: int = 5000
    oled_ok: bool = True


@dataclass
class SequenceState:
    """Avance de la secuencia automática."""

    running: bool = False
    step: int = 0
    total: int = 0


# --- Errores -----------------------------------------------------------------


class ProtocolError(Exception):
    """Respuesta del firmware con el token ERR."""


class TimeoutError_(Exception):
    """El firmware no respondió dentro del plazo."""


def describe_error(code: str) -> str:
    """Traduce un código de error del firmware a texto legible."""
    return ERROR_TEXT.get(code, code)