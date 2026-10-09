"""
test_menu.py - Prueba del menú interactivo contra un firmware simulado.

Encadena las cuatro opciones del menú de terminal del firmware tal y como lo
haría un usuario, comprobando que cada una llega al firmware como el comando
correcto.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import serial  # noqa: E402

from modules.core import menu  # noqa: E402
from modules.core.robot import Robot  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_protocol import FakeSerial  # noqa: E402

# Secuencia de teclas que el usuario "escribe": ver posición, HOME, ángulos
# manuales y una opción inválida.
TECLAS = iter(
    [
        "4",  # ver posición actual
        "",  # ENTER para volver
        "3",  # ir a HOME
        "",  # ENTER
        "2",  # ángulos manuales
        "45",  # Q1
        "30",  # Q2
        "-30,5",  # Q3 con coma decimal
        "0",  # Q4
        "",  # ENTER
        "9",  # opción inválida
        "",  # ENTER para volver
        "2",  # ángulos manuales cancelado
        "",  # cancelar con ENTER vacío
        "",  # ENTER para volver
        "1",  # secuencia automática
        "4",  # ver posición (la secuencia termina sola en el firmware real)
        "",  # ENTER para volver
    ]
)


def test():
    serial.Serial = FakeSerial
    robot = Robot.connect("/dev/fake")

    # menu._leer_linea usa input(); se sustituye por las teclas preparadas.
    # Al agotarse se interrumpe, como si el usuario pulsara Ctrl-C.
    def _tecla():
        try:
            return next(TECLAS)
        except StopIteration:
            raise KeyboardInterrupt

    menu._leer_linea = _tecla

    menu.run(robot, robot.info())

    enviados = robot.t._serial.received
    print("\nComandos enviados al firmware:")
    for c in enviados:
        print("  ", c)

    esperados = ["HOME", "MOVE 45.00 30.00 -30.50 0.00", "SEQ START"]
    faltan = [e for e in esperados if e not in enviados]
    if faltan:
        print("\nFALTAN:", faltan)
        raise SystemExit(1)

    # La secuencia solo debe arrancar cuando se elige la opción 1
    if enviados.index("SEQ START") < enviados.index("MOVE 45.00 30.00 -30.50 0.00"):
        print("\nFALLO: la secuencia arrancó antes de tiempo")
        raise SystemExit(1)

    robot.close()
    print("\nMenú verificado: las 4 opciones producen los comandos esperados")


if __name__ == "__main__":
    test()