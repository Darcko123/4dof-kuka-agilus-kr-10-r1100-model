"""
menu.py - Menú interactivo que reemplaza al del firmware.

Mantiene las cuatro opciones del menú de terminal del STM32 y su
comportamiento: secuencia automática, captura manual de ángulos articulares,
ir a HOME y ver la posición actual. Lo que hace es pedir los datos al usuario
y enviarlos como comandos de texto; toda la lógica de control sigue en el
firmware.
"""

from __future__ import annotations

import sys

import click

from ..utils.output import (
    console,
    print_dim,
    print_info,
    print_success,
    print_warning,
)
from . import protocol as proto
from . import ui
from .robot import Robot


def _leer_linea() -> str:
    """Lee una línea de la entrada estándar.

    Si la entrada no es un terminal (p. ej. una tubería) se devuelve "" en
    lugar de leer, para que la interfaz no se quede esperando entrada.
    """
    if not sys.stdin.isatty():
        return ""
    return input().strip()


def _esperar_tecla(mensaje: str = "↵ ENTER para continuar") -> None:
    """Espera un ENTER, como el Menu_PrintReturn() del firmware."""
    console.print()
    print_dim(mensaje)
    _leer_linea()


def _parse_angulo(texto: str, limite: proto.JointLimit) -> float | None:
    """Valida un ángulo en grados contra los límites de su articulación."""
    texto = texto.strip().replace(",", ".")
    if not texto:
        return None
    try:
        valor = float(texto)
    except ValueError:
        return None
    return valor if limite.contains(valor) else None


def _pedir_angulos(
    limites: list[proto.JointLimit], nombres: list[str]
) -> list[float] | None:
    """Captura los cuatro ángulos, uno a uno, como Menu_ProcessAngle().

    Devuelve None si el usuario cancela con ENTER vacío.
    """
    ui.show_header("INGRESO MANUAL DE ÁNGULOS")
    print_dim("Grados; acepta '.' o ','. ENTER vacío cancela.")
    console.print()

    angulos: list[float] = []
    for i, limite in enumerate(limites):
        while True:
            ui.prompt_angulo(i, len(limites), nombres[i], limite)
            texto = _leer_linea()
            if texto == "":
                return None
            valor = _parse_angulo(texto, limite)
            if valor is None:
                ui.show_error("Valor inválido o fuera de rango")
                continue
            angulos.append(valor)
            break
    return angulos


def _opcion_secuencia(
    robot: Robot, limites: list[proto.JointLimit], nombres: list[str]
) -> None:
    """Opción 1: ejecutar la secuencia automática del firmware."""
    print_info("Iniciando secuencia...")
    robot.sequence_start()

    try:
        while True:
            sec = robot.sequence_poll()
            if not sec.running:
                break
            ui.show_sequence(sec.step, sec.total)
            ui.show_arm_state(robot.status(), limites, nombres)
    except (KeyboardInterrupt, click.Abort):
        print_warning("Secuencia detenida")
        robot.sequence_stop()
        return

    print_success("Secuencia completada")


def _opcion_angulos(
    robot: Robot, limites: list[proto.JointLimit], nombres: list[str]
) -> None:
    """Opción 2: ingresar ángulos manualmente y mover."""
    angulos = _pedir_angulos(limites, nombres)
    if angulos is None:
        return

    ui.show_header("MOVIMIENTO MANUAL")
    ui.show_moviendo("MANUAL")
    try:
        estado = robot.move(angulos)
    except proto.ProtocolError as e:
        ui.show_error(f"No se pudo ejecutar el movimiento: {e}")
        _esperar_tecla()
        return

    ui.show_ok()
    ui.show_arm_state(estado, limites, nombres)
    _esperar_tecla()


def _opcion_home(
    robot: Robot, limites: list[proto.JointLimit], nombres: list[str]
) -> None:
    """Opción 3: ir a la posición HOME."""
    ui.show_header("POSICIÓN HOME")
    ui.show_moviendo("HOME")
    try:
        estado = robot.home()
    except proto.ProtocolError as e:
        ui.show_error(f"No se pudo ejecutar el movimiento: {e}")
        _esperar_tecla()
        return

    ui.show_ok()
    ui.show_arm_state(estado, limites, nombres)
    _esperar_tecla()


def _opcion_posicion(
    robot: Robot, limites: list[proto.JointLimit], nombres: list[str]
) -> None:
    """Opción 4: ver la posición actual."""
    ui.show_position(robot.status(), limites)
    _esperar_tecla()


OPCIONES = {
    "1": _opcion_secuencia,
    "2": _opcion_angulos,
    "3": _opcion_home,
    "4": _opcion_posicion,
}


def run(robot: Robot, info: proto.RobotInfo) -> None:
    """Bucle principal del menú interactivo."""
    try:
        limites = robot.limits()
    except (proto.ProtocolError, proto.TimeoutError_):
        limites = []

    nombres = info.joint_names or proto.DEFAULT_JOINT_NAMES

    while True:
        estado = robot.status()
        ui.show_menu(estado)

        ui.show_prompt()
        try:
            opcion = _leer_linea()
        except (KeyboardInterrupt, click.Abort):
            # Ctrl-C en el prompt es la forma natural de salir
            print_dim("Saliendo")
            break

        if opcion == "":
            # ENTER vacío vuelve a mostrar el menú, como en el firmware
            continue

        accion = OPCIONES.get(opcion)
        if accion is None:
            ui.show_error("Opción inválida")
            continue

        try:
            accion(robot, limites, nombres)
        except (KeyboardInterrupt, click.Abort):
            print_dim("Saliendo")
            break
        except (proto.ProtocolError, proto.TimeoutError_) as e:
            ui.show_error(str(e))