"""
test_ui.py - Renderiza las vistas de la interfaz sin hardware.

Imprime las mismas pantallas que mostraría el menú de terminal del firmware,
para comprobar el formato y detectar errores de renderizado.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rich.console import Console  # noqa: E402

from modules.core import protocol as proto, ui  # noqa: E402

# La consola se estrecha para simular una terminal de 80 columnas, que es
# donde el formato se aprieta
console = Console(width=80)
ui.console = console

estado = proto.ArmState(q=[45.0, 22.5, 12.0, 167.25], x=118.4, y=3.2, z=142.7, pitch=18.3, yaw=4.1)
limites = [
    proto.JointLimit(0, 0.0, 180.0),
    proto.JointLimit(1, 0.0, 180.0),
    proto.JointLimit(2, 0.0, 180.0),
    proto.JointLimit(3, 0.0, 180.0),
]

print("=" * 80)
ui.show_menu(estado)
print()
print("=" * 80)
ui.show_position(estado, limites)
print()
print("=" * 80)
ui.show_header("MOVIMIENTO MANUAL", oled_ok=False)
ui.show_moviendo("MANUAL")
ui.show_ok()
ui.show_arm_state(estado, limites)
ui.show_retorno()
print()
print("=" * 80)
ui.show_sequence(3, 5)
print()
print("=" * 80)

# Casos límite del rango articular
for angulo in (0.0, 45.0, 90.0, 135.0, 180.0):
    ui.show_arm_state(
        proto.ArmState(q=[angulo] * 4, x=0, y=0, z=0),
        limites,
    )
    print()