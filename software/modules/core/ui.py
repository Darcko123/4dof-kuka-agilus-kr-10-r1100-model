"""
ui.py - Vistas de la interfaz de usuario en Python.

Reproduce lo que mostraba el menú de terminal del firmware (encabezado, tabla
de articulaciones con barra de rango, pose del efector y progreso de la
secuencia), usando Rich en lugar de secuencias ANSI.
"""

from __future__ import annotations

from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from ..utils.output import STYLES, console
from . import protocol as proto

# Ancho de la barra de rango articular, igual que GAUGE_WIDTH del firmware
GAUGE_WIDTH = 21

TITULO = "KUKA AGILUS KR10  ·  Brazo robótico 4 GDL"
SUBTITULO = "Cinemática directa · Denavit-Hartenberg"

# Opciones del menú principal: mismas cuatro que el firmware, en el mismo orden
OPCIONES = [
    "Ejecutar secuencia automática",
    "Ingresar ángulos manualmente",
    "Ir a posición HOME",
    "Ver posición actual",
]

SEQUENCE_STEPS_TOTAL = 5


def show_header(titulo: str, *, oled_ok: bool = True) -> None:
    """Encabezado con el título de la sección, como Menu_ShowHeader()."""
    cuerpo = Text()
    cuerpo.append("    " + TITULO, style="cyan bold")
    cuerpo.append("\n    " + SUBTITULO, style="blue")

    console.print(
        Panel(
            cuerpo,
            title=f"[green bold]▸ {titulo}[/green bold]",
            title_align="left",
            border_style=STYLES["header"],
            padding=(1, 2),
        )
    )

    if not oled_ok:
        console.print("[yellow]⚠ Pantalla OLED no disponible[/yellow]")
    console.print()


def show_menu(estado: proto.ArmState) -> None:
    """Menú principal con las cuatro opciones y el resumen del efector."""
    show_header("MENÚ PRINCIPAL", oled_ok=estado.oled_ok)

    for i, opcion in enumerate(OPCIONES, 1):
        console.print(f"  [cyan bold][{i}][/cyan bold] {opcion}")

    console.print("[dim]" + "─" * 50 + "[/dim]")
    console.print(
        f"  [dim]Efector:[/dim] X [yellow]{estado.x:.1f}[/yellow]"
        f"  Y [yellow]{estado.y:.1f}[/yellow]"
        f"  Z [yellow]{estado.z:.1f}[/yellow] mm"
        f"  Pitch [yellow]{estado.pitch:.1f}°[/yellow]"
    )


def show_prompt() -> None:
    """Indicador de entrada del menú principal."""
    console.print("\n  [magenta bold]KR10 ›[/magenta bold] Opción [1-4]: ", end="")


def show_retorno() -> None:
    """Recordatorio de cómo volver al menú."""
    console.print("\n  [dim]↵ ENTER para volver al menú[/dim]")


def show_position(
    estado: proto.ArmState, limites: list[proto.JointLimit]
) -> None:
    """Configuración articular y pose del efector."""
    show_header("POSICIÓN ACTUAL", oled_ok=estado.oled_ok)
    show_arm_state(estado, limites)
    show_retorno()


def _gauge_index(valor: float, limite: proto.JointLimit) -> int:
    """Índice de q dentro de la barra de rango, como Menu_GaugeIndex()."""
    span = limite.max_deg - limite.min_deg
    if span <= 0:
        return 0
    t = (valor - limite.min_deg) / span
    t = max(0.0, min(1.0, t))
    return int(t * (GAUGE_WIDTH - 1) + 0.5)


def _gauge(valor: float, limite: proto.JointLimit) -> Text:
    """Barra de rango con la marca ● en la posición actual y ┼ en el cero."""
    pos = _gauge_index(valor, limite)
    zero = _gauge_index(0.0, limite)

    barra = Text()
    for k in range(GAUGE_WIDTH):
        if k == pos:
            barra.append("●", style="yellow bold")
        elif k == zero:
            barra.append("┼", style="dim")
        else:
            barra.append("─", style="dim")
    return barra


def show_arm_state(
    estado: proto.ArmState,
    limites: list[proto.JointLimit],
    nombres: list[str] | None = None,
) -> None:
    """Tabla de ángulos articulares con barra de rango y pose del efector."""
    if not limites:
        # El firmware no devolvió límites: se muestra la tabla sin barra
        limites = [
            proto.JointLimit(i, min(estado.q[i] - 90, -90), 90) for i in range(len(estado.q))
        ]
    if nombres is None:
        nombres = proto.DEFAULT_JOINT_NAMES

    tabla = Table(
        box=None,
        show_header=True,
        header_style="blue",
        padding=(0, 2),
        expand=False,
    )
    tabla.add_column("Articulación")
    tabla.add_column("Ángulo", justify="right")
    tabla.add_column("Rango")

    for i, q in enumerate(estado.q):
        nombre = nombres[i] if i < len(nombres) else f"Q{i + 1}"

        if i < len(limites):
            lim = limites[i]
            rango = Text(no_wrap=True)
            rango.append(f"{lim.min_deg:.0f}  ", style="dim")
            rango.append_text(_gauge(q, lim))
            rango.append(f"  {lim.max_deg:.0f}", style="dim")
        else:
            rango = Text("─" * GAUGE_WIDTH, style="dim")

        tabla.add_row(f"[cyan]Q{i + 1}[/cyan] {nombre}", f"[yellow]{q:7.1f}°[/yellow]", rango)

    console.print(tabla)

    # Efector final: se mantiene el formato de dos columnas del firmware
    def _linea_efector(etiqueta: str, valor: float, unidad: str, extra: str = "") -> Text:
        """Línea 'X <valor> mm <extra>' del bloque Efector final."""
        linea = Text("  ", style="")
        linea.append(f"{etiqueta:>2} ", style="")
        linea.append(f"{valor:>9.2f}", style="yellow")
        linea.append(f" {unidad}")
        if extra:
            linea.append(f"      {extra[0]:>5} ")
            linea.append(f"{extra[1]:>7.2f}°", style="yellow")
        return linea

    console.print("\n  [blue]Efector final[/blue]")
    console.print(_linea_efector("X", estado.x, "mm", ("Pitch", estado.pitch)))
    console.print(_linea_efector("Y", estado.y, "mm", ("Yaw", estado.yaw)))
    console.print(_linea_efector("Z", estado.z, "mm"))


def show_sequence(step: int, total: int) -> None:
    """Panel de la secuencia automática con el progreso de las poses."""
    progreso = "".join("● " if k < step else "○ " for k in range(total))

    show_header("SECUENCIA AUTOMÁTICA")
    console.print(f"  Pose [yellow]{step}/{total}[/yellow]   [green]{progreso}[/green]")
    console.print()
    console.print("  [dim]↵ ENTER para detener[/dim]")


def show_moviendo(titulo: str = "") -> None:
    """Aviso de que el robot está en movimiento."""
    sufijo = f" ({titulo})" if titulo else ""
    console.print(f"[blue]ℹ Moviendo robot{sufijo}...[/blue]")


def show_ok(mensaje: str = "Movimiento completado") -> None:
    """Confirmación de operación exitosa."""
    console.print(f"[green bold]✓ {mensaje}[/green bold]")


def show_error(mensaje: str) -> None:
    """Error legible para el usuario."""
    console.print(f"[red bold]✗ {mensaje}[/red bold]")


def prompt_angulo(
    indice: int,
    total: int,
    nombre: str,
    limite: proto.JointLimit,
) -> None:
    """Solicita el ángulo de una articulación, como Menu_PromptAngle()."""
    console.print(
        f"  [magenta bold][{indice + 1}/{total}][/magenta bold] "
        f"[cyan]Q{indice + 1}[/cyan] {nombre} "
        f"[dim]({limite.min_deg:.0f}° a {limite.max_deg:.0f}°)[/dim] "
        f"[magenta bold]›[/magenta bold] ",
        end="",
    )