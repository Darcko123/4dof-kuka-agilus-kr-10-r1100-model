"""
output.py - Consola Rich compartida y utilidades de salida para todos los módulos
"""

from __future__ import annotations

import re
import time
from typing import Callable, Optional

from rich.console import Console, Group
from rich.errors import MarkupError
from rich.padding import Padding
from rich.panel import Panel
from rich.style import Style
from rich.table import Table

STYLES = {
    "header": Style(color="cyan", bold=True),
    "success": Style(color="green", bold=True),
    "warning": Style(color="yellow", bold=True),
    "error": Style(color="red", bold=True),
    "info": Style(color="blue", bold=True),
    "dim": Style(dim=True),
    "prompt": Style(color="magenta", bold=True),
    "device": Style(color="cyan"),
}

console = Console()
console_err = Console(stderr=True)

_REDACT_RE = re.compile(r"^(set\s+pass\b)\s+\S+(.*)$", re.IGNORECASE)


def make_tracer(
    level: int, t0: Optional[float] = None
) -> Optional[Callable[[str, str], None]]:
    if level <= 0:
        return None
    start = t0 if t0 is not None else time.monotonic()

    def trace(direction: str, text: str) -> None:
        text = _REDACT_RE.sub(r"\1 (redacted)\2", text)
        arrow = ">" if direction == "tx" else "<"
        style = "cyan" if direction == "tx" else "dim"
        prefix = f"{arrow} "
        if level >= 2:
            elapsed = time.monotonic() - start
            nbytes = len(text.encode("ascii", "replace"))
            prefix = f"{arrow} [{elapsed:8.3f}s] ({nbytes:>3}B) "
        console_err.print(f"[{style}]{prefix}{text}[/{style}]")

    return trace


def safe_print(text: str, *, err: bool = False, **kwargs) -> None:
    out = console_err if err else console
    try:
        out.print(text, **kwargs)
    except MarkupError:
        out.print(text, markup=False, **kwargs)


def print_success(message: str) -> None:
    safe_print(f"[green]✓[/green] {message}", style=STYLES["success"])


def print_warning(message: str) -> None:
    safe_print(f"[yellow]⚠[/yellow] {message}", style=STYLES["warning"], err=True)


def print_error(message: str) -> None:
    safe_print(f"[red]✗[/red] {message}", style=STYLES["error"], err=True)


def print_info(message: str) -> None:
    safe_print(f"[blue]ℹ[/blue] {message}", style=STYLES["info"])


def print_dim(message: str) -> None:
    safe_print(f"  {message}", style=STYLES["dim"])


def print_subtitle(message: str) -> None:
    console.print(f"\n  [yellow]{message}[/yellow]")


def print_example(command: str, description: str = "") -> None:
    if description:
        # Alinea las descripciones manualmente si hace falta, o simplemente imprime
        console.print(f"  [green]{command}[/green] {description}")
    else:
        console.print(f"  [green]{command}[/green]")


def fmt_command(text: str) -> str:
    """Marca `text` como un comando listo para copiar y pegar (para usar dentro de mensajes)."""
    return f"[green]{text}[/green]"


def numbered_steps(steps: list[str]) -> Padding:
    grid = Table.grid(padding=(0, 1))
    grid.add_column(justify="right", style="white", no_wrap=True)
    grid.add_column(overflow="fold")
    for i, step in enumerate(steps, 1):
        grid.add_row(f"{i}.", re.sub(r"\n[ \t]+", "\n", step))
    return Padding(grid, (0, 0, 0, 2))


def print_error_panel(
    title: str,
    problem: str,
    why: str = "",
    fix: list[str] | None = None,
    fix_title: str = "How to fix it",
    notes: list[str] | None = None,
) -> None:
    body: list = [f"[red bold]✗  {problem}[/red bold]"]

    if why:
        body += ["", why]

    if fix:
        body += ["", f"[yellow bold]{fix_title}[/yellow bold]", "", numbered_steps(fix)]

    notes = [n for n in (notes or []) if n]
    if notes:
        body += [""] + [f"[dim]{note}[/dim]" for note in notes]

    console.print("")
    console.print(
        Panel(
            Group(*body),
            title=f"[red bold]{title}[/red bold]",
            title_align="left",
            border_style=STYLES["error"],
            padding=(1, 2),
            expand=False,
        )
    )
    console.print("")


def print_success_panel(
    title: str,
    summary: str,
    why: str = "",
    notes: list[str] | None = None,
) -> None:
    body: list = [f"[green bold]✓  {summary}[/green bold]"]

    if why:
        body += ["", why]

    for note in [n for n in (notes or []) if n]:
        body += ["", f"[dim]{note}[/dim]"]

    console.print("")
    console.print(
        Panel(
            Group(*body),
            title=f"[green bold]{title}[/green bold]",
            title_align="left",
            border_style=STYLES["success"],
            padding=(1, 2),
            expand=False,
        )
    )
    console.print("")
