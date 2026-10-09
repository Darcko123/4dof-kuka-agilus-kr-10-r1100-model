#! /usr/bin/env python3

import logging
import os
import sys

import serial

# Internas
from ..utils._version import __version__

# Externas
import click
from rich.logging import RichHandler
from rich.panel import Panel
from rich.table import Table

from ..utils.output import (
    console,
    STYLES,
    print_success,
    print_error,
    print_warning,
    print_info,
    print_dim,
    print_example,
)

from . import menu, protocol as proto, ui
from .robot import Robot
from .transport import list_devices

import platform
from pathlib import Path

# Información de la aplicación
VERSION_NUMBER = __version__
COMPANY = "Darcko123"
_FUNNY_PHRASES = [
    "¡Cuidado! Este robot tiene más personalidad que tu jefe.",
]

import random as _random

FUNNY_PHRASE = _random.choice(_FUNNY_PHRASES)

logger = logging.getLogger("rich")
FORMAT = "%(message)s"
logging.basicConfig(
    level="WARNING", format=FORMAT, datefmt="[%X]", handlers=[RichHandler(markup=True)]
)


def print_header(module=None):
    """Imprime el encabezado con arte ASCII"""
    if module:
        label = f"kukaagilus {module}"
    elif platform.system() != "Windows" and os.geteuid() == 0:
        label = "kukaagilus: (root)"
    else:
        label = "kukaagilus"

    ascii_art = f"""
           ###            |
          # ###           |
         # ### #          |
     ## # ##### # ##      |
    ##### ##### #####     |
   ######   #   ## ###    |
  #### ###  #  ### ####   |
  ##   ### ######    ##   |  {label}
 ###############      ##  |  v{VERSION_NUMBER}
 ### #########  ######### |  {FUNNY_PHRASE}
######################### |
   ############# #####    |
     #### ##########      |
        #########         | 
           ###            |
"""

    colored_ascii = f"[cyan bold]{ascii_art}[/cyan bold]"

    header_panel = Panel(
        colored_ascii,
        title=f"[cyan]{COMPANY}[/cyan]",
        border_style=STYLES["header"],
        title_align="left",
        padding=(1, 2),
    )
    console.print(header_panel)

# ============= Comandos de autocompletado para la shell =============


@click.group(name="kukaagilus")
def cli():
    """Control del brazo KUKA Agilus KR10 (4 GDL) sobre STM32F446

    La interfaz vive aquí; el firmware se ocupa del control, la cinemática y
    los servomotores. El enlace es USB-CDC.
    """
    pass


@click.group()
def completion():
    """
    Instalar el autocompletado de la CLI en bash, zsh o fish
    """
    pass


cli.add_command(completion)


@completion.command("install")
@click.option(
    "--shell",
    type=click.Choice(["bash", "zsh", "fish"]),
    default=None,
    help="Shell para instalar autocompletado (detectada automáticamente si se omite)",
)
def completion_install(shell):
    """
    """
    if platform.system() == "Windows":
        print_error("Shell completion is not supported on Windows.")
        sys.exit(1)

    import subprocess as _sp

    # Detecta la shell automáticamente
    if shell is None:
        shell_env = os.environ.get("SHELL", "")
        if "zsh" in shell_env:
            shell = "zsh"
        elif "fish" in shell_env:
            shell = "fish"
        elif "bash" in shell_env:
            shell = "bash"
        else:
            print_error("Could not detect shell. Use --shell bash|zsh|fish.")
            sys.exit(1)
        print_info(f"Detected shell: {shell}")

    env_var = "_KUKAAGILUS_COMPLETE"

    script_abs = str(Path(sys.argv[0]).resolve())
    python_abs = str(Path(sys.executable).resolve())
    cmd_to_call = f"{python_abs} {script_abs}"

    if shell == "bash":
        target = (
            Path.home()
            / ".local"
            / "share"
            / "bash-completion"
            / "completions"
            / "kukaagilus"
        )
        source_flag = "bash_source"
        rc_note = None
    elif shell == "zsh":
        target = Path.home() / ".zfunc" / "_kukaagilus"
        source_flag = "zsh_source"
        rc_note = "fpath=(~/.zfunc $fpath)\nautoload -Uz compinit && compinit"
    elif shell == "fish":
        target = Path.home() / ".config" / "fish" / "completions" / "kukaagilus.fish"
        source_flag = "fish_source"
        rc_note = None

    try:
        result = _sp.run(
            [python_abs, script_abs],
            env={**os.environ, env_var: source_flag},
            capture_output=True,
            text=True,
        )
        script = result.stdout
    except OSError as e:
        print_error(f"Failed to generate completion script: {e}")
        sys.exit(1)

    if not script.strip():
        print_error(
            "Empty completion script generated.\n"
            "Make sure you are running this command via:\n"
            f"  python {script_abs} completion install"
        )
        if result.stderr.strip():
            print_dim(result.stderr.strip())
        sys.exit(1)

    # -------------------------------------------------------------------- #
    # Posprocesado: sustituye el nombre de programa 'kukaagilus' que Click #
    # incrusta en el script por la invocación completa                     #
    # "python /ruta/abs/KUKAAgilus.py". Se cubren todos los patrones que   #
    # puede generar Click 7.x / 8.x.                                       #
    # -------------------------------------------------------------------- #
    if shell == "zsh":
        # 1. Directiva #compdef: se registra para todos los nombres que el
        #    usuario podría escribir
        script = script.replace(
            "#compdef kukaagilus", "#compdef kukaagilus KUKAAgilus.py ./KUKAAgilus.py"
        )
        # 2. La guarda que aborta cuando el comando no está en $commands[].
        #    Se neutraliza porque usamos una ruta absoluta, no una entrada del PATH.
        script = script.replace(
            "(( ! $+commands[kukaagilus] ))",
            "false",  # 'false' devuelve 1, así que el bloque (( )) nunca retorna
        )
        # 3. La línea que realmente llama al programa para obtener las sugerencias.
        #    Click 8 genera:  _KUKAAGILUS_COMPLETE=zsh_complete kukaagilus
        script = script.replace(
            f"{env_var}=zsh_complete kukaagilus", f"{env_var}=zsh_complete {cmd_to_call}"
        )
        # 4. El registro compdef al final del script
        script = script.replace(
            "compdef _kukaagilus_completion kukaagilus",
            "compdef _kukaagilus_completion kukaagilus KUKAAgilus.py ./KUKAAgilus.py",
        )

        # 5. Añade un envoltorio explícito para que "python KUKAAgilus.py <TAB>" y
        #    "./KUKAAgilus.py <TAB>" también activen el autocompletado. zsh compara
        #    con el último componente de $words[1], así que registramos un caso
        #    general que delega en nuestra función.
        extra = (
            "\n"
            "# Habilita el autocompletado al invocar 'python KUKAAgilus.py' o './KUKAAgilus.py'\n"
            "_kukaagilus_completion_python_wrapper() {\n"
            "  local script_name=${words[2]:t}  # nombre base del script pasado como argumento\n"
            "  if [[ $script_name == KUKAAgilus.py ]]; then\n"
            f"    (( ! $+functions[_kukaagilus_completion] )) && source {target}\n"
            '    words=(kukaagilus "${words[@]:2}")\n'
            "    (( CURRENT-- ))\n"
            "    _kukaagilus_completion\n"
            "  else\n"
            "    _files\n"
            "  fi\n"
            "}\n"
            "compdef _kukaagilus_completion_python_wrapper python python3\n"
        )
        script += extra

    elif shell == "bash":
        # Click <=8.0 genera:  _KUKAAGILUS_COMPLETE=bash_complete kukaagilus
        # Click >=8.1 genera:  _KUKAAGILUS_COMPLETE=bash_complete $1
        script = script.replace(
            f"{env_var}=bash_complete kukaagilus",
            f"{env_var}=bash_complete {cmd_to_call}",
        )
        script = script.replace(
            f"{env_var}=bash_complete $1",
            f"{env_var}=bash_complete {cmd_to_call}",
        )
        # Registra tanto 'kukaagilus' como 'KUKAAgilus.py' (Click 8.1 añade -o nosort)
        script = script.replace(
            "complete -F _kukaagilus_completion kukaagilus",
            "complete -F _kukaagilus_completion kukaagilus KUKAAgilus.py",
        )
        script = script.replace(
            "complete -o nosort -F _kukaagilus_completion kukaagilus",
            "complete -o nosort -F _kukaagilus_completion kukaagilus KUKAAgilus.py",
        )
        # Añade un envoltorio que intercepta 'python KUKAAgilus.py <TAB>'
        extra = (
            "\n"
            "# Habilita el autocompletado al invocar 'python KUKAAgilus.py'\n"
            "_kukaagilus_completion_python_wrapper() {\n"
            "    local cur script_arg\n"
            '    cur="${COMP_WORDS[COMP_CWORD]}"\n'
            '    script_arg="${COMP_WORDS[1]}"\n'
            '    if [[ "$(basename "$script_arg")" == "KUKAAgilus.py" ]]; then\n'
            "        # Reconstruye COMP_WORDS sin el 'python' / ruta inicial\n"
            '        local new_words=(kukaagilus "${COMP_WORDS[@]:2}")\n'
            '        COMP_WORDS=("${new_words[@]}")\n'
            "        COMP_CWORD=$(( COMP_CWORD - 1 ))\n"
            "        _kukaagilus_completion\n"
            "    fi\n"
            "}\n"
            "complete -F _kukaagilus_completion_python_wrapper python python3\n"
        )
        script += extra

    elif shell == "fish":
        # Fish usa otro mecanismo; basta con sustituir el nombre del programa.
        # Click <=8.0 lo coloca justo tras la variable de entorno; >=8.1, tras COMP_CWORD.
        script = script.replace(
            f"{env_var}=fish_complete kukaagilus",
            f"{env_var}=fish_complete {cmd_to_call}",
        )
        script = script.replace(
            "COMP_CWORD=(commandline -t) kukaagilus)",
            f"COMP_CWORD=(commandline -t) {cmd_to_call})",
        )
        # También autocompleta al invocar './KUKAAgilus.py'
        script += (
            "\ncomplete --no-files --command KUKAAgilus.py "
            '--arguments "(_kukaagilus_completion)"\n'
        )

    # Escribe el script
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(script, encoding="utf-8")
    except OSError as e:
        print_error(f"could not write completion script to {target}: {e}")
        sys.exit(1)
    print_success(f"Completion script written to: {target}")

    # zsh necesita una entrada fpath en .zshrc
    if rc_note:
        zshrc = Path.home() / ".zshrc"
        try:
            existing = zshrc.read_text(encoding="utf-8") if zshrc.exists() else ""
        except OSError as e:
            print_error(f"could not read {zshrc}: {e}")
            sys.exit(1)
        if "~/.zfunc" not in existing and ".zfunc" not in existing:
            try:
                if zshrc.exists():
                    backup = zshrc.with_name(".zshrc.bak-kukaagilus")
                    backup.write_text(existing, encoding="utf-8")
                with zshrc.open("a", encoding="utf-8") as f:
                    f.write(f"\n# autocompletado de kukaagilus\n{rc_note}\n")
            except OSError as e:
                print_error(f"could not update {zshrc}: {e}")
                sys.exit(1)
            print_success(f"Added fpath entry to {zshrc}")
        else:
            print_dim("~/.zfunc already in fpath — skipping .zshrc edit")

    console.print("")
    if shell == "bash":
        print_info("Restart your shell or run:")
        print_example(f"source {target}")
    elif shell == "zsh":
        print_info("Restart your shell or run:")
        print_example("source ~/.zshrc && compinit -u")
    elif shell == "fish":
        print_info("Completion is active immediately in new fish sessions.")


# ============= Conexión con el robot =============

_port_option = click.option(
    "--port",
    "-p",
    default=None,
    help="Puerto USB-CDC del STM32 (autodetectado si se omite)",
)


def _conectar(port: str | None) -> Robot:
    """Abre el enlace con el STM32 o aborta mostrando el motivo."""
    try:
        return Robot.connect(port)
    except ConnectionError as e:
        print_error(str(e))
        raise SystemExit(1)
    except serial.SerialException as e:
        print_error(f"Error de puerto serie: {e}")
        raise SystemExit(1)


def _fallo(e: Exception) -> None:
    """Traduce un fallo de protocolo a un mensaje de error y sale."""
    print_error(str(e))
    raise SystemExit(1)


@cli.command("ports")
def ports():
    """Lista los puertos USB-CDC disponibles"""
    found = list_devices()
    if not found:
        print_warning("No se encontró ningún puerto USB-CDC del STM32")
        return
    for device in found:
        console.print(f"  [green]{device}[/green]")


@cli.command("connect")
@_port_option
def connect(port):
    """Abre el enlace con el robot y muestra su identidad"""
    robot = _conectar(port)
    try:
        info = robot.info()
        estado = robot.status()
        puerto = robot.t.port
    except (proto.ProtocolError, proto.TimeoutError_) as e:
        _fallo(e)
    finally:
        robot.close()

    print_success(f"Conectado a {puerto}")
    print_info(
        f"{info.model} · firmware {info.version} · "
        f"{info.joints} GDL · OLED {'OK' if info.oled_ok else 'no disponible'}"
    )
    print_dim(f"Movimiento: {info.move_ms} ms · Secuencia: {info.seq_length} poses")
    print_dim("")
    print_dim(
        f"Efector: X {estado.x:.2f}  Y {estado.y:.2f}  Z {estado.z:.2f} mm  "
        f"Pitch {estado.pitch:.2f}°  Yaw {estado.yaw:.2f}°"
    )


@cli.command("status")
@_port_option
def status(port):
    """Muestra la configuración articular y la pose del efector"""
    robot = _conectar(port)
    try:
        estado = robot.status()
        limites = robot.limits()
    except (proto.ProtocolError, proto.TimeoutError_) as e:
        _fallo(e)
    finally:
        robot.close()

    ui.show_position(estado, limites)


@cli.command("home")
@_port_option
def home_cmd(port):
    """Lleva el brazo a la posición HOME"""
    robot = _conectar(port)
    try:
        ui.show_moviendo("HOME")
        estado = robot.home()
        limites = robot.limits()
    except (proto.ProtocolError, proto.TimeoutError_) as e:
        _fallo(e)
    finally:
        robot.close()

    print_success("Movimiento completado")
    ui.show_arm_state(estado, limites)


@cli.command("move")
@click.argument("angles", nargs=proto.NUM_JOINTS, type=float, required=True)
@_port_option
def move(angles, port):
    """Mueve el brazo a los ángulos articulares en GRADOS (Q1 Q2 Q3 Q4)"""
    robot = _conectar(port)
    try:
        ui.show_moviendo("MANUAL")
        estado = robot.move(list(angles))
        limites = robot.limits()
    except (proto.ProtocolError, proto.TimeoutError_) as e:
        _fallo(e)
    finally:
        robot.close()

    print_success("Movimiento completado")
    ui.show_arm_state(estado, limites)


@cli.command("sequence")
@click.option("--once", is_flag=True, default=False, help="Ejecuta la secuencia y termina")
@_port_option
def sequence(once, port):
    """Ejecuta la secuencia automática de prueba"""
    robot = _conectar(port)
    try:
        print_info("Iniciando secuencia...")
        robot.sequence_start()
        while True:
            sec = robot.sequence_poll()
            if not sec.running:
                break
            ui.show_sequence(sec.step, sec.total)
            if once:
                robot.sequence_stop()
                break
    except (KeyboardInterrupt, click.Abort):
        print_warning("Secuencia detenida")
        robot.sequence_stop()
        raise SystemExit(0)
    except (proto.ProtocolError, proto.TimeoutError_) as e:
        _fallo(e)
    finally:
        robot.close()

    print_success("Secuencia completada")


@cli.command("stop")
@_port_option
def stop(port):
    """Aborta el movimiento o la secuencia en curso"""
    robot = _conectar(port)
    try:
        estado = robot.stop()
        limites = robot.limits()
    except (proto.ProtocolError, proto.TimeoutError_) as e:
        _fallo(e)
    finally:
        robot.close()

    print_success("Movimiento detenido")
    ui.show_arm_state(estado, limites)


@cli.command("oled")
@click.argument("title")
@_port_option
def oled(title, port):
    """Cambia el título de la barra superior de la pantalla OLED"""
    robot = _conectar(port)
    try:
        robot.oled(title)
    except (proto.ProtocolError, proto.TimeoutError_) as e:
        _fallo(e)
    finally:
        robot.close()
    print_success(f"OLED: {title}")


@cli.command("menu")
@_port_option
def menu_cmd(port):
    """Menú interactivo con las cuatro opciones del robot"""
    robot = _conectar(port)
    try:
        menu.run(robot, robot.info())
    except (proto.ProtocolError, proto.TimeoutError_) as e:
        _fallo(e)
    finally:
        robot.close()


def main_cli() -> None:
    """Punto de entrada principal para la CLI"""

    if not os.environ.get("_KUKAAGILUS_COMPLETE"):
        module = next((a for a in sys.argv[1:] if not a.startswith("-")), None)
        print_header(module)

    # El nombre del programa debe ser "kukaagilus" para que las variables de
    # autocompletado (_KUKAAGILUS_COMPLETE) se generen con ese prefijo
    cli(prog_name="kukaagilus")