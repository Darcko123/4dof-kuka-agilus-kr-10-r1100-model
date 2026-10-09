#!/usr/bin/env bash
# Compila el proyecto usando el makefile generado por STM32CubeIDE 1.19.
# Uso: ./compile.sh [Debug|Release] [clean]
set -o pipefail

R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[34m'; C=$'\e[36m'; W=$'\e[1m'; N=$'\e[0m'

CONFIG="${1:-Debug}"
PROJ_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJ="$(basename "$PROJ_DIR")"
CUBE_DIR="${CUBE_DIR:-/opt/st/stm32cubeide_1.19.0}"
# Toolchain incluido en CubeIDE (13.3.rel1); si no está, se usa el del sistema
TOOLCHAIN="$(ls -d "$CUBE_DIR"/plugins/*gnu-tools-for-stm32*/tools/bin 2>/dev/null | tail -1)"
[ -n "$TOOLCHAIN" ] && export PATH="$TOOLCHAIN:$PATH"
MAKE_BIN="$(ls -d "$CUBE_DIR"/plugins/*make*/tools/bin 2>/dev/null | tail -1)"
[ -n "$MAKE_BIN" ] && export PATH="$MAKE_BIN:$PATH"

die() { echo -e "${R}${W}✖ $*${N}"; exit 1; }

echo -e "${C}${W}"
echo "╔══════════════════════════════════════════════════╗"
printf "║  %-48s║\n" "STM32 BUILD :: $PROJ"
echo "╚══════════════════════════════════════════════════╝${N}"
echo -e "${B}▸ Configuración:${N} $CONFIG"
echo -e "${B}▸ Compilador:   ${N} $(command -v arm-none-eabi-gcc || echo 'NO ENCONTRADO')"

command -v arm-none-eabi-gcc >/dev/null || die "arm-none-eabi-gcc no encontrado"
[ -f "$PROJ_DIR/$CONFIG/makefile" ] || die "No existe $CONFIG/makefile — compila una vez desde CubeIDE para generarlo"

cd "$PROJ_DIR/$CONFIG" || exit 1

if [ "$2" = "clean" ]; then
  echo -e "${Y}▸ Limpiando...${N}"
  make -s clean >/dev/null
fi

echo -e "${Y}▸ Compilando...${N}"
LOG="$(mktemp)"
make -j"$(nproc)" all 2>&1 | tee "$LOG" | while IFS= read -r line; do
  case "$line" in
    *error:*|*"***"*) echo -e "${R}${line}${N}" ;;
    *warning:*)       echo -e "${Y}${line}${N}" ;;
    *arm-none-eabi-gcc*) f="$(grep -o '\.\./[^ ]*\.[cs]' <<<"$line" | head -1)"; [ -n "$f" ] && echo -e "${G}  CC${N} ${f#../}" ;;
    *) ;;
  esac
done
RC=${PIPESTATUS[0]}

WARN=$(grep -c 'warning:' "$LOG"); ERR=$(grep -c 'error:' "$LOG"); rm -f "$LOG"

echo
if [ "$RC" -eq 0 ]; then
  echo -e "${W}Uso de memoria:${N}"
  arm-none-eabi-size "$PROJ.elf"
  echo -e "\n${G}${W}✔ BUILD OK${N}  (${Y}${WARN} warnings${N})  →  $CONFIG/$PROJ.elf"
else
  die "BUILD FALLÓ  (${ERR} errores, ${WARN} warnings)"
fi
