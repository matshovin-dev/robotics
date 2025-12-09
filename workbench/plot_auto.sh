#!/bin/bash
# Automatisk hot-reload plotter
cd "$(dirname "$0")"

FILE="wb_plotter_live.c"

echo "=== AUTO-RELOAD PLOTTER ==="
echo "Overvåker: $FILE"
echo "Lagre filen for å oppdatere grafen"
echo ""

pkill -f "./wb_plotter_live" 2>/dev/null

trap 'pkill -f "./wb_plotter_live" 2>/dev/null; exit 0' INT TERM

echo "Kompilerer..."
make wb_plotter_live && ./wb_plotter_live &

LAST=$(stat -f %m "$FILE")

while true; do
    sleep 0.5
    NOW=$(stat -f %m "$FILE")
    if [ "$NOW" != "$LAST" ]; then
        LAST="$NOW"
        echo "Oppdaterer..."
        pkill -f "./wb_plotter_live" 2>/dev/null
        sleep 0.1
        make wb_plotter_live && ./wb_plotter_live &
    fi
done
