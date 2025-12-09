#!/bin/bash
# Enkel plotter med manuell reload
# Trykk Enter for å oppdatere grafen etter du har lagret filen

cd "$(dirname "$0")"

echo "=== PLOTTER ==="
echo "Trykk ENTER for å rekompilere og oppdatere grafen"
echo "Trykk Ctrl+C for å avslutte"
echo ""

pkill -f "./wb_plotter_live" 2>/dev/null

run_plotter() {
    pkill -f "./wb_plotter_live" 2>/dev/null
    sleep 0.1
    echo "Kompilerer..."
    if make wb_plotter_live 2>&1 | tail -1; then
        ./wb_plotter_live &
        echo "Kjører!"
    else
        echo "Feil!"
    fi
    echo ""
}

trap 'pkill -f "./wb_plotter_live" 2>/dev/null; exit 0' INT TERM

run_plotter

while true; do
    read -r
    run_plotter
done
