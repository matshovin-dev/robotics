#!/bin/bash
# Hot-reload plotter
# Bruk: ./plot_watch_simple.sh

cd "$(dirname "$0")"

FILE="wb_plotter_live.c"

echo "=== HOT-RELOAD PLOTTER ==="
echo "Overvåker: $FILE"
echo "Lagre filen for å oppdatere grafen"
echo "Trykk Ctrl+C for å avslutte"
echo ""

# Drep eventuelle gamle prosesser
pkill -f "wb_plotter_live" 2>/dev/null

cleanup() {
    echo ""
    echo "Avslutter..."
    pkill -f "wb_plotter_live" 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

# Kompiler første gang
echo "Kompilerer..."
make wb_plotter_live || exit 1

# Start plotter
./wb_plotter_live &

# Hent initial timestamp
LAST=$(stat -f %m "$FILE")
echo "Startet! Venter på endringer..."
echo ""

# Overvåk
while true; do
    sleep 0.5
    NOW=$(stat -f %m "$FILE" 2>/dev/null)

    if [[ "$NOW" != "$LAST" ]]; then
        # Debounce: vent til endringene har stoppet
        sleep 0.5
        LAST=$(stat -f %m "$FILE" 2>/dev/null)
        echo "Endring! Rekompilerer..."

        pkill -f "wb_plotter_live" 2>/dev/null
        sleep 0.1

        if make wb_plotter_live 2>&1; then
            ./wb_plotter_live &
            echo "OK - graf oppdatert"
        else
            echo "FEIL ved kompilering"
        fi
    fi
done
