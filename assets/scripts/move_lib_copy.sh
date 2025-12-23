#!/bin/bash
#
# move_lib_copy.sh - Kopier moves fra en range til en annen i move_lib.json
#
# Bruk: ./move_lib_copy.sh /move_lib.json
#
# Kommenter ut/inn linjer under for å velge hvilke moves som skal kopieres


START_SRC=9
END_SRC=13
START_DST=14

# ============================================

if [ $# -lt 1 ]; then
    echo "Bruk: $0 <move_lib.json>"
    exit 1
fi

INPUT_FILE="$1"
BACKUP_FILE="${INPUT_FILE}.bak"
TEMP_FILE="${INPUT_FILE}.tmp"

if [ ! -f "$INPUT_FILE" ]; then
    echo "Feil: Filen $INPUT_FILE finnes ikke"
    exit 1
fi

# Sjekk at jq er installert
if ! command -v jq &> /dev/null; then
    echo "Feil: jq er ikke installert. Installer med: brew install jq"
    exit 1
fi

# Beregn antall moves som skal kopieres
COUNT=$((END_SRC - START_SRC + 1))
END_DST=$((START_DST + COUNT - 1))

echo "Kopierer moves $START_SRC-$END_SRC til $START_DST-$END_DST"
echo "Backup lagres i: $BACKUP_FILE"

# Lag backup
cp "$INPUT_FILE" "$BACKUP_FILE"

# Bruk jq til å kopiere moves
jq --argjson start_src "$START_SRC" \
   --argjson end_src "$END_SRC" \
   --argjson start_dst "$START_DST" '
.moves |= (
    # For hver move i source range, kopier til destination
    reduce range($start_src; $end_src + 1) as $src_idx (.;
        # Beregn destination index
        ($start_dst + ($src_idx - $start_src)) as $dst_idx |
        # Hent source move og oppdater index
        (.[$src_idx] | .index = $dst_idx) as $new_move |
        # Sett inn på destination
        .[$dst_idx] = $new_move
    )
)
' "$INPUT_FILE" > "$TEMP_FILE"

if [ $? -eq 0 ]; then
    mv "$TEMP_FILE" "$INPUT_FILE"
    echo "Ferdig! Moves kopiert."

    # Vis hva som ble kopiert
    echo ""
    echo "Kopierte moves:"
    for i in $(seq $START_SRC $END_SRC); do
        dst=$((START_DST + (i - START_SRC)))
        name=$(jq -r ".moves[$dst].name" "$INPUT_FILE")
        echo "  $i -> $dst: \"$name\""
    done
else
    echo "Feil under prosessering av JSON"
    rm -f "$TEMP_FILE"
    exit 1
fi
