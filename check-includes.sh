#!/bin/bash
# check-includes.sh – fungerer på macOS, Linux, zsh, bash, alt

root="${1:-.}"

ret=0

while IFS= read -r -d '' file; do
    echo "Checking $file"

    # 1. .c-fil skal inkludere sin egen .h først
    if [[ "$file" =~ \.c$ ]]; then
        base="${file##*/}"          # filnavn uten sti
        base="${base%.c}"           # fjern .c
        expected="#include \"$base.h\""
        first_include=$(grep -m1 '^#include' "$file" || true)
        if [[ -f "${file%.c}.h" ]] && [[ "$first_include" != "$expected" ]]; then
            echo "ERROR: $file må starte med $expected"
            ret=1
        fi
    fi

    # 2. Advarsel for <> på noe som ser ut som vårt eget bibliotek
    grep '^#include <' "$file" | grep -E '(robotics|cutils|common|utils|myproject)' > /dev/null && {
        echo "WARNING: Bruk \"\" ikke <> for egne biblioteker: $file"
        ret=1
    }

done < <(find "$root" -name "*.c" -print0 -o -name "*.h" -print0)

exit $ret