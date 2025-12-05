#!/bin/bash

# Script for å oppdatere robotics tree struktur i markdown

cd /Users/matsmac/vsCode/robotics

# Generer tree output
TREE_OUTPUT=$(tree -I 'html|latex|*.o|*.d|.git|build')

# Lag markdown fil med farger og CSS styling
cat > /Users/matsmac/vsCode/docs/notes/tre.md << 'HEADER'
# Robotics Project Structure

<style>
body { font-size: 12px; }
pre { font-size: 12px; }
</style>

<pre style="font-family: 'Courier New', monospace; background: #1e1e1e; padding: 15px; color: #d4d4d4; line-height: 1.4;">
HEADER

# Prosesser hver linje og legg til farger
echo "$TREE_OUTPUT" | while IFS= read -r line; do
    # Mapper (blå, bold)
    if [[ "$line" =~ ^(.*)([├└│]──[[:space:]])([^/]+)/$ ]]; then
        prefix="${BASH_REMATCH[1]}${BASH_REMATCH[2]}"
        name="${BASH_REMATCH[3]}"
        echo "${prefix}<span style=\"color: #569cd6; font-weight: bold;\">${name}</span>/"
    # .c filer (grønn)
    elif [[ "$line" =~ \.c$ ]]; then
        echo "$line" | sed -E 's/([^[:space:]]+\.c)$/<span style="color: #4ec9b0;">\1<\/span>/'
    # .h filer (cyan)
    elif [[ "$line" =~ \.h$ ]]; then
        echo "$line" | sed -E 's/([^[:space:]]+\.h)$/<span style="color: #9cdcfe;">\1<\/span>/'
    # Makefile (gul)
    elif [[ "$line" =~ Makefile$ ]]; then
        echo "$line" | sed -E 's/(Makefile)$/<span style="color: #dcdcaa;">\1<\/span>/'
    # .md og .txt filer (oransje)
    elif [[ "$line" =~ \.(md|txt)$ ]]; then
        echo "$line" | sed -E 's/([^[:space:]]+\.(md|txt))$/<span style="color: #ce9178;">\1<\/span>/'
    # .sh filer (grønn)
    elif [[ "$line" =~ \.sh$ ]]; then
        echo "$line" | sed -E 's/([^[:space:]]+\.sh)$/<span style="color: #4ec9b0;">\1<\/span>/'
    # Executables (magenta) - filer uten extension
    elif [[ "$line" =~ (viz-stewart|viz-stewart-compare|viz-stewart-kinematics)$ ]]; then
        echo "$line" | sed -E 's/(viz-[^[:space:]]+)$/<span style="color: #c586c0; font-weight: bold;">\1<\/span>/'
    # Statistikk linje (grå)
    elif [[ "$line" =~ ^[0-9]+[[:space:]]+(directories|files) ]]; then
        echo "<span style=\"color: #808080;\">$line</span>"
    else
        echo "$line"
    fi
done >> /Users/matsmac/vsCode/docs/notes/tre.md

# Avslutt HTML
cat >> /Users/matsmac/vsCode/docs/notes/tre.md << 'FOOTER'
</pre>

---

## Farge-nøkkel:

- <span style="color: #569cd6; font-weight: bold;">Blå (bold)</span> - Mapper
- <span style="color: #4ec9b0;">Grønn</span> - C source filer (.c)
- <span style="color: #9cdcfe;">Cyan</span> - Header filer (.h)
- <span style="color: #dcdcaa;">Gul</span> - Makefiles
- <span style="color: #ce9178;">Oransje</span> - Dokumentasjon (.md, .txt)
- <span style="color: #c586c0; font-weight: bold;">Magenta (bold)</span> - Executables

---

**Tips:** Trykk `Cmd+Shift+V` for preview med farger!

*Sist oppdatert: $(date '+%Y-%m-%d %H:%M')*
FOOTER

echo "✅ Tree oppdatert i docs/notes/robotics-colored.md"
