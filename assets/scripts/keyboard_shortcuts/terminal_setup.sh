#!/bin/bash
# Terminal Setup - Starter terminaler med titler og tilhørende historikk
# Legg til i ~/.zshrc: source /Users/matsmac/vsCode/robotics/assets/scripts/keyboard_shortcuts/terminal_history.zsh

# Åpne ny terminal med tittel
open_terminal() {
    local title="$1"
    local dir="${2:-$HOME}"

    osascript <<EOF
tell application "Terminal"
    activate
    set newTab to do script "cd '$dir'; source ~/.zshrc"
    set custom title of front window to "$title"
end tell
EOF
}

# Eksempel: Start forhåndsdefinerte terminaler
case "$1" in
    "all")
        open_terminal "B: Build" "$HOME/vsCode/robotics"
        sleep 0.5
        open_terminal "C: Code" "$HOME/vsCode/robotics"
        sleep 0.5
        open_terminal "Ste: Stepper" "$HOME/vsCode/robotics"
        ;;
    "B")
        open_terminal "B: Build" "$HOME/vsCode/robotics"
        ;;
    "C")
        open_terminal "C: Code" "$HOME/vsCode/robotics"
        ;;
    "Ste")
        open_terminal "Ste: Stepper" "$HOME/vsCode/robotics"
        ;;
    *)
        echo "Bruk: $0 [all|B|C|Ste]"
        echo "  all - Åpne alle tre terminalene"
        echo "  B   - Åpne Build terminal"
        echo "  C   - Åpne Code terminal"
        echo "  Ste - Åpne Stepper terminal"
        ;;
esac
