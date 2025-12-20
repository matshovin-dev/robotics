# Terminal History - Separat historikk per vindustittel
# Legg til i ~/.zshrc: source /Users/matsmac/vsCode/robotics/assets/scripts/keyboard_shortcuts/terminal_history.zsh

# Hent vindustittel fra Terminal.app
_get_terminal_title() {
    osascript -e 'tell application "Terminal" to get custom title of front window' 2>/dev/null
}

# Sett historikkfil basert på tittel-prefiks
_setup_history_by_title() {
    local title=$(_get_terminal_title)
    local prefix=$(echo "$title" | grep -oE '^[A-Za-z]+' | head -1)

    if [[ -n "$prefix" ]]; then
        export HISTFILE=~/.zsh_history_${prefix}
    else
        export HISTFILE=~/.zsh_history
    fi
}

# Funksjon for å sette tittel manuelt
title() {
    echo -ne "\033]0;$1\007"
    # Vent litt så Terminal får oppdatert tittelen
    sleep 0.1
    _setup_history_by_title
    echo "Tittel: $1 | Historikk: $HISTFILE"
}

# Historikk-innstillinger
HISTSIZE=10000
SAVEHIST=10000
setopt INC_APPEND_HISTORY
setopt HIST_IGNORE_DUPS
unsetopt SHARE_HISTORY

# Sett opp historikk ved oppstart
_setup_history_by_title

# Vis info
echo "Terminal historikk: $HISTFILE"
