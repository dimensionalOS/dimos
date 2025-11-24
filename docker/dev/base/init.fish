source /usr/share/autojump/autojump.fish
alias c clear
alias t tmux
alias g grep
alias f find
alias ka killall
alias ls 'exa --sort=modified --group-directories-first --icons'
alias l ls
alias lt 'ls --tree'
alias lg 'ls --grid --long'
alias grep 'grep --color=auto'
alias p 'ping -c 1 -w 1'
alias psac 'ps aux | ccze -Ao nolookups'
alias psa 'ps aux'
alias pdn 'p dns'
alias s 'sudo -iu root'
alias ccze 'ccze -o nolookups -A'

bind \cj __fzf_autojump

function fish_greeting
    echo -e "\033[34m"
    cat /etc/motd
    echo -e "\033[0m"
end
