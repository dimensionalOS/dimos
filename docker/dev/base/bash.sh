#!/bin/bash
# history
shopt -s histappend
export HISTCONTROL="ignoredups"
export HISTSIZE=100000
export HISTFILESIZE=100000
export HISTIGNORE='ls'

# basic vars
export EDITOR="nano"
export LESS='-R'

# basic aliases
alias ta='tmux a'
alias ccze='ccze -o nolookups -A'
alias pd='p d'
alias t='tmux'
alias g='grep'
alias f='find'
alias ..="cd .."
alias ka="killall"
alias la="ls -al"
alias l="ls"
alias sl="ls"
alias ls="ls --color"
alias c="clear"
alias psa="ps aux"
alias grep="grep --color=auto"
alias p="ping -c 1 -w 1"
alias psg="ps aux | grep"
alias unitg="systemctl list-unit-files | grep"
alias ug="unitg"
alias unit="echo 'systemctl list-unit-files'; systemctl list-unit-files"
alias scr="echo 'sudo systemctl daemon-reload'; sudo systemctl daemon-reload"
alias psac="ps aux | ccze -Ao nolookups"
alias psa="ps aux"
alias pdn="p dns"
alias s="sudo -iu root"
alias m="mount"
alias oip="wget -qO-  http://www.ipaddr.de/?plain"
alias getlogin="echo genpass 6 : genpass 20"
alias rscp="rsync -vrt --size-only  --partial --progress "
alias rscpd="rsync --delete-after -vrt --size-only  --partial --progress "
alias v="vim"
alias npm="export PYTHON=python2; npm"
alias ssh="ssh -o ConnectTimeout=1"
alias gp="git push"
alias rh="history -a; history -c; history -r"
alias gs="git status"
alias gd="git diff"
alias ipy="python -c 'import IPython; IPython.terminal.ipapp.launch_new_instance()'"

function npmg
{
    echo 'global npm install'
    tmpUmask u=rwx,g=rx,o=rx npm $@
}

function tmpUmask
{
    oldUmask=$(umask)
    newUmask=$1
    
    shift
    umask $newUmask
    echo umask $(umask -S)
    echo "$@"
    eval $@
    umask $oldUmask
    echo umask $(umask -S)
    
}

function newloginuser
{
    read user
    pass=$(genpass 20)

    echo $user : $pass
    echo site?
    read site
    echo site: $site

    echo $site : $user : $pass >> ~/.p
}

function newlogin
{
    user=$(genpass 6)
    pass=$(genpass 20)

    echo $user : $pass
    echo site?
    read site
    echo site: $site

    echo $site : $user : $pass >> ~/.p

}


function newlogin
{
    pass=$(genpass 30)
    echo $pass
}


function getpass {
  echo $(genpass 20)
}

function genpass
{
  newpass=$(cat /dev/urandom | base64 | tr -d "0" | tr -d "y"  | tr -d "Y" | tr -d "z"  | tr -d "Z" | tr -d "I" | tr -d "l" | tr -d "//" | head -c$1)
  echo -n $newpass
}

function sx
{
    if [ -z $1 ]
    then
        screen -x $(cat /tmp/sx)
    else
        echo -n $1 > /tmp/sx
        screen -x $1
    fi
}

function loopy
{
    while [ 1 ]; do
        eval "$1"
        if [ "$2" ]; then sleep $2; else sleep 1; fi
    done
}


function we
{
  eval "$@"
  until [ $? -eq 0 ]; do
    sleep 1; eval "$@"
  done
}

alias wf='waitfor'
function waitfor
{
  eval "$1"
  until [ $? -eq 0 ]; do
    sleep 1; eval "$1"
  done
  eval "$2"
}

function waitnot
{
  eval "$1"
  until [ $? -ne 0 ]; do
    sleep 1; eval "$1"
  done
  eval "$2"
}

function wrscp
{
    echo rscp $@
    waitfor "rscp $1 $2"
}

function waitfornot
{
  eval "$1"
  until [ $? -ne 0 ]; do
    sleep 1
    eval "$1"
  done
  eval "$2"
}


function watchFile
{
    tail -F $1 2>&1 | sed -e "$(echo -e "s/^\(tail: .\+: file truncated\)$/\1\e[2J \e[0f/")"
}

PS1='${debian_chroot:+($debian_chroot)}\[\033[32m\]\u@dimos\[\033[00m\]:\[\033[34m\]\w\[\033[00m\] \$ '
