# Source in Bash or Zsh; activation never installs anything.
if [ -n "${ZSH_VERSION:-}" ]; then
    _dimos_source=${(%):-%x}
else
    _dimos_source=${BASH_SOURCE[0]}
fi
_dimos_root=$(cd "$(dirname "$_dimos_source")/.." && pwd -P) || return
if [ ! -x "$_dimos_root/.venv/bin/python" ]; then
    echo 'Application environment is missing. Run uv sync --locked first.' >&2
    unset _dimos_source _dimos_root
    return 1
fi
if typeset -f deactivate >/dev/null 2>&1; then deactivate; fi
_dimos_exports=$("$_dimos_root/.venv/bin/python" "$_dimos_root/.dimos/environment.py" "$_dimos_root")
_dimos_status=$?
if [ "$_dimos_status" -eq 0 ]; then eval "$_dimos_exports"; fi
unset _dimos_source _dimos_root _dimos_exports
if [ "$_dimos_status" -ne 0 ]; then unset _dimos_status; return 1; fi
unset _dimos_status
