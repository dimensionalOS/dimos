#!/usr/bin/env bash
# The installer integration harness: one musl binary, one ubuntu:22.04 container, three phases.
#
#   docker run --rm --platform linux/arm64 \
#     -v "$PWD/dist/dimos-aarch64-linux-musl:/usr/local/bin/dimos:ro" \
#     -v "$PWD/installer/tests/container.sh:/t.sh:ro" \
#     -e DIMOS_BRANCH=aaryan/installer [-e DIMOS_TEST_EXTRAS=base] \
#     [-e DIMOS_INSTALLER_URL=http://<host>:8000] ubuntu:22.04 bash /t.sh
#
# Root installs the prerequisites and two users, then re-enters itself as each of them:
# `tester` has NOPASSWD sudo and runs the full install, `nosudo` has none and must exit 2.

# shellcheck disable=SC2329  # every check is invoked by name through run_check
set -u

SELF=$(readlink -f "$0")
MOUNTED_BIN=/usr/local/bin/dimos
BRANCH=${DIMOS_BRANCH:-aaryan/installer}
EXTRAS=${DIMOS_TEST_EXTRAS:-base}
UV_CACHE=/var/cache/dimos-uv
WORK=/tmp/dimos-test-$(id -un)
DIR=$HOME/dimos
BIN=$HOME/.local/bin/dimos
failures=0

main() {
  case "${DIMOS_TEST_PHASE:-root}" in
  root) root_phase ;;
  user) user_phase ;;
  nosudo) nosudo_phase ;;
  *)
    echo "xx   unknown DIMOS_TEST_PHASE: ${DIMOS_TEST_PHASE}, want root|user|nosudo"
    exit 1
    ;;
  esac
}

root_phase() {
  apt_prereqs
  mkdir -p "$UV_CACHE"
  chmod 777 "$UV_CACHE" # shared between the two users so the sudo-less rerun reuses the wheels
  useradd -m -s /bin/bash tester
  printf 'tester ALL=(ALL) NOPASSWD:ALL\n' >/etc/sudoers.d/tester
  chmod 440 /etc/sudoers.d/tester
  useradd -m -s /bin/bash nosudo
  as_user tester user
  as_user nosudo nosudo
  finish
}

apt_prereqs() {
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  apt-get install -y -qq sudo curl git ca-certificates util-linux
}

# runuser takes an argv vector, so nothing is pasted into a shell string; stdin is /dev/null so
# `docker run -t` can never hand sudo a TTY and hide the sudo-less path this harness exists to test.
as_user() {
  local user=$1 phase=$2 rc
  runuser -u "$user" -- env \
    HOME="/home/$user" USER="$user" LOGNAME="$user" \
    DIMOS_TEST_PHASE="$phase" DIMOS_BRANCH="$BRANCH" DIMOS_TEST_EXTRAS="$EXTRAS" \
    DIMOS_INSTALLER_URL="${DIMOS_INSTALLER_URL:-}" UV_CACHE_DIR="$UV_CACHE" \
    bash "$SELF" </dev/null
  rc=$?
  [ "$rc" -eq 0 ] || failures=$((failures + 1))
}

user_phase() {
  mkdir -p "$WORK"
  touch "$WORK/marker"
  record dry "$MOUNTED_BIN" setup --dry-run --non-interactive --mode dev \
    --branch "$BRANCH" --extras "$EXTRAS" --dir "$DIR"
  run_check dry_run_exits_0_without_a_consent_prompt
  run_check dry_run_creates_nothing_outside_the_action_log
  record install "$MOUNTED_BIN" setup --non-interactive --yes --agent --mode dev \
    --branch "$BRANCH" --extras "$EXTRAS" --dir "$DIR"
  run_check first_run_exits_0_and_leaves_the_binary_and_the_state_file
  run_check missing_systemd_is_a_note_not_a_failure
  run_check the_venv_imports_dimos
  record second "$BIN" setup --non-interactive --yes --agent
  run_check second_run_applies_nothing
  run_check forwarding_reaches_the_venv_dimos
  run_check the_venv_dimos_forwards_setup_back_to_the_installer
  run_check service_setup_without_systemd_exits_1
  run_check update_dry_run_leaves_the_installed_binary_alone
  finish
}

nosudo_phase() {
  mkdir -p "$WORK"
  # base, not $EXTRAS: the sudo message is what is under test, so buy the cheapest install that reaches it.
  record nosudo "$MOUNTED_BIN" setup --non-interactive --mode dev \
    --branch "$BRANCH" --extras base --dir "$DIR"
  run_check a_sudoless_run_exits_2_naming_both_sudo_env_vars
  run_check a_sudoless_run_still_installs_dimos_because_uv_needs_no_root
  finish
}

# No `set -e`: a non-zero exit is an assertion input here, never a reason to abort.
record() {
  local tag=$1
  shift
  "$@" >"$WORK/$tag.out" 2>"$WORK/$tag.err" </dev/null
  echo $? >"$WORK/$tag.rc"
}

rc_of() {
  cat "$WORK/$1.rc"
}

run_check() {
  if "$1"; then
    echo "ok   $1"
  else
    echo "xx   $1"
    failures=$((failures + 1))
  fi
}

finish() {
  if [ "$failures" -eq 0 ]; then
    echo "ok   ${DIMOS_TEST_PHASE:-root}: every check passed"
    exit 0
  fi
  echo "xx   ${DIMOS_TEST_PHASE:-root}: $failures check(s) failed"
  exit 1
}

dry_run_exits_0_without_a_consent_prompt() {
  [ "$(rc_of dry)" -eq 0 ] || {
    tail -20 "$WORK/dry.err"
    return 1
  }
}

# ~/.local is pruned because the action log lives there by design; the two paths it could hide are
# asserted absent instead.
dry_run_creates_nothing_outside_the_action_log() {
  find / -xdev \( -path /proc -o -path /sys -o -path /dev -o -path /run \
    -o -path /tmp -o -path /var/tmp -o -path "$HOME/.local" \) -prune \
    -o -newer "$WORK/marker" -print >"$WORK/diff.txt"
  if [ -s "$WORK/diff.txt" ]; then
    head -20 "$WORK/diff.txt"
    return 1
  fi
  [ ! -e "$DIR" ] && [ ! -e "$BIN" ]
}

first_run_exits_0_and_leaves_the_binary_and_the_state_file() {
  [ "$(rc_of install)" -eq 0 ] || {
    tail -20 "$WORK/install.err"
    return 1
  }
  [ -x "$BIN" ] || return 1
  grep -q '"mode": "dev"' "$HOME/.config/dimos/installer.json"
}

missing_systemd_is_a_note_not_a_failure() {
  grep -q "no systemd" "$WORK/install.out" || return 1
  ! grep -q '"failed"' "$WORK/install.out"
}

the_venv_imports_dimos() {
  "$DIR/.venv/bin/python" -c "import dimos" </dev/null
}

second_run_applies_nothing() {
  [ "$(rc_of second)" -eq 0 ] || {
    tail -20 "$WORK/second.err"
    return 1
  }
  tail -1 "$WORK/second.out" | grep -o '\["[a-z_ -]*","applied"\]' |
    head -20 >"$WORK/applied.txt"
  [ ! -s "$WORK/applied.txt" ] || {
    cat "$WORK/applied.txt"
    return 1
  }
}

forwarding_reaches_the_venv_dimos() {
  record list "$BIN" list
  [ "$(rc_of list)" -eq 0 ] && [ -s "$WORK/list.out" ]
}

the_venv_dimos_forwards_setup_back_to_the_installer() {
  record fwd "$DIR/.venv/bin/dimos" setup --help
  grep -q "Install DimOS and bring up robot hardware" "$WORK/fwd.out"
}

service_setup_without_systemd_exits_1() {
  record svc "$BIN" service setup unitree-go2 --non-interactive
  [ "$(rc_of svc)" -eq 1 ] && grep -qi systemd "$WORK/svc.err"
}

update_dry_run_leaves_the_installed_binary_alone() {
  if [ -z "${DIMOS_INSTALLER_URL:-}" ]; then
    echo "     skipped: set DIMOS_INSTALLER_URL to the release server to cover self-update"
    return 0
  fi
  local before
  before=$(sha256sum "$BIN")
  record update "$BIN" update --dry-run --non-interactive
  [ "$(rc_of update)" -eq 0 ] && [ "$before" = "$(sha256sum "$BIN")" ]
}

a_sudoless_run_exits_2_naming_both_sudo_env_vars() {
  [ "$(rc_of nosudo)" -eq 2 ] || {
    tail -20 "$WORK/nosudo.err"
    return 1
  }
  grep -q SUDO_ASKPASS "$WORK/nosudo.err" && grep -q DIMOS_SUDO_PASSWORD "$WORK/nosudo.err"
}

a_sudoless_run_still_installs_dimos_because_uv_needs_no_root() {
  [ -x "$DIR/.venv/bin/python" ]
}

main
