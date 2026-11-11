#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0
# Exercise installer system configuration without changing the host.
set -euo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)

# Each case gets a fresh Bash process so errexit remains active in the installer.
if [[ "${1:-}" == --case ]]; then
    : "${TEST_DIR:?expected test directory from run_case}"
    source "$repo/scripts/install.sh"
    NON_INTERACTIVE="$TEST_NON_INTERACTIVE"
    DRY_RUN="$TEST_DRY_RUN"
    NO_SYSCTL=0
    DETECTED_OS=ubuntu

    ulimit() { printf '%s\n' "$TEST_HARD_LIMIT"; }
    sysctl() { printf '67108864\n'; }
    id() {
        case "$1" in
            -u) printf '1000\n' ;;
            -un) printf 'testuser\n' ;;
            *) return 99 ;;
        esac
    }
    sudo() {
        printf '%s\n' "$*" >> "$TEST_DIR/sudo"
        if [[ "$TEST_SUDO_STATUS" != 0 ]]; then return "$TEST_SUDO_STATUS"; fi
        if [[ "$1" == -n ]]; then shift; fi
        case "$1" in
            prlimit)
                printf '%s\n' "$*" > "$TEST_DIR/prlimit"
                return "$TEST_PRLIMIT_STATUS"
                ;;
            tee)
                cat > "$TEST_DIR/limits"
                return "$TEST_TEE_STATUS"
                ;;
            *) return 99 ;;
        esac
    }
    if [[ "$NON_INTERACTIVE" == 0 ]]; then
        prompt_confirm() { return 0; }
    fi

    printf '%s\n' "$$" > "$TEST_DIR/pid"
    configure_system
    printf 'reached verification\n'
    exit 0
fi

test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT

fail() {
    printf 'FAIL: %s\n' "$*" >&2
    cat "$test_dir/stdout" "$test_dir/stderr" >&2
    exit 1
}

assert_equal() {
    [[ "$1" == "$2" ]] || fail "expected '$2', got '$1'"
}

assert_contains() {
    grep -Fq -- "$2" "$test_dir/$1" || fail "$1 is missing '$2'"
}

assert_absent() {
    if grep -Fq -- "$2" "$test_dir/$1"; then fail "$1 unexpectedly contains '$2'"; fi
}

run_case() {
    local name="$1"; shift
    printf 'Testing: %s\n' "$name"
    local file
    for file in sudo prlimit limits stdout stderr; do : > "$test_dir/$file"; done
    if ! env TEST_DIR="$test_dir" TEST_NON_INTERACTIVE=1 TEST_DRY_RUN=0 \
        TEST_HARD_LIMIT=8192 TEST_PRLIMIT_STATUS=0 TEST_TEE_STATUS=0 TEST_SUDO_STATUS=0 \
        "$@" bash "$repo/scripts/test-install-memlock.sh" --case \
        </dev/null > "$test_dir/stdout" 2> "$test_dir/stderr"; then
        fail "$name exited before verification"
    fi
    assert_contains stdout 'reached verification'
}

assert_commands() {
    local sudo_flags="$1" pid
    pid=$(<"$test_dir/pid")
    assert_equal "$(<"$test_dir/sudo")" \
        "$(printf '%sprlimit --pid=%s --memlock=67108864:67108864\n%stee /etc/security/limits.d/99-dimos-memlock.conf' "$sudo_flags" "$pid" "$sudo_flags")"
    assert_equal "$(<"$test_dir/prlimit")" "prlimit --pid=$pid --memlock=67108864:67108864"
}

for limit in 65536 unlimited; do
    run_case "sufficient hard limit ($limit)" TEST_HARD_LIMIT="$limit"
    assert_equal "$(<"$test_dir/sudo")" ''
    assert_contains stdout 'locked-memory limit already configured'
done

run_case 'unattended update'
assert_commands '-n '
assert_equal "$(<"$test_dir/limits")" "$(printf "# Written by dimos: zenoh's SHM pool must be lockable.\ntestuser\t-\tmemlock\t65536")"
assert_contains stdout 'locked-memory limit raised for this session'
assert_contains stdout 'locked-memory limit saved for future logins'
assert_equal "$(<"$test_dir/stderr")" ''

run_case 'interactive update with piped stdin' TEST_NON_INTERACTIVE=0
assert_commands ''
assert_contains stdout 'locked-memory limit raised for this session'
assert_contains stdout 'locked-memory limit saved for future logins'

run_case 'live limit update fails' TEST_PRLIMIT_STATUS=1
assert_commands '-n '
assert_contains stderr 'could not raise the limit for this session'
assert_absent stdout 'locked-memory limit raised for this session'
assert_contains stdout 'locked-memory limit saved for future logins'

run_case 'persistence fails' TEST_TEE_STATUS=1
assert_commands '-n '
assert_contains stderr 'could not save locked-memory limit to /etc/security/limits.d/99-dimos-memlock.conf'
assert_contains stdout 'locked-memory limit raised for this session'
assert_absent stdout 'locked-memory limit saved for future logins'

run_case 'sudo authorization fails' TEST_SUDO_STATUS=1
assert_contains sudo '-n prlimit'
assert_contains sudo '-n tee'
assert_equal "$(<"$test_dir/prlimit")" ''
assert_equal "$(<"$test_dir/limits")" ''
assert_contains stderr 'could not raise the limit for this session'
assert_contains stderr 'could not save locked-memory limit to'
assert_absent stdout 'locked-memory limit raised for this session'
assert_absent stdout 'locked-memory limit saved for future logins'

run_case 'dry run' TEST_DRY_RUN=1
assert_equal "$(<"$test_dir/sudo")" ''
assert_contains stdout '[dry-run] sudo -n prlimit'
assert_contains stdout '[dry-run] would write /etc/security/limits.d/99-dimos-memlock.conf'
assert_absent stdout 'locked-memory limit raised for this session'
assert_absent stdout 'locked-memory limit saved for future logins'

printf 'All memlock installer tests passed.\n'
