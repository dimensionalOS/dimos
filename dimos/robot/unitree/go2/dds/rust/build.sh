#!/usr/bin/env bash
# Build go2_dds. cyclonedds-sys runs bindgen (libclang) against an installed CycloneDDS;
# the defaults below find both in the nix dev shell or on a stock Ubuntu with
# libclang-dev and cyclonedds-dev installed.
set -euo pipefail
cd "$(dirname "$0")"
if [[ -z "${CYCLONEDDS_LIB_DIR:-}" && -n "${CYCLONEDDS_HOME:-}" ]]; then
    export CYCLONEDDS_LIB_DIR="$CYCLONEDDS_HOME/lib" CYCLONEDDS_INCLUDE_DIR="$CYCLONEDDS_HOME/include"
fi
if [[ -z "${LIBCLANG_PATH:-}" ]]; then
    for d in /usr/lib/llvm-*/lib /usr/lib /usr/lib64 /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu; do
        if ls "$d"/libclang.so* >/dev/null 2>&1; then
            export LIBCLANG_PATH="$d"
            break
        fi
    done
fi
if [[ -z "${BINDGEN_EXTRA_CLANG_ARGS:-}" ]]; then
    # libclang's own builtin headers, then the C compiler's include dirs (the nix glibc
    # lives nowhere libclang would look on its own).
    args=""
    for d in "${LIBCLANG_PATH:-/nonexistent}"/clang/*/include; do
        [[ -d "$d" ]] && args="-isystem $d"
    done
    args="$args $(echo | cc -E -Wp,-v - 2>&1 | sed -n 's/^ \(\/.*\)$/-isystem \1/p' | tr '\n' ' ')"
    export BINDGEN_EXTRA_CLANG_ARGS="$args"
fi
# No arguments builds the release binary; `./build.sh test` or `./build.sh clippy` reuse the env.
[[ $# -eq 0 ]] && set -- build --release
exec cargo "$@"
