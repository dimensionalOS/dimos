#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

pointlio_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cmake_args=(
  -S "$pointlio_dir"
  -B "$pointlio_dir/build"
  -DCMAKE_BUILD_TYPE=Release
)
if [[ -n "${DIMOS_LCM_DIR:-}" ]]; then
  cmake_args+=("-DDIMOS_LCM_DIR=${DIMOS_LCM_DIR}")
fi
if [[ -n "${M20_POINTLIO_DIR:-}" ]]; then
  cmake_args+=("-DPOINTLIO_DIR=${M20_POINTLIO_DIR}")
fi
if [[ -n "${M20_PFR_DIR:-}" ]]; then
  cmake_args+=("-DFETCHCONTENT_SOURCE_DIR_PFR=${M20_PFR_DIR}")
fi
if [[ -n "${M20_NLOHMANN_JSON_DIR:-}" ]]; then
  cmake_args+=("-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=${M20_NLOHMANN_JSON_DIR}")
fi

cmake "${cmake_args[@]}"
cmake --build "$pointlio_dir/build" --parallel "${M20_BUILD_JOBS:-4}"
