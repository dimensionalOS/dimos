#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

pointlio_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${M20_ROS_SETUP:-/opt/robot/scripts/setup_ros2.sh}"

if [[ -f "$ros_setup" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$ros_setup"
  set -u
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/foxy/setup.bash
  set -u
else
  echo "M20 ROS setup not found: $ros_setup" >&2
  exit 1
fi

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
