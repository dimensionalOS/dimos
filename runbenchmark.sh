#!/usr/bin/env bash
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly REPLAY_DATABASES=(
    go2_bigoffice
    go2_short
    go2_mid360_stairs
    go2_china_office
    go2_hongkong_office
    go2_slamabuse1
    go2_slamabuse2
)

if [[ $# -eq 1 && "$1" == "--list" ]]; then
    echo "Run these comprehensive benchmarks one at a time:"
    for database in "${REPLAY_DATABASES[@]}"; do
        printf './runbenchmark.sh %s\n' "${database}"
    done
    exit 0
fi

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <replay-db>" >&2
    echo "       $0 --list" >&2
    echo "Example: $0 go2_mid360_stairs" >&2
    exit 2
fi

readonly PYTHON="${SCRIPT_DIR}/.venv/bin/python"
readonly BENCHMARK="${SCRIPT_DIR}/demo_benchmark_python.py"
readonly REPLAY_DB="$1"

db_name="$(basename "${REPLAY_DB}")"
readonly DB_LABEL="${db_name%.db}"
readonly OUTPUT_DIR="${SCRIPT_DIR}/benchmark_results/${DB_LABEL}"
readonly RUN_PLAN=(
    python-1
    rust-1
)

if [[ ! -x "${PYTHON}" ]]; then
    echo "Missing virtual environment Python: ${PYTHON}" >&2
    echo "Run 'uv sync --extra all' first." >&2
    exit 1
fi

mkdir -p "${OUTPUT_DIR}"

for run in "${RUN_PLAN[@]}"; do
    implementation="${run%-*}"
    echo
    echo "Running ${REPLAY_DB}: ${run}"
    "${PYTHON}" "${BENCHMARK}" \
        --implementation "${implementation}" \
        --blueprint unitree-go2 \
        --replay-db "${REPLAY_DB}" \
        --transport lcm \
        --duration 60 \
        --out "${OUTPUT_DIR}/${run}"
done

result_files=()
for run in "${RUN_PLAN[@]}"; do
    result_files+=("${OUTPUT_DIR}/${run}/result.json")
done

echo
echo "Building comparison"
"${PYTHON}" "${BENCHMARK}" \
    --compare "${result_files[@]}" \
    --out "${OUTPUT_DIR}/comparison"

echo
echo "Benchmark complete: ${OUTPUT_DIR}/comparison"
