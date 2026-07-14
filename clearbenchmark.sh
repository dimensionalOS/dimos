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

if [[ $# -ne 0 ]]; then
    echo "Usage: $0" >&2
    exit 2
fi

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly RESULTS_DIR="${SCRIPT_DIR}/benchmark_results"

if [[ ! -d "${RESULTS_DIR}" ]]; then
    echo "No benchmark results to clear."
    exit 0
fi

printf 'Delete all benchmark results in %s? [y/N] ' "${RESULTS_DIR}"
read -r response
if [[ ! "${response}" =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 0
fi

rm -rf -- "${RESULTS_DIR}"
mkdir -p "${RESULTS_DIR}"
echo "Benchmark results cleared."
