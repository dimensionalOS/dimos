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

import pytest

import dimos.benchmark.evaluation.registry as registry


def test_unknown_evaluation_reports_available_builtins() -> None:
    with pytest.raises(
        registry.EvaluationRegistryError,
        match="Available evaluations: libero-pro, vlnce-r2r",
    ):
        registry.resolve_evaluation("missing")


def test_builtin_libero_pro_evaluation_resolves_in_repo() -> None:
    resolved = registry.resolve_evaluation("libero-pro")

    assert resolved.provider == "dimos"
    assert resolved.evaluation.name == "libero-pro"


def test_builtin_vlnce_r2r_evaluation_resolves_with_live_agent() -> None:
    resolved = registry.resolve_evaluation("vlnce-r2r")

    assert resolved.provider == "dimos"
    assert resolved.evaluation.runtime_profile == "live-agent-v1"
