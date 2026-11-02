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

"""Correctness gates for the experiment, using real pinned backend/message code."""

import numpy as np
import pytest

from dimos.sim2.experiments.demo_state_delivery import delivery_run
from dimos.sim2.experiments.demo_state_scaling import (
    STRATEGIES,
    Pipeline,
    WorldPair,
    failure_cases,
    max_error,
    physical_run,
    reference,
)


@pytest.fixture
def pair(tmp_path):
    return WorldPair.build(tmp_path, 4, joints=7)


@pytest.mark.parametrize("kind", STRATEGIES)
@pytest.mark.parametrize("fraction", [0, 0.25, 1])
def test_every_pipeline_reconstructs_the_same_sample(pair, kind, fraction):
    pipe = Pipeline(pair, kind)
    for frame in range(30):
        pair.pose_sequence(frame, fraction)
        result = pipe.consume(pipe.encode(pipe.capture()))
        assert max_error(result, reference(pair)) <= 1.000001e-6


def test_capture_choice_does_not_change_physics_trajectory(pair):
    runs = [physical_run(pair, kind, 6, 0) for kind in (*STRATEGIES, "physics_only")]
    np.testing.assert_allclose(
        [run["trajectory_checksum"] for run in runs], runs[0]["trajectory_checksum"], rtol=0, atol=0
    )
    assert all(run["peak_contacts"] > 0 for run in runs)


@pytest.mark.parametrize("kind", ["pim_full", "compact_full"])
def test_complete_sample_recovers_current_state_after_loss_and_late_join(pair, kind):
    results = [r for r in failure_cases(pair) if r["kind"] == kind]
    assert len(results) == 2
    assert all(r["pass"] for r in results)


@pytest.mark.parametrize("case", ["late_join", "missed_final_update", "older_frame_rejected"])
@pytest.mark.xfail(
    strict=True,
    raises=AssertionError,
    reason="Pinned Jeff mirror has no baseline/gap/order recovery; a passing result requires review",
)
def test_delta_receiver_recovers_a_valid_current_state(pair, case):
    result = next(r for r in failure_cases(pair) if r["kind"] == "jeff_delta" and r["case"] == case)
    assert result["pass"], result


@pytest.mark.parametrize("kind", STRATEGIES)
def test_real_process_receiver_gets_coherent_decodable_state(pair, kind):
    result = delivery_run(pair, kind, 50, 1, 0.12, 0)
    consumer = result["consumers"][0]
    assert consumer["missing"] == 0
    assert consumer["errors"] == []
    assert consumer["max_probe_error"] <= 1e-6
    assert consumer["received"] == 6
