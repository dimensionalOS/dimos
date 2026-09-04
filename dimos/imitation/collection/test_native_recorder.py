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

from collections.abc import Iterator
from pathlib import Path

import pytest
import pytest_mock

from dimos.robot.manipulators.dual_openyam.learning import DualOpenYamQuestRecorder


@pytest.fixture
def recorder(tmp_path: Path) -> Iterator[DualOpenYamQuestRecorder]:
    instance = DualOpenYamQuestRecorder(
        store={"kind": "sqlite", "path": str(tmp_path / "collection.db")},
        record_tf=False,
    )
    yield instance
    instance.stop()


def test_profile_recorder_declares_and_resolves_all_typed_streams(
    recorder: DualOpenYamQuestRecorder,
    mocker: pytest_mock.MockerFixture,
) -> None:
    names = (
        "left_wrist_image",
        "right_wrist_image",
        "coordinator_joint_state",
        "applied_joint_position_command",
        "status",
    )
    for name in names:
        getattr(recorder, name).transport = mocker.MagicMock(channel=f"dimos/{name}")

    specs = recorder._stream_specs()

    assert {spec.name: spec.codec for spec in specs} == {
        "left_wrist_image": "jpeg",
        "right_wrist_image": "jpeg",
        "coordinator_joint_state": "lcm",
        "applied_joint_position_command": "lcm",
        "status": "lcm",
    }
