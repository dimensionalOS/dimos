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

from pathlib import Path

from pydantic import ValidationError
import pytest

from dimos.imitation.dataprep.core import OutputConfig, SyncConfig
from dimos.imitation.profile import (
    ImageSource,
    JointPositionAction,
    JointPositionSource,
    PolicyIOProfile,
)


def _profile() -> PolicyIOProfile:
    joints = ("left_joint", "right_joint")
    return PolicyIOProfile(
        name="dual-test",
        robot_type="dual_test",
        observations={
            "left": ImageSource(stream="left_image", shape=(480, 640, 3)),
            "state": JointPositionSource(stream="joint_state", joints=joints),
        },
        action=JointPositionAction(
            key="actions",
            demonstration=JointPositionSource(stream="joint_command", joints=joints),
        ),
        sync=SyncConfig(anchor="left", rate_hz=30.0, tolerance_ms=20.0),
    )


def test_profile_builds_matching_dataprep_config(tmp_path: Path) -> None:
    profile = _profile()
    output = OutputConfig(path=tmp_path / "dataset")

    config = profile.dataprep_config(source="recording.mcap", output=output)

    assert config.source == "recording.mcap"
    assert config.output == output
    assert config.observation["left"].stream == "left_image"
    assert config.observation["left"].shape == (480, 640, 3)
    assert config.observation["state"].names == ["left_joint", "right_joint"]
    assert config.action["actions"].stream == "joint_command"
    assert config.sync.anchor == "left"


@pytest.mark.parametrize(
    ("update", "message"),
    [
        ({"sync": SyncConfig(anchor="missing", rate_hz=30.0, tolerance_ms=20.0)}, "anchor"),
        (
            {
                "action": JointPositionAction(
                    key="left",
                    demonstration=JointPositionSource(stream="joint_command", joints=("joint",)),
                )
            },
            "action key",
        ),
    ],
)
def test_profile_rejects_ambiguous_feature_contracts(
    update: dict[str, object], message: str
) -> None:
    values = _profile().model_dump()
    values.update(update)

    with pytest.raises(ValidationError, match=message):
        PolicyIOProfile.model_validate(values)


def test_profile_rejects_conflicting_types_on_one_stream() -> None:
    values = _profile().model_dump()
    values["action"] = {
        "key": "actions",
        "demonstration": {"stream": "left_image", "joints": ["joint"]},
    }

    with pytest.raises(ValidationError, match="conflicting source types"):
        PolicyIOProfile.model_validate(values)


@pytest.mark.parametrize(
    "source",
    [
        {"stream": "left-image", "shape": (480, 640, 3)},
        {"stream": "left_image", "shape": (480, 640, 1)},
        {"stream": "left_image", "shape": (0, 640, 3)},
    ],
)
def test_image_source_requires_an_rgb_hwc_python_port(source: dict[str, object]) -> None:
    with pytest.raises(ValidationError):
        ImageSource.model_validate(source)
