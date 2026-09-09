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

"""Backend selection through the public module and blueprint config parser."""

from collections.abc import Iterator
import json
import subprocess
import sys

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
import dimos.manipulation.grasping.grasp_gen_x as grasp_gen_x
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXConfig, GraspGenXError
from dimos.manipulation.grasping.grasp_proposal import GraspProposalModule
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.manipulators.xarm.blueprints.perception import xarm_perception
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    xarm_perception_sim,
    xarm_room_sim,
)
from dimos.spec.utils import spec_annotation_compliance


@pytest.fixture
def module() -> Iterator[GraspProposalModule]:
    instance = GraspProposalModule()
    try:
        yield instance
    finally:
        instance.stop()


@pytest.fixture
def graspgenx_config():
    return {
        "gripper": {
            "extents_open": [0.1, 0.1, 0.1],
            "offset_open": [0.0, 0.0, 0.0],
            "extents_half_open": [0.08, 0.1, 0.1],
            "offset_half_open": [0.0, 0.0, 0.0],
            "fingertip_depth": 0.02,
        },
    }


def cloud():
    return PointCloud2.from_numpy(
        np.array([[0, 0, 0], [0.1, 0, 0.1], [0, 0.1, 0.1]], dtype=np.float32),
        frame_id="world",
        timestamp=1.0,
    )


def test_default_proposals_and_lifecycle(module):
    assert spec_annotation_compliance(module, GraspGenSpec)
    with pytest.raises(RuntimeError, match="not been started"):
        module.propose_grasps(cloud())
    module.start()
    module.start()
    result = module.propose_grasps(cloud())
    assert len(result) == 1
    assert result.candidates[0].score == 1.0
    assert result.header.frame_id == "world"
    module.stop()
    with pytest.raises(RuntimeError, match="not been started"):
        module.propose_grasps(cloud())
    module.start()
    assert len(module.propose_grasps(cloud())) == 1


def test_learned_backend_lifecycle_and_errors(graspgenx_config, mocker):
    runtime = mocker.patch.object(grasp_gen_x, "_create_runtime")
    runtime.return_value.infer.return_value = (np.eye(4)[None], np.array([0.75]))
    instance = GraspProposalModule(backend="graspgenx", graspgenx=graspgenx_config)
    try:
        instance.start()
        instance.start()
        runtime.assert_called_once_with(instance.config.graspgenx)
        result = instance.propose_grasps(cloud())
        assert result.candidates[0].score == 0.75
        assert result.header.timestamp == 1.0
        runtime.return_value.infer.side_effect = RuntimeError("inference broke")
        with pytest.raises(GraspGenXError, match="inference"):
            instance.propose_grasps(cloud())
        instance.stop()
        instance.start()
        assert runtime.call_count == 2
    finally:
        instance.stop()


def test_failed_model_start_does_not_fall_back(graspgenx_config, mocker):
    runtime = mocker.patch.object(grasp_gen_x, "_create_runtime")
    runtime.side_effect = RuntimeError("CUDA unavailable")
    instance = GraspProposalModule(backend="graspgenx", graspgenx=graspgenx_config)
    try:
        with pytest.raises(GraspGenXError, match="initialize"):
            instance.start()
        with pytest.raises(RuntimeError, match="not been started"):
            instance.propose_grasps(cloud())
    finally:
        instance.stop()


def test_json_configuration_and_cli_override(graspgenx_config, tmp_path):
    config_path = tmp_path / "grasp.json"
    config_path.write_text(
        json.dumps(
            {"graspproposalmodule": {"graspgenx": {**graspgenx_config, "max_candidates": 20}}}
        )
    )
    parsed = BlueprintConfigParser(GraspProposalModule.blueprint()).parse(
        [
            "--graspproposalmodule.backend",
            "graspgenx",
            "--graspproposalmodule.graspgenx.max-candidates",
            "5",
        ],
        config_path=config_path,
        environ={},
    )
    instance = GraspProposalModule(**parsed.module_kwargs("graspproposalmodule"))
    try:
        assert isinstance(instance.config.graspgenx, GraspGenXConfig)
        assert instance.config.graspgenx.max_candidates == 5
        assert instance.config.graspgenx.gripper.fingertip_depth == 0.02
    finally:
        instance.stop()


@pytest.mark.parametrize("backend", ["invalid", "graspgenx"])
def test_invalid_or_incomplete_backend_config_fails_before_start(backend):
    with pytest.raises(BlueprintConfigError):
        BlueprintConfigParser(GraspProposalModule.blueprint()).parse(
            ["--graspproposalmodule.backend", backend], environ={}
        )


@pytest.mark.parametrize("backend", ["heuristic", "graspgenx"])
@pytest.mark.parametrize("blueprint", [xarm_perception, xarm_perception_sim, xarm_room_sim])
def test_xarm_blueprints_select_one_configurable_provider(blueprint, backend):
    providers = [
        atom.module for atom in blueprint.active_blueprints if issubclass(atom.module, GraspGenSpec)
    ]
    assert providers == [GraspProposalModule]
    parsed = BlueprintConfigParser(blueprint).parse(
        ["--graspproposalmodule.backend", backend], environ={}
    )
    kwargs = parsed.module_kwargs("graspproposalmodule")
    assert kwargs["backend"] == backend
    assert kwargs["graspgenx"]["gripper"]["extents_open"] == (0.088924, 0.03, 0.037)
    assert kwargs["graspgenx"]["grasp_frame_to_tcp"][2][3] == 0.172


def test_heuristic_start_and_cli_help_do_not_load_optional_runtime():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import sys
from dimos.manipulation.grasping.grasp_proposal import GraspProposalModule
from dimos.robot.manipulators.xarm.config import XARM_GRASPGENX_CONFIG
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
module = GraspProposalModule(graspgenx=XARM_GRASPGENX_CONFIG)
try:
    BlueprintConfigParser(module.blueprint()).format_help()
    module.start()
    assert "dimos.manipulation.grasping.grasp_gen_x_runtime" not in sys.modules
    assert not any(name == "graspgenx" or name.startswith("graspgenx.") for name in sys.modules)
finally:
    module.stop()
""",
        ],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
