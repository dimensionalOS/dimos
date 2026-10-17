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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import (
    build_dual_openyam_webxr,
    teleop_webxr_dual_openyam,
)


def test_visualization_configuration_preserves_teleop_composition(mocker):
    download = mocker.patch(
        "dimos.utils.data.get_data", side_effect=AssertionError("Config must not download models")
    )
    base = teleop_webxr_dual_openyam
    configured = build_dual_openyam_webxr(visualization=ViserVisualizationConfig(host="0.0.0.0"))

    assert configured.remapping_map == base.remapping_map
    assert [atom.module for atom in configured.blueprints] == [
        atom.module for atom in base.blueprints
    ]
    [original] = [atom for atom in base.blueprints if atom.module is ManipulationModule]
    [updated] = [atom for atom in configured.blueprints if atom.module is ManipulationModule]
    assert original.name == updated.name
    assert original.kwargs["visualization"].host == "127.0.0.1"
    assert updated.kwargs == {
        **original.kwargs,
        "visualization": ViserVisualizationConfig(host="0.0.0.0"),
    }
    assert [
        (atom.name, atom.kwargs)
        for atom in configured.blueprints
        if atom.module is not ManipulationModule
    ] == [
        (atom.name, atom.kwargs)
        for atom in base.blueprints
        if atom.module is not ManipulationModule
    ]

    parsed = BlueprintConfigParser(configured).parse(
        ["--manipulationmodule.visualization.host", "127.0.0.1"], environ={}
    )
    assert parsed.module_kwargs(updated.name)["visualization"]["host"] == "127.0.0.1"
    download.assert_not_called()
