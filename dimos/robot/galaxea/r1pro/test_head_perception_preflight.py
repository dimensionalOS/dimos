# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.galaxea.r1pro.blueprints.perception.r1pro_head_perception import (
    r1pro_head_perception,
)
from dimos.robot.galaxea.r1pro.head_perception_preflight import head_perception_dependencies


def test_missing_dependencies_and_cpu_torch_fail_before_hardware(mocker):
    mocker.patch(
        "dimos.robot.galaxea.r1pro.head_perception_preflight.importlib.util.find_spec",
        side_effect=lambda name: None if name in {"sam2", "open3d"} else mocker.sentinel.spec,
    )
    torch = mocker.MagicMock(__version__="2.6.0+cpu")
    torch.version.cuda = None
    torch.cuda.is_available.return_value = False
    mocker.patch(
        "dimos.robot.galaxea.r1pro.head_perception_preflight.importlib.import_module",
        return_value=torch,
    )
    error = head_perception_dependencies()
    assert "sam2" in error and "open3d" in error
    assert "2.6.0+cpu" in error and "CUDA PyTorch required" in error


def test_supported_environment_passes_without_loading_models(mocker):
    mocker.patch(
        "dimos.robot.galaxea.r1pro.head_perception_preflight.importlib.util.find_spec",
        return_value=mocker.sentinel.spec,
    )
    torch = mocker.MagicMock()
    torch.cuda.is_available.return_value = True
    imported = mocker.patch(
        "dimos.robot.galaxea.r1pro.head_perception_preflight.importlib.import_module",
        return_value=torch,
    )
    assert head_perception_dependencies() is None
    imported.assert_called_once_with("torch")


def test_blueprint_requirement_fails_before_any_module_start(mocker):
    mocker.patch(
        "dimos.robot.galaxea.r1pro.head_perception_preflight.head_perception_dependencies",
        return_value="CUDA PyTorch missing",
    )
    mocker.patch(
        "dimos.robot.galaxea.r1pro.head_perception_preflight.head_perception_assets",
        return_value=[],
    )
    mocker.patch(
        "dimos.core.coordination.module_coordinator._resolve_blueprint_config",
        return_value=({}, {}, {}),
    )
    mocker.patch(
        "dimos.core.coordination.module_coordinator._materialize_transports", return_value={}
    )
    mocker.patch("dimos.core.coordination.module_coordinator._run_configurators")
    start = mocker.patch.object(ModuleCoordinator, "start")
    deploy = mocker.patch("dimos.core.coordination.module_coordinator._deploy_all_modules")
    with pytest.raises(SystemExit):
        ModuleCoordinator.build(r1pro_head_perception)
    start.assert_not_called()
    deploy.assert_not_called()
