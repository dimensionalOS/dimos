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

from unittest.mock import call, sentinel

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import _run_configurators
from dimos.core.global_config import global_config
from dimos.protocol.service.system_configurator import base, lcm_config


@pytest.mark.parametrize("skip, expected", [(False, [call([sentinel.check])]), (True, [])])
def test_offline_opt_out_controls_system_configuration(monkeypatch, mocker, skip, expected):
    monkeypatch.setattr(global_config, "skip_system_configuration", skip)
    monkeypatch.setattr(global_config, "transport", "lcm")
    mocker.patch.object(lcm_config, "lcm_configurators", return_value=[])
    configure = mocker.patch.object(base, "configure_system")
    blueprint = mocker.Mock(spec=Blueprint, configurator_checks=[sentinel.check])

    _run_configurators(blueprint)

    assert configure.call_args_list == expected
