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

from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.experimental.robot.bosdyn.spot.effectors.high_level import SpotHighLevel
from dimos.robot.get_all_blueprints import get_by_name


@pytest.mark.parametrize(
    ("blueprint", "option", "field"),
    [
        ("spot", "--spothighlevel.password=example", "username"),
        ("spot-record", "--spothighlevel.username=operator", "password"),
    ],
)
def test_live_blueprints_require_both_credentials(blueprint, option, field) -> None:
    parser = BlueprintConfigParser(get_by_name(blueprint).solidify())
    with pytest.raises(BlueprintConfigError, match=rf"spothighlevel\.{field}: Field required"):
        parser.parse([option], environ={})


@pytest.mark.parametrize(
    ("username", "password", "field"),
    [(" ", "example", "username"), ("operator", "\t", "password")],
)
def test_python_build_rejects_blank_credentials_before_host_setup(
    username, password, field, mocker
) -> None:
    setup = mocker.patch(
        "dimos.core.coordination.module_coordinator._run_configurators",
        side_effect=AssertionError("reached host setup"),
    )
    with pytest.raises(BlueprintConfigError, match=rf"spothighlevel\.{field}: .*blank"):
        ModuleCoordinator.build(SpotHighLevel.blueprint(username=username, password=password))
    setup.assert_not_called()


def test_run_config_validates_environment_password_without_stripping_it() -> None:
    parser = BlueprintConfigParser(get_by_name("spot").solidify())
    options = ["--spothighlevel.username=operator"]
    with pytest.raises(BlueprintConfigError, match=r"spothighlevel\.password: .*blank"):
        parser.parse(options, environ={"SPOTHIGHLEVEL__PASSWORD": " "})

    parsed = parser.parse(options, environ={"SPOTHIGHLEVEL__PASSWORD": " example "})
    assert parsed.module_kwargs("spothighlevel")["password"] == " example "
