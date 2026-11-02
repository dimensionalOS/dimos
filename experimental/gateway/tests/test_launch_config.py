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

from experimental.gateway import launches
from experimental.gateway.launches import LaunchConfig


def test_parse_takes_both_forms_and_refuses_a_mix() -> None:
    assert launches.parse({"robot_ip": "x"}).global_ == {"robot_ip": "x"}
    structured = launches.parse({"global": {"replay": True}, "modules": {"cam": {"fps": 5}}})
    assert (structured.global_, structured.modules) == ({"replay": True}, {"cam": {"fps": 5}})
    with pytest.raises(ValueError):
        launches.parse({"global": {}, "robot_ip": "x"})
    with pytest.raises(ValueError):
        launches.parse([1])


def test_secrets_go_in_the_environment_not_the_command_line() -> None:
    launch = LaunchConfig(
        {"typesafe_api_key": "k", "robot_ip": "1.2.3.4"}, {"cam/x": {"token": "t", "fps": 5}}
    )
    args = launches.run_args("demo", launch)
    assert args == ["--robot-ip=1.2.3.4", "run", "demo", "--cam-x.fps=5"]
    env = launches.secret_env(launch.global_, launch.modules, launch.secret_paths())
    assert env == {"TYPESAFE_API_KEY": "k", "CAM_X__TOKEN": "t"}


def test_flags_spell_values_as_dimos_reads_them() -> None:
    assert launches.global_flags(
        {"replay": True, "dtop": False, "n_workers": 2, "robot_ips": ["a"]}
    ) == [
        "--no-dtop",
        "--n-workers=2",
        "--replay",
        '--robot-ips=["a"]',
    ]
    launch = LaunchConfig({"replay": True}, {}, args=["--api-key", "x"])
    assert launches.run_args("demo", launch) == [
        *("run", "demo"),
        *("--replay=true", "--api-key", "x"),
    ]
    assert launches.shown_args(["--api-key", "x", "--token=y", "--fps=3"]) == [
        "--api-key",
        launches.HIDDEN,
        f"--token={launches.HIDDEN}",
        "--fps=3",
    ]


def test_dotted_module_fields_become_one_object_flag() -> None:
    flags = launches.module_flags(
        {"controlcoordinator": {"addresses.xarm_arm": "192.0.2.1", "addresses.piper_arm": "can0"}}
    )
    assert flags == ['--controlcoordinator.addresses={"piper_arm":"can0","xarm_arm":"192.0.2.1"}']


def test_launch_env_defaults_are_settings_dimos_reads() -> None:
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    parsed = BlueprintConfigParser(RerunBridgeModule.blueprint()).parse(
        environ=launches.launch_env_defaults()
    )
    kwargs = parsed.module_kwargs(RerunBridgeModule.name)
    assert (kwargs["rerun_open"], kwargs["rerun_web"]) == ("none", True)
