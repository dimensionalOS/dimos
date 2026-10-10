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

from experimental.gateway import launches, overrides as ov
from experimental.gateway.launches import LaunchConfig


def test_parse_takes_both_forms_and_refuses_a_mix() -> None:
    assert ov.parse({"robot_ip": "x"}).global_ == {"robot_ip": "x"}
    structured = ov.parse({"global": {"replay": True}, "modules": {"cam": {"fps": 5}}})
    assert (structured.global_, structured.modules) == ({"replay": True}, {"cam": {"fps": 5}})
    with pytest.raises(ValueError):
        ov.parse({"global": {}, "robot_ip": "x"})
    with pytest.raises(ValueError):
        ov.parse([1])


def test_null_drops_a_saved_value() -> None:
    assert ov.merge({"a": 1, "b": 2}, {"b": None, "c": 3}) == {"a": 1, "c": 3}
    assert ov.merge_modules({"cam": {"fps": 5}}, {"cam": {"fps": None}}) == {}


def test_secrets_go_in_the_environment_not_the_command_line() -> None:
    launch = LaunchConfig(
        {"typesafe_api_key": "k", "robot_ip": "1.2.3.4"}, {"cam/x": {"token": "t", "fps": 5}}
    )
    args = launches.run_args("demo", launch)
    assert args == ["--robot-ip=1.2.3.4", "run", "demo", "--cam-x.fps=5"]
    env = ov.secret_env(launch.global_, launch.modules, launch.secrets())
    assert env == {"TYPESAFE_API_KEY": "k", "CAM_X__TOKEN": "t"}


def test_flags_spell_values_as_dimos_reads_them() -> None:
    assert ov.global_flags({"replay": True, "dtop": False, "n_workers": 2, "robot_ips": ["a"]}) == [
        "--no-dtop",
        "--n-workers=2",
        "--replay",
        '--robot-ips=["a"]',
    ]
    launch = LaunchConfig({"replay": True}, {}, args=["--api-key", "x"])
    assert launches.run_args("demo", launch) == ["run", "demo", "--replay=true", "--api-key", "x"]
    assert ov.shown_args(["--api-key", "x", "--token=y", "--fps=3"]) == [
        "--api-key",
        ov.HIDDEN,
        f"--token={ov.HIDDEN}",
        "--fps=3",
    ]
