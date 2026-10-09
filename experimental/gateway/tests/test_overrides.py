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

"""Launch overrides: the same cases as Desktop's Rust (src/dimos/overrides.rs tests), plus secrets."""

from typing import Any

import pytest

from experimental.gateway.utils.overrides import (
    HIDDEN,
    LaunchOverrides,
    is_secret_name,
    merge_modules,
    module_flags,
    parse,
    redact,
    secret_env,
    secret_paths,
    validate_global,
    validate_modules,
)

GLOBAL_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "robot_ip": {"anyOf": [{"type": "string"}, {"type": "null"}], "default": None},
        "n_workers": {"type": "integer", "default": 2},
        "nerf_speed": {"type": "number"},
        "replay": {"type": "boolean"},
        "viewer": {"enum": ["rerun", "none"], "type": "string"},
        "zenoh_connect_timeout": {"type": "number", "minimum": 0, "maximum": 86400},
        "transport": {"$ref": "#/$defs/Transport"},
        "topics": {"type": "array", "items": {"type": "string"}},
        "unitree_aes_128_key": {"anyOf": [{"type": "string"}, {"type": "null"}]},
    },
    "$defs": {"Transport": {"enum": ["lcm", "zenoh"], "type": "string"}},
}

BLUEPRINT_CONFIG: dict[str, Any] = {
    "name": "unitree-go2",
    "modules": [
        {
            "module": "go2connection",
            "class": "x.GO2Connection",
            "args": [
                {
                    "name": "lidar",
                    "type": "bool",
                    "schema": {"type": "boolean"},
                    "json_compatible": True,
                },
                {
                    "name": "mode",
                    "type": "x.Go2Mode",
                    "choices": ["default", "rage"],
                    "schema": {"enum": ["default", "rage"], "type": "string"},
                    "json_compatible": True,
                },
                {
                    "name": "motion_mode",
                    "type": "str | None",
                    "schema": {"anyOf": [{"type": "string"}, {"type": "null"}]},
                    "json_compatible": True,
                },
                {"name": "blueprint", "type": "Callable", "json_compatible": False},
                {
                    "name": "aes_128_key",
                    "type": "str",
                    "schema": {"type": "string"},
                    "json_compatible": True,
                },
            ],
        },
        {
            "module": "voxelgridmapper",
            "class": "x.VoxelGridMapper",
            "args": [
                {
                    "name": "voxel_size",
                    "type": "float",
                    "schema": {"type": "number"},
                    "json_compatible": True,
                },
                {
                    "name": "max_hz",
                    "type": "dict[str, float]",
                    "schema": {"type": "object", "additionalProperties": {"type": "number"}},
                    "json_compatible": True,
                },
            ],
        },
        {
            "module": "ns/cam_x",
            "class": "x.Camera",
            "args": [{"name": "fps", "type": "int", "default": 30}],
        },
    ],
}


def global_error(values: dict[str, Any]) -> str:
    with pytest.raises(ValueError) as caught:
        validate_global(values, GLOBAL_SCHEMA, "overrides.global")
    return str(caught.value)


def module_error(values: dict[str, Any]) -> str:
    with pytest.raises(ValueError) as caught:
        validate_modules(values, BLUEPRINT_CONFIG, "overrides.modules")
    return str(caught.value)


def parse_error(raw: Any) -> str:
    with pytest.raises(ValueError) as caught:
        parse(raw)
    return str(caught.value)


def test_parse_takes_both_shapes_and_refuses_a_mix() -> None:
    flat = parse({"robot_ip": "10.0.0.2"})
    assert flat == LaunchOverrides({"robot_ip": "10.0.0.2"}, {})
    structured = parse({"global": {"n_workers": 4}, "modules": {"go2connection": {"lidar": False}}})
    assert structured == LaunchOverrides({"n_workers": 4}, {"go2connection": {"lidar": False}})
    assert parse(None) == LaunchOverrides()
    assert parse_error({"global": {}, "robot_ip": "x"}) == (
        "overrides has `robot_ip` next to `global`/`modules`: put GlobalConfig keys under overrides.global"
    )
    assert parse_error({"modules": {"go2connection": 3}}) == (
        "overrides.modules.go2connection must be an object of fields, not number"
    )
    assert parse_error([1]) == "overrides must be an object, not list"
    assert parse_error({"global": "x"}) == "overrides.global must be an object, not string"
    assert parse({"secrets": ["robot_ip"], "global": {}}).secrets == ["robot_ip"]
    assert parse_error({"secrets": "robot_ip"}).startswith(
        "overrides.secrets must be a list of paths"
    )


def test_global_values_are_checked_against_the_schema() -> None:
    validate_global(
        {
            "robot_ip": "10.0.0.2",
            "n_workers": 4,
            "nerf_speed": 2,
            "replay": True,
            "viewer": "none",
            "transport": "lcm",
        },
        GLOBAL_SCHEMA,
        "overrides.global",
    )
    validate_global({"robot_ip": None, "n_workers": 2.0}, GLOBAL_SCHEMA, "overrides.global")
    assert global_error({"robot_iq": "x"}) == (
        "overrides.global.robot_iq: no such GlobalConfig field (did you mean robot_ip?)"
    )
    assert global_error({"n_workers": "four"}) == (
        'overrides.global.n_workers: needs a whole number, got string "four"'
    )
    assert (
        global_error({"n_workers": "8"})
        == 'overrides.global.n_workers: needs a whole number, got string "8"'
    )
    assert (
        global_error({"n_workers": 2.5})
        == "overrides.global.n_workers: needs a whole number, got number 2.5"
    )
    assert 'needs one of "rerun", "none"' in global_error({"viewer": "foxglove"})
    assert '"lcm", "zenoh"' in global_error({"transport": "ros"})
    assert global_error({"zenoh_connect_timeout": -1}) == (
        "overrides.global.zenoh_connect_timeout: needs at least 0, got -1"
    )
    assert "needs a string" in global_error({"robot_ip": 5})
    assert "can't be passed as a `dimos` flag" in global_error({"topics": ["a"]})
    assert "needs true or false" in global_error({"replay": "yes"})
    assert "needs true or false" in global_error({"replay": 1})
    # a secret's value never shows in the message
    assert global_error({"unitree_aes_128_key": 7}) == (
        "overrides.global.unitree_aes_128_key: needs a string (value hidden: it's secret)"
    )


def test_module_values_are_checked_against_each_field() -> None:
    validate_modules(
        {
            "go2connection": {"lidar": False, "mode": "rage", "motion_mode": None},
            "voxelgridmapper": {"voxel_size": 0.1, "max_hz": {"lidar": 2}},
            "ns/cam_x": {"fps": "anything"},
        },
        BLUEPRINT_CONFIG,
        "overrides.modules",
    )
    assert module_error({"go2conection": {"lidar": False}}) == (
        "overrides.modules.go2conection: no such module in unitree-go2 (did you mean go2connection?)"
    )
    assert module_error({"go2connection": {"lidr": False}}) == (
        "overrides.modules.go2connection.lidr: no such field of go2connection (did you mean lidar?)"
    )
    assert module_error({"go2connection": {"lidar": "no"}}) == (
        'overrides.modules.go2connection.lidar: needs true or false, got string "no"'
    )
    assert 'needs one of "default", "rage"' in module_error({"go2connection": {"mode": "fast"}})
    assert "can't be written as JSON" in module_error({"go2connection": {"blueprint": "x"}})
    assert "`lidar`: needs a number" in module_error(
        {"voxelgridmapper": {"max_hz": {"lidar": "fast"}}}
    )
    assert module_error({"go2connection": {"aes_128_key": 1}}) == (
        "overrides.modules.go2connection.aes_128_key: needs a string (value hidden: it's secret)"
    )


def test_module_flags_follow_dimos_run() -> None:
    assert module_flags(
        {
            "go2connection": {"lidar": False, "motion_mode": None, "odom_frame_id": "--odd"},
            "voxel_grid": {"voxel_size": 0.1, "max_hz": {"lidar": 2}},
            "ns/cam_x": {"frame_rate": 15},
        }
    ) == [
        "--go2connection.lidar=false",
        "--go2connection.odom-frame-id=--odd",
        "--ns-cam-x.frame-rate=15",
        '--voxel-grid.max-hz={"lidar":2}',
        "--voxel-grid.voxel-size=0.1",
    ]


def test_merge_drops_nulls() -> None:
    saved = {"a": {"x": 1, "y": 2}, "b": {"z": 3}}
    once = {"a": {"x": 5, "y": None}, "b": {"z": None}, "c": {"w": 1}}
    assert merge_modules(saved, once) == {"a": {"x": 5}, "c": {"w": 1}}


def test_secrets_by_name_or_listed_go_in_the_environment() -> None:
    assert [
        is_secret_name(n)
        for n in ("key", "aes_128_key", "API_TOKEN", "passwd", "keyboard", "robot_ip")
    ] == [
        True,
        True,
        True,
        True,
        False,
        False,
    ]
    global_ = {"unitree_aes_128_key": "k1", "robot_ip": "10.0.0.2", "dimos_api_key": None}
    modules = {"go2connection": {"aes_128_key": "k2", "lidar": False}, "ns/cam": {"ip": "1.2.3.4"}}
    paths = secret_paths(global_, modules, ["ns/cam.ip"])
    assert paths == [
        "dimos_api_key",
        "unitree_aes_128_key",
        "go2connection.aes_128_key",
        "ns/cam.ip",
    ]
    assert secret_env(global_, modules, paths) == {
        "UNITREE_AES_128_KEY": "k1",
        "GO2CONNECTION__AES_128_KEY": "k2",
        "NS_CAM__IP": "1.2.3.4",
    }
    assert redact(global_, modules, paths) == (
        {"unitree_aes_128_key": HIDDEN, "robot_ip": "10.0.0.2", "dimos_api_key": None},
        {"go2connection": {"aes_128_key": HIDDEN, "lidar": False}, "ns/cam": {"ip": HIDDEN}},
    )
