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

import asyncio
from io import StringIO
import json
from pathlib import Path
import sys
from typing import Any, Literal

from pydantic import BaseModel, Field
import pytest
from pytest_mock import MockerFixture

from dimos.agents.annotation import skill
from dimos.core import global_config as global_config_module
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from dimos.robot import get_all_blueprints
from experimental.gateway import introspect
from experimental.gateway.introspect import (
    Cache,
    IntrospectError,
    blueprint_modules,
    field_args,
    is_secret_name,
    module_args,
    rpcs,
    run_child,
    spec_topic,
    targets,
)


def test_spec_topic_reads_lcm_and_zenoh_specs() -> None:
    assert spec_topic(LCMTransport.spec("/cmd_vel", Twist)) == "/cmd_vel"
    assert (
        spec_topic(ZenohTransport.spec(ZenohTopic("dimos/head_left_color", Twist)))
        == "/head_left_color"
    )


class LeftCamera(Module):
    jpeg_out: Out[Twist]


class RightCamera(Module):
    jpeg_out: Out[Twist]


def test_remapped_streams_show_their_wired_topic() -> None:
    blueprint = (
        autoconnect(LeftCamera.blueprint(), RightCamera.blueprint())
        .solidify()
        .remappings(
            [
                (LeftCamera, "jpeg_out", "head_left_color"),
                (RightCamera, "jpeg_out", "head_right_color"),
            ]
        )
        .transports(
            {
                ("head_left_color", Twist): ZenohTransport.spec(
                    ZenohTopic("dimos/head_left_color", Twist)
                )
            }
        )
    )
    streams = {
        (module["name"], stream["name"]): stream
        for module in blueprint_modules(blueprint, {})
        for stream in module["streams"]
    }
    left = streams["leftcamera", "jpeg_out"]
    assert (left["wire"], left["topic"]) == ("head_left_color", "/head_left_color")
    assert streams["rightcamera", "jpeg_out"]["topic"] == "/head_right_color"


class Dog(Module):
    @rpc
    def bark(self, times: int = 2) -> str:
        return "woof" * times

    @skill
    def trick(self, name: Literal["sit", "jump"]) -> str:
        """Do a trick."""
        return name


def test_a_modules_own_rpcs_are_listed_with_a_skill_flag() -> None:
    found = {record["name"]: record for record in rpcs(Dog)}
    assert set(found) == {"bark", "trick"}  # not every Module's start, stop, ...
    assert (
        not found["bark"]["skill"]
        and found["trick"]["skill"]
        and found["trick"]["doc"] == "Do a trick."
    )
    assert found["bark"]["params"] == [
        {"name": "times", "type": "int", "default": "2", "required": False, "choices": None}
    ]
    assert found["trick"]["params"][0]["required"] and found["trick"]["params"][0]["choices"] == [
        "sit",
        "jump",
    ]


def test_the_cache_shares_a_computation_and_drops_expired_ones() -> None:
    cache, computed = Cache(), []

    async def compute() -> int:
        computed.append(1)
        await asyncio.sleep(0.01)
        return len(computed)

    async def scenario() -> None:
        assert await asyncio.gather(*(cache.get("a", 60, compute) for _ in range(3))) == [1, 1, 1]
        await cache.get("b", 0, compute)
        await cache.get("c", 60, compute)

    asyncio.run(scenario())
    assert len(computed) == 3
    assert set(cache.entries) == {"a", "c"}


def test_another_checkout_is_introspected_with_this_file_and_its_own_dimos(tmp_path: Path) -> None:
    # a branch checkout with no gateway in it: its own dimos must be the one imported
    robot = tmp_path / "dimos" / "robot"
    robot.mkdir(parents=True)
    (tmp_path / "dimos" / "__init__.py").write_text("")
    (robot / "__init__.py").write_text("")
    (robot / "get_all_blueprints.py").write_text(
        "def get_by_name(name):\n    raise LookupError(f'{name} from the branch checkout')\n"
    )
    with pytest.raises(IntrospectError, match="r1pro-nav from the branch checkout"):
        asyncio.run(run_child(tmp_path, ["blueprint", "r1pro-nav"], python=sys.executable))


def test_a_checkout_with_an_older_gateway_of_its_own_is_introspected_with_this_one(
    tmp_path: Path,
) -> None:
    # Desktop runs a pinned copy of the gateway against the user's checkout, which may hold an older gateway (an
    # untracked experimental/gateway/ on a dev branch): neither its introspect.py nor its helpers may be the ones run
    robot = tmp_path / "dimos" / "robot"
    robot.mkdir(parents=True)
    (tmp_path / "dimos" / "__init__.py").write_text("")
    (robot / "__init__.py").write_text("")
    (robot / "get_all_blueprints.py").write_text(
        "def get_by_name(name):\n    raise LookupError(f'{name} from the branch checkout')\n"
    )
    old = tmp_path / "experimental" / "gateway"
    old.mkdir(parents=True)
    for helper in ("introspect.py", "constants.py", "launches.py", "store.py"):
        (old / helper).write_text("raise SystemExit('the checkout\\'s own older gateway ran')\n")
    with pytest.raises(IntrospectError, match="r1pro-nav from the branch checkout"):
        asyncio.run(run_child(tmp_path, ["blueprint", "r1pro-nav"]))


def test_a_blueprints_source_is_found_without_importing_its_missing_dependencies(
    tmp_path: Path,
) -> None:
    # an R1 checkout lists unitree-go2 without the go2 packages installed: its source must still be found
    robot = tmp_path / "dimos" / "robot"
    (robot / "go2").mkdir(parents=True)
    for package in (tmp_path / "dimos", robot, robot / "go2"):
        (package / "__init__.py").write_text("")
    (robot / "go2" / "blueprints.py").write_text(
        "import not_installed_here\n\n\nclass Go2Connection: ...\n\n\ngo2 = Go2Connection\n"
    )
    (robot / "all_blueprints.py").write_text(
        "all_blueprints = {'unitree-go2': 'dimos.robot.go2.blueprints:go2'}\n"
        "all_modules = {'go2-connection': 'dimos.robot.go2.blueprints.Go2Connection'}\n"
    )
    for name, line in (("unitree-go2", 7), ("go2-connection", 4)):
        found = asyncio.run(run_child(tmp_path, ["source", name], python=sys.executable))
        assert found == {"name": name, "file": "dimos/robot/go2/blueprints.py", "line": line}


class NeedsArgsConfig(ModuleConfig):
    rate_hz: float = 1.0


class NeedsArgs(Module):
    config: NeedsArgsConfig

    @classmethod
    def blueprint(cls, **kwargs):  # type: ignore[no-untyped-def, override]
        return super().blueprint(must=kwargs["must"])


def test_config_fields_are_read_without_launches_and_its_newer_dimos_imports(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # a checkout older than this gateway lacks what launches.py imports from dimos (DIM-630: every config was a 500)
    monkeypatch.setitem(sys.modules, "experimental.gateway.launches", None)
    args = field_args(NeedsArgsConfig.model_fields, set())
    assert args["rate_hz"]["default"] == 1.0
    assert args["rate_hz"]["secret"] is False


def test_a_module_whose_blueprint_needs_arguments_still_lists_its_config() -> None:
    assert "rate_hz" in module_args(NeedsArgs)


class AuthOptions(BaseModel):
    api_key: str = ""
    region: str = "west"


class RecursiveOptions(BaseModel):
    # the whole annotation is quoted: Python 3.10 leaves a string inside list[...] unresolved
    children: "list[RecursiveOptions]" = Field(default_factory=list)
    region: str = "west"


class CatalogConfig(ModuleConfig):
    auth: AuthOptions = Field(default_factory=AuthOptions)
    profiles: list[AuthOptions] = Field(default_factory=list)
    credentials: dict[str, Any] = Field(default_factory=dict)
    defaults: list[dict[str, str]] = Field(default_factory=lambda: [{"token": "example"}])
    tree: RecursiveOptions = Field(default_factory=RecursiveOptions)


class CatalogModule(Module):
    config: CatalogConfig


def test_catalog_marks_nested_secret_types_and_values() -> None:
    args = field_args(
        CatalogConfig.model_fields,
        set(),
        {"credentials": {"ServiceA": {"api-key": "example", "region": "west"}}},
    )

    assert {
        name: args[name]["secret"]
        for name in ("auth", "profiles", "credentials", "defaults", "tree")
    } == {
        "auth": True,
        "profiles": True,
        "credentials": True,
        "defaults": True,
        "tree": False,
    }


@pytest.mark.parametrize("path", ["auth.api_key", "api-key", "auth_token.value"])
def test_secret_names_include_nested_parents_and_hyphenated_keys(path: str) -> None:
    assert is_secret_name(path)


def test_parser_targets_distinguish_nested_models_from_atomic_collections(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    blueprint = CatalogModule.blueprint().namespace("robot0")
    monkeypatch.setattr(get_all_blueprints, "get_by_name", lambda name: blueprint)

    paths = targets("catalog")

    assert set(paths["robot0/catalogmodule"]) - set(ModuleConfig.model_fields) == {
        "auth.api_key",
        "auth.region",
        "profiles",
        "credentials",
        "defaults",
        "tree.children",
        "tree.region",
    }
    assert "robot_ip" in paths["g"]


def test_targets_apply_launch_globals_before_selecting_blueprint(
    monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    monkeypatch.setenv("SIMULATION", "")
    monkeypatch.setenv("REPLAY", "true")
    monkeypatch.setenv("ROBOT_IPS", "192.0.2.1")
    settings = GlobalConfig(_env_file=None)
    monkeypatch.setattr(global_config_module, "global_config", settings)
    blueprints = {"": Dog.blueprint(), "mujoco": CatalogModule.blueprint()}

    def configured_blueprint(name: str):
        return blueprints[settings.simulation]

    monkeypatch.setattr(get_all_blueprints, "get_by_name", configured_blueprint)
    monkeypatch.setattr(
        sys,
        "stdin",
        StringIO(json.dumps({"simulation": "mujoco", "replay": None, "robot_ips": "192.0.2.2"})),
    )
    emit = mocker.patch.object(introspect, "emit")

    introspect.main(["targets", "catalog"])

    paths = emit.call_args.args[0]
    assert "auth.api_key" in paths["catalogmodule"]
    assert "dog" not in paths
    assert settings.replay is True
    assert settings.processed_robot_ips == ("192.0.2.2",)
