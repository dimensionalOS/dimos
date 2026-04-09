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

from typing import Literal

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.robot.cli.dimos import arg_help


def test_blueprint_arg_help():
    class ConfigA(ModuleConfig):
        min_interval_sec: float = 0.1
        entity_prefix: str = "world"
        viewer_mode: Literal["native", "web", "connect", "none"] = "native"

    class TestModuleA(Module):
        config: ConfigA

    class ConfigB(ModuleConfig):
        memory_limit: str = "25%"
        ip: str = "127.0.0.1"

    class TestModuleB(Module):
        config: ConfigB

    blueprint = autoconnect(TestModuleA.blueprint(), TestModuleB.blueprint())
    output = arg_help(blueprint.config(), blueprint)
    # List output produces better diff in pytest error output.
    assert output.split("\n") == [
        "    testmodulea:",
        "      * testmodulea.default_rpc_timeout: float (default: 120.0)",
        "      * testmodulea.frame_id_prefix: str | None (default: None)",
        "      * testmodulea.frame_id: str | None (default: None)",
        "      * testmodulea.min_interval_sec: float (default: 0.1)",
        "      * testmodulea.entity_prefix: str (default: world)",
        "      * testmodulea.viewer_mode: typing.Literal['native', 'web', 'connect', 'none'] (default: native)",
        "    testmoduleb:",
        "      * testmoduleb.default_rpc_timeout: float (default: 120.0)",
        "      * testmoduleb.frame_id_prefix: str | None (default: None)",
        "      * testmoduleb.frame_id: str | None (default: None)",
        "      * testmoduleb.memory_limit: str (default: 25%)",
        "      * testmoduleb.ip: str (default: 127.0.0.1)",
        "",
    ]


def test_blueprint_arg_help_extra_args():
    """Test defaults passed to .blueprint() override."""

    class ConfigA(ModuleConfig):
        frame_id_prefix: str | None = None
        min_interval_sec: float = 0.1
        entity_prefix: str = "world"
        viewer_mode: Literal["native", "web", "connect", "none"] = "native"

    class TestModuleA(Module):
        config: ConfigA

    class ConfigB(ModuleConfig):
        memory_limit: str = "25%"
        ip: str = "127.0.0.1"

    class TestModuleB(Module):
        config: ConfigB

    module_a = TestModuleA.blueprint(frame_id_prefix="foo", viewer_mode="web")
    blueprint = autoconnect(module_a, TestModuleB.blueprint(ip="1.1.1.1"))
    output = arg_help(blueprint.config(), blueprint)
    # List output produces better diff in pytest error output.
    assert output.split("\n") == [
        "    testmodulea:",
        "      * testmodulea.default_rpc_timeout: float (default: 120.0)",
        "      * testmodulea.frame_id_prefix: str | None (default: foo)",
        "      * testmodulea.frame_id: str | None (default: None)",
        "      * testmodulea.min_interval_sec: float (default: 0.1)",
        "      * testmodulea.entity_prefix: str (default: world)",
        "      * testmodulea.viewer_mode: typing.Literal['native', 'web', 'connect', 'none'] (default: web)",
        "    testmoduleb:",
        "      * testmoduleb.default_rpc_timeout: float (default: 120.0)",
        "      * testmoduleb.frame_id_prefix: str | None (default: None)",
        "      * testmoduleb.frame_id: str | None (default: None)",
        "      * testmoduleb.memory_limit: str (default: 25%)",
        "      * testmoduleb.ip: str (default: 1.1.1.1)",
        "",
    ]


def test_blueprint_arg_help_required():
    """Test required arguments."""

    class Config(ModuleConfig):
        foo: int
        spam: str = "eggs"

    class TestModule(Module):
        config: Config

    blueprint = TestModule.blueprint()
    output = arg_help(blueprint.config(), blueprint)
    assert output.split("\n") == [
        "    testmodule:",
        "      * testmodule.default_rpc_timeout: float (default: 120.0)",
        "      * testmodule.frame_id_prefix: str | None (default: None)",
        "      * testmodule.frame_id: str | None (default: None)",
        "      * [Required] testmodule.foo: int",
        "      * testmodule.spam: str (default: eggs)",
        "",
    ]
