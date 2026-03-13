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

from dimos.core.blueprints import autoconnect
from dimos.core.module import Module, ModuleConfig

# from dimos.robot.cli.dimos import arghelp
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.rerun.bridge import RerunBridgeModule, _default_blueprint


def test_blueprint_arghelp():
    blueprint = autoconnect(RerunBridgeModule.blueprint(), GO2Connection.blueprint())
    output = arghelp(blueprint.config(), blueprint)
    assert output.split("\n") == [
        "    rerunbridgemodule:",
        "      * rerunbridgemodule.frame_id_prefix: str | None (default: None)",
        "      * rerunbridgemodule.frame_id: str | None (default: None)",
        "      * rerunbridgemodule.entity_prefix: str (default: world)",
        "      * rerunbridgemodule.topic_to_entity: collections.abc.Callable[[typing.Any], str] | None (default: None)",
        "      * rerunbridgemodule.viewer_mode: typing.Literal['native', 'web', 'connect', 'none']",
        "      * rerunbridgemodule.connect_url: str (default: rerun+http://127.0.0.1:9877/proxy)",
        "      * rerunbridgemodule.memory_limit: str (default: 25%)",
        f"      * rerunbridgemodule.blueprint: collections.abc.Callable[rerun.blueprint.api.Blueprint] | None (default: {_default_blueprint})",
        "    go2connection:",
        "      * go2connection.frame_id_prefix: str | None (default: None)",
        "      * go2connection.frame_id: str | None (default: None)",
        "      * go2connection.ip: str",
        "",
    ]


def test_blueprint_arghelp_extra_args():
    """Test defaults passed to .blueprint() override."""

    bridge = RerunBridgeModule.blueprint(frame_id_prefix="foo", viewer_mode="web")
    blueprint = autoconnect(bridge, GO2Connection.blueprint(ip="1.1.1.1"))
    output = arghelp(blueprint.config(), blueprint)
    assert output.split("\n") == [
        "    rerunbridgemodule:",
        "      * rerunbridgemodule.frame_id_prefix: str | None (default: foo)",
        "      * rerunbridgemodule.frame_id: str | None (default: None)",
        "      * rerunbridgemodule.entity_prefix: str (default: world)",
        "      * rerunbridgemodule.topic_to_entity: collections.abc.Callable[[typing.Any], str] | None (default: None)",
        "      * rerunbridgemodule.viewer_mode: typing.Literal['native', 'web', 'connect', 'none'] (default: web)",
        "      * rerunbridgemodule.connect_url: str (default: rerun+http://127.0.0.1:9877/proxy)",
        "      * rerunbridgemodule.memory_limit: str (default: 25%)",
        f"      * rerunbridgemodule.blueprint: collections.abc.Callable[rerun.blueprint.api.Blueprint] | None (default: {_default_blueprint})",
        "    go2connection:",
        "      * go2connection.frame_id_prefix: str | None (default: None)",
        "      * go2connection.frame_id: str | None (default: None)",
        "      * go2connection.ip: str (default: 1.1.1.1)",
        "",
    ]


def test_blueprint_arghelp_required():
    """Test required arguments."""

    class Config(ModuleConfig):
        foo: int
        spam: str = "eggs"

    class TestModule(Module[Config]):
        default_config = Config

    blueprint = TestModule.blueprint()
    output = arghelp(blueprint.config(), blueprint)
    assert output.split("\n") == [
        "    testmodule:",
        "      * testmodule.frame_id_prefix: str | None (default: None)",
        "      * testmodule.frame_id: str | None (default: None)",
        "      * [Required] testmodule.foo: int",
        "      * testmodule.spam: str (default: eggs)",
        "",
    ]
