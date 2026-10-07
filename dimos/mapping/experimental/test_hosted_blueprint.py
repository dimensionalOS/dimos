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
from dimos.hosted.daemon import HostDescriptor
from dimos.hosted.fragment_compiler import compile_fragments
from dimos.mapping.experimental.blueprints import go2_dds_nav_hosted


def test_go2_hosted_splits_robot_from_viewer() -> None:
    go2 = HostDescriptor("go2-id", "e", "go2", frozenset(), {}, "available", ())
    config = BlueprintConfigParser(go2_dds_nav_hosted).parse(environ={})
    fragments = compile_fragments(
        go2_dds_nav_hosted,
        config,
        run_id="run",
        generation=1,
        application_name="go2",
        application_revision="rev",
        hosts=[go2],
        local_host_id="laptop",
    )

    robot = fragments["go2-id"].load_payload()
    viewer = fragments["laptop"].load_payload()
    assert {a.name for a in viewer.blueprint.active_blueprints} == {
        "meshmodule",
        "rerunbridgemodule",
        "rerunwebsocketserver",
    }
    # GO2DDS follows the daemon's client session instead of opening its own router.
    (go2dds,) = [a for a in robot.blueprint.active_blueprints if a.name == "go2dds"]
    assert go2dds.kwargs.get("session") is None
    assert "map_regions" in {b.name for b in robot.boundary_streams}
