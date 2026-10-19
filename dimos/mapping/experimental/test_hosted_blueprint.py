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


from dimos.hosted.fragment_compiler import compile_fragments, resolve_hosted_assignments
from dimos.hosted.protocol import HostDescriptor
from dimos.mapping.experimental.blueprints import go2_dds_nav_hosted


def test_go2_hosted_splits_robot_mesher_and_viewer() -> None:
    go2 = HostDescriptor("go2-id", "go2jetson", {"go2": "", "gpu": "jetson"}, "rev", ())
    gpu = HostDescriptor("gpu-id", "compute", {"gpu": "RTX 5080", "gpu_mem_gb": "16"}, "rev", ())
    laptop = HostDescriptor("laptop", "laptop", {}, "rev", ())
    assignments = resolve_hosted_assignments(
        go2_dds_nav_hosted, [go2, gpu, laptop], local_host_id="laptop", revision="rev"
    )
    fragments = compile_fragments(go2_dds_nav_hosted, assignments, run_id="run", revision="rev")

    robot = fragments["go2-id"].load_payload()
    viewer = fragments["laptop"].load_payload()
    mesher = fragments["gpu-id"].load_payload()
    assert {a.name for a in mesher.blueprint.active_blueprints} == {"meshmodule"}
    assert {a.name for a in viewer.blueprint.active_blueprints} == {
        "rerunbridgemodule",
        "rerunwebsocketserver",
    }
    assert "go2dds" in {a.name for a in robot.blueprint.active_blueprints}
    assert mesher.remote_module_references == ()
