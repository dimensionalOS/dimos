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

import ast
from collections.abc import Iterable, Iterator
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT

DIMOS_DIR = DIMOS_PROJECT_ROOT / "dimos"

# Importing any of these at module level loads a heavyweight native library into
# every process that transitively imports the module - including worker
# processes that never touch it. Import them inside the function/method that
# uses them (or under `if TYPE_CHECKING:` for annotations only).
HEAVY_MODULES = ("cv2", "open3d", "rerun")

# Files that predate this rule and still import heavy modules eagerly.
# Do NOT add to this list -- new code must import HEAVY_MODULES inline.
# When you fix a file, remove it here
GRANDFATHERED = {
    "core/o3dpickle.py",
    "experimental/scene_cooking/browser/collision.py",
    "experimental/scene_cooking/mujoco/collision_export.py",
    "experimental/scene_cooking/source_assets/inspect.py",
    "experimental/scene_cooking/source_assets/mesh.py",
    "experimental/security_demo/security_module.py",
    "hardware/sensors/camera/realsense/camera.py",
    "hardware/sensors/camera/webcam.py",
    "hardware/sensors/camera/zed/camera.py",
    "manipulation/grasping/visualize_grasps.py",
    "mapping/loop_closure/pgo.py",
    "mapping/loop_closure/pgo_auto.py",
    "mapping/loop_closure/utils/markers_rrd.py",
    "mapping/occupancy/visualizations.py",
    "mapping/occupancy/visualize_path.py",
    "mapping/pointclouds/accumulators/general.py",
    "mapping/pointclouds/accumulators/protocol.py",
    "mapping/pointclouds/demo.py",
    "mapping/pointclouds/util.py",
    "mapping/ray_tracing/transformer.py",
    "mapping/ray_tracing/utils/raytrace_rrd.py",
    "mapping/relocalization/relocalize.py",
    "mapping/voxels/grid.py",
    "mapping/voxels/impl/o3d.py",
    "memory2/vis/utils.py",
    "models/segmentation/edge_tam.py",
    "msgs/sensor_msgs/PointCloud2.py",
    "navigation/cmu_nav/modules/simple_planner/simple_planner.py",
    "navigation/nav_3d/evaluator/blueprints.py",
    "navigation/nav_3d/evaluator/mesh_loader.py",
    "navigation/nav_3d/mls_planner/utils/plan_rrd.py",
    "perception/common/utils.py",
    "perception/detection/objectDB.py",
    "perception/detection/type/detection2d/seg.py",
    "perception/detection/type/detection3d/object.py",
    "perception/fiducial/fixture_verification.py",
    "perception/image_embedding.py",
    "perception/object_scene_registration.py",
    "perception/object_tracker.py",
    "perception/object_tracker_2d.py",
    "perception/object_tracker_3d.py",
    "perception/perceive_loop_skill.py",
    "perception/spatial_perception.py",
    "perception/visual_memory.py",
    "robot/drone/drone_tracking_module.py",
    "robot/manipulators/xarm/blueprints/worldbelief.py",
    "robot/unitree/g1/blueprints/primitive/unitree_g1_vis.py",
    "robot/unitree/go2/dds/cli/render.py",
    "robot/unitree/type/lidar.py",
    "robot/unitree/type/map.py",
    "simulation/genesis/stream.py",
    "simulation/isaac/stream.py",
    "simulation/mujoco/depth_camera.py",
    "simulation/mujoco/mujoco_process.py",
    "simulation/unity/module.py",
    "skills/manipulation/pick_and_place.py",
    "stream/video_provider.py",
    "utils/cli/apriltag.py",
    "utils/cli/cameracalibrate/cameracalibrate.py",
    "utils/extract_frames.py",
    "visualization/rerun/bridge.py",
    "visualization/rerun/init.py",
    "visualization/rerun/resource_stats.py",
    "web/dimos_interface/api/server.py",
    "web/relay_bridge/demo_smoke.py",
}


def _is_type_checking(test: ast.expr) -> bool:
    """True for the guard of `if TYPE_CHECKING:` / `if typing.TYPE_CHECKING:`."""
    if isinstance(test, ast.Name):
        return test.id == "TYPE_CHECKING"
    if isinstance(test, ast.Attribute):
        return test.attr == "TYPE_CHECKING"
    return False


def _iter_eager_nodes(nodes: Iterable[ast.AST]) -> Iterator[ast.AST]:
    """Yield nodes whose code runs at import time.

    Skips function/lambda bodies (imports there are inline, i.e. deferred) and
    the body of `if TYPE_CHECKING:` blocks (never executed at runtime). The
    `else:` branch of a TYPE_CHECKING guard does run at import time.
    """
    for node in nodes:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            continue
        if isinstance(node, ast.If) and _is_type_checking(node.test):
            yield from _iter_eager_nodes(node.orelse)
            continue
        yield node
        yield from _iter_eager_nodes(ast.iter_child_nodes(node))


def _heavy_import(node: ast.Import | ast.ImportFrom) -> str | None:
    """The heavy module an import statement pulls in, or None."""
    if isinstance(node, ast.Import):
        for alias in node.names:
            if alias.name.split(".")[0] in HEAVY_MODULES:
                return alias.name
    if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
        if node.module.split(".")[0] in HEAVY_MODULES:
            return node.module
    return None


def find_eager_heavy_imports() -> dict[str, list[tuple[int, str]]]:
    """Map of dimos-relative file path -> [(line, module)] for eager heavy imports."""
    hits: dict[str, list[tuple[int, str]]] = {}
    for path in sorted(DIMOS_DIR.rglob("*.py")):
        if path.name.startswith("test_") or path.name == "conftest.py":
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in _iter_eager_nodes(tree.body):
            if not isinstance(node, (ast.Import, ast.ImportFrom)):
                continue
            module = _heavy_import(node)
            if module is not None:
                hits.setdefault(str(path.relative_to(DIMOS_DIR)), []).append((node.lineno, module))
    return hits


def test_heavy_imports_are_inline() -> None:
    """Fail if a non-grandfathered file imports cv2/open3d at module level."""
    hits = find_eager_heavy_imports()
    new = {f: lines for f, lines in hits.items() if f not in GRANDFATHERED}
    if new:
        listing = "\n".join(
            f"  - dimos/{f}:{line}: `{module}`"
            for f, lines in sorted(new.items())
            for line, module in lines
        )
        raise AssertionError(
            f"Found module-level import(s) of {'/'.join(HEAVY_MODULES)}:\n{listing}\n\n"
            "These libraries load large native extensions into every process that "
            "transitively imports the module. Import them inside the function or "
            "method that uses them; imports needed only for type annotations go "
            "under `if TYPE_CHECKING:`."
        )


def test_grandfathered_list_is_current() -> None:
    """Fail if a grandfathered file no longer has eager heavy imports (ratchet)."""
    stale = GRANDFATHERED - set(find_eager_heavy_imports())
    if stale:
        listing = "\n".join(f"  - dimos/{f}" for f in sorted(stale))
        raise AssertionError(
            f"These files no longer import {'/'.join(HEAVY_MODULES)} eagerly -- remove "
            f"them from GRANDFATHERED in {Path(__file__).name} so they stay clean:\n{listing}"
        )
