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

from pathlib import Path

import cv2
import numpy as np

from dimos.mapping.floorplan.generator import (
    FloorplanOptions,
    FloorplanResult,
    check_floor_plausibility,
    generate_floorplan,
)
from dimos.mapping.reconstruction.scene_model import SceneModel


def _synthetic_building() -> SceneModel:
    """A 8 x 5 x 2.8 m single-story space with walls, ceiling and a doorway."""
    rng = np.random.default_rng(3)
    pts = []

    def _plane(xs, ys, zs, n):  # type: ignore[no-untyped-def]
        pts.append(np.column_stack([rng.uniform(*xs, n), rng.uniform(*ys, n), rng.uniform(*zs, n)]))

    _plane((0, 8), (0, 5), (0.0, 0.03), 6000)  # floor
    _plane((0, 8), (0, 5), (2.75, 2.8), 6000)  # ceiling (indoor evidence)
    _plane((0, 3.4), (0, 0.06), (0, 2.7), 2500)  # south wall, west of doorway
    _plane((4.4, 8), (0, 0.06), (0, 2.7), 2500)  # south wall, east of doorway
    _plane((0, 8), (4.94, 5), (0, 2.7), 4000)  # north wall
    _plane((0, 0.06), (0, 5), (0, 2.7), 2500)  # west wall
    _plane((7.94, 8), (0, 5), (0, 2.7), 2500)  # east wall
    _plane((5.5, 6.5), (2.0, 3.0), (0.7, 0.76), 600)  # a table

    # trajectory: robot drove around inside and out through the doorway
    xs = np.linspace(1.0, 7.0, 60)
    traj = np.column_stack([xs, np.full_like(xs, 2.5), np.full_like(xs, 0.3)])
    door_pass = np.column_stack([np.full(10, 3.9), np.linspace(2.5, -1.0, 10), np.full(10, 0.3)])
    trajectory = np.vstack([traj, door_pass]).astype(np.float32)
    return SceneModel(np.vstack(pts).astype(np.float32), trajectory)


def test_generate_floorplan_from_injected_model(tmp_path: Path) -> None:
    opts = FloorplanOptions(
        out=tmp_path / "plan",
        ai_review=False,  # geometric pipeline only — no network
        wall_z=1.6,
        close_loops=False,  # keep the test light
        project="TEST BUILDING",
    )
    result = generate_floorplan(opts, model=_synthetic_building())

    assert result.dxf.exists() and result.dxf.stat().st_size > 5_000
    assert result.jpeg.exists() and result.jpeg.stat().st_size > 20_000
    assert result.model_rrd is None  # save_3d_model defaults off
    assert result.debug_dir is None  # debug artifacts default off...
    leftovers = list(tmp_path.glob("*.debug-*")) + list(tmp_path.glob("*.review-*"))
    assert not leftovers  # ...and none leak next to the outputs

    assert len(result.sheets) == 1
    stats = result.sheets[0]
    assert stats.walls >= 4  # the four-ish wall runs of the room
    assert stats.doors >= 1  # the doorway the robot drove through
    assert 6.0 <= stats.extent_m[0] <= 11.0  # ~8 m wide + margins

    summary = result.summary()
    assert "LEVEL 1" in summary and "doors" in summary and str(result.dxf) in summary


def test_generate_floorplan_save_3d_model(tmp_path: Path) -> None:
    opts = FloorplanOptions(
        out=tmp_path / "plan",
        ai_review=False,
        wall_z=1.6,
        close_loops=False,
        save_3d_model=True,
    )
    result = generate_floorplan(opts, model=_synthetic_building())
    assert result.model_rrd is not None and result.model_rrd.exists()
    assert "dimos-viewer" in result.summary()


def test_floorplan_result_agent_encode(tmp_path: Path) -> None:
    """agent_encode() attaches renditions as images, and skips unreadable ones."""
    good_render = tmp_path / "plan.render-drafted.png"
    cv2.imwrite(str(good_render), np.zeros((4, 4, 3), dtype=np.uint8))
    missing_render = tmp_path / "plan.render-cyanotype.png"  # never written

    result = FloorplanResult(
        dxf=tmp_path / "plan.dxf",
        jpeg=tmp_path / "plan.jpg",
        sheets=[],
        renders=[good_render, missing_render],
    )

    content = result.agent_encode()

    assert content[0] == {"type": "text", "text": result.summary()}
    image_blocks = [c for c in content if c.get("type") == "image_url"]
    assert len(image_blocks) == 1  # the missing render is skipped, not raised
    assert image_blocks[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")


def test_floorplan_result_agent_encode_no_renders(tmp_path: Path) -> None:
    result = FloorplanResult(dxf=tmp_path / "plan.dxf", jpeg=tmp_path / "plan.jpg", sheets=[])
    assert result.agent_encode() == [{"type": "text", "text": result.summary()}]


def test_floor_plausibility_flags_drifted_trajectory() -> None:
    """A drift-split floor stack (observed: 3 real stories detected as 6 spanning
    43 m of z, with a 30 m 'story') must be flagged; real stacks must not."""
    drifted = [-40.08, -35.98, -33.68, -3.48, -0.38, 3.22]
    warning = check_floor_plausibility(drifted)
    assert warning is not None and "drift" in warning

    assert check_floor_plausibility([0.08, 3.28, 6.78]) is None  # the real building
    assert check_floor_plausibility([0.0]) is None  # single story
    assert check_floor_plausibility([]) is None


def test_drift_warning_lands_in_summary(tmp_path: Path) -> None:
    result = FloorplanResult(
        dxf=tmp_path / "plan.dxf",
        jpeg=tmp_path / "plan.jpg",
        sheets=[],
        warnings=["odometry drift suspected"],
    )
    assert "WARNING: odometry drift suspected" in result.summary()
