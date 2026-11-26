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

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.sim_msgs import WorldState
import numpy as np
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
from dimos.sim2.description import scene_revision
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer

# Requires an offscreen OpenGL context on the graphics-capable runner.
pytestmark = pytest.mark.self_hosted


@pytest.fixture
def make_renderer(world):
    with ExitStack() as cleanup:

        def load(cameras, scene=world):
            renderer = MujocoRenderer()
            cleanup.callback(renderer.close)
            renderer.load(scene, cameras)
            return renderer

        yield load


@pytest.fixture
def renderer(make_renderer):
    return make_renderer(
        (
            CameraConfig("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"),
            CameraConfig("world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera"),
        )
    )


def test_camera_renders_recorded_body_poses_without_physics_state(
    backend, poses, renderer, motor_commands
):
    sample = poses()
    rgb, depth = renderer.render(
        sample, ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
    assert rgb.std() > 5
    assert depth.shape == (240, 320) and np.isfinite(depth).all()
    backend.apply_action(motor_commands(), [])
    for _ in range(300):
        backend.step()
    moved = poses()
    after, _ = renderer.render(
        moved, ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
    # Require visible motion without coupling the check to scene brightness.
    changed_pixels = np.count_nonzero(np.max(np.abs(after.astype(float) - rgb), axis=-1) > 10)
    assert changed_pixels > 0.01 * rgb.shape[0] * rgb.shape[1]
    replay, _ = renderer.render(
        sample, ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
    # Filament color grading dithers by a few 8-bit levels between captures.
    np.testing.assert_allclose(replay.astype(float), rgb.astype(float), atol=5)
    replay_depth = renderer.render(
        sample, ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"][1]
    np.testing.assert_array_equal(replay_depth, depth)


def test_a_camera_renders_only_the_images_it_has(poses, make_renderer):
    renderer = make_renderer(
        (
            CameraConfig(
                "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",
                depth=False,
            ),
            CameraConfig(
                "world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera",
                color=False,
            ),
        )
    )
    images = renderer.render(
        poses(),
        (
            "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",
            "world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera",
        ),
    )
    rgb, no_depth = images["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
    assert rgb.shape == (240, 320, 3) and rgb.std() > 5
    assert no_depth.size == 0
    no_color, depth = images[
        "world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera"
    ]
    assert no_color.size == 0
    assert depth.shape == (240, 320) and np.isfinite(depth).all()
    with pytest.raises(ValueError, match="color, depth or both"):
        CameraConfig("neither", color=False, depth=False)


def test_usd_authored_lights_illuminate_camera_image(poses, world, make_renderer):
    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, world.usd, "lights", None))
    light = stage.GetPrimAtPath("/world/light")
    light.GetAttribute("inputs:intensity").Set(0.0)
    dark_scene = UsdModel.from_layer(stage.GetRootLayer().ExportToString())
    cameras = (
        CameraConfig(
            "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera", depth=False
        ),
    )
    lit, _ = make_renderer(cameras).render(
        poses(), ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
    sample = poses()
    sample.model_revision = scene_revision(dark_scene)
    dark, _ = make_renderer(cameras, dark_scene).render(
        sample, ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
    assert lit.mean() > dark.mean() + 5


def test_two_views_retain_their_pixels(poses, renderer):
    sample = poses(time_ns=200_000_000)
    images = renderer.render(
        sample,
        (
            "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",
            "world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera",
        ),
    )
    left, right = (
        images["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"][0],
        images["world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera"][0],
    )
    assert np.count_nonzero(np.any(left != right, axis=-1)) > 0.01 * 320 * 240
    saved = left.copy()
    renderer.render(
        sample, ("world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )
    np.testing.assert_array_equal(left, saved)


def test_different_resolutions_keep_intrinsics_and_share_the_model(poses, make_renderer):
    renderer = make_renderer(
        (
            CameraConfig(
                "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",
                width=160,
                height=120,
                fx=94.18,
                fy=94.18,
                cx=80,
                cy=60,
            ),
            CameraConfig("world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera"),
        )
    )
    images = renderer.render(
        poses(),
        (
            "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",
            "world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera",
        ),
    )
    assert images["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"][
        0
    ].shape == (120, 160, 3)
    assert images["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"][
        1
    ].shape == (120, 160)
    assert images["world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera"][
        0
    ].shape == (240, 320, 3)
    assert images["world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera"][
        1
    ].shape == (240, 320)


def test_any_pinhole_calibration_renders_where_its_camera_info_says(poses, make_renderer):
    # The same view through an ordinary camera and through one with unequal focal
    # lengths and an off-center principal point: every surface point one sees must
    # be where the other's calibration puts it.
    centered = CameraConfig(
        "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"
    )
    skewed = CameraConfig(
        "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",
        width=300,
        height=200,
        fx=150,
        fy=230,
        cx=170,
        cy=80,
    )
    sample = poses()
    _, reference = make_renderer((centered,)).render(
        sample, ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
    _, depth = make_renderer((skewed,)).render(
        sample, ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
    )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
    assert depth.shape == (200, 300)
    v, u = np.mgrid[5:195:7, 5:295:7]
    z = depth[v, u]
    x = (u + 0.5 - skewed.cx) / skewed.fx * z
    y = (v + 0.5 - skewed.cy) / skewed.fy * z
    ur = np.floor(centered.fx * x / z + centered.cx).astype(int)
    vr = np.floor(centered.fy * y / z + centered.cy).astype(int)
    seen = (z > 0) & (ur >= 0) & (ur < centered.width) & (vr >= 0) & (vr < centered.height)
    assert seen.sum() > 200
    error = np.abs(reference[vr[seen], ur[seen]] - z[seen])
    # Object edges land on the neighbouring surface; everything else agrees to a centimetre.
    assert np.mean(error < 0.01) > 0.95
    assert np.unique(np.round(z[seen], 2)).size > 20  # A real scene, not one flat depth.


def test_sensor_thread_can_capture_rgbd_and_close_without_a_main_thread_event_loop(world, poses):
    sample = poses()

    def capture():
        renderer = MujocoRenderer()
        try:
            renderer.load(
                world,
                (
                    CameraConfig(
                        "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"
                    ),
                ),
            )
            process = renderer._process
            rgb, depth = renderer.render(
                sample, ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
            )["world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"]
        finally:
            renderer.close()
        assert process.poll() == 0
        assert rgb.std() > 5
        assert np.count_nonzero(depth > 0) > 100

    with ThreadPoolExecutor(max_workers=1) as executor:
        executor.submit(capture).result(timeout=30)


@pytest.mark.parametrize("distance", [0.25, 2.0, 10.0])
def test_depth_is_optical_axis_metres_and_matches_color_silhouette(distance):
    scene = UsdModel.from_layer(f"""#usda 1.0
    (defaultPrim = "world"; metersPerUnit = 1; upAxis = "Z")
    def Xform "world" {{
        def Xform "camera" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]) {{
            bool physics:kinematicEnabled = true
            float physics:mass = 1
            float3 physics:diagonalInertia = (1, 1, 1)
        }}
        def Cube "target" {{
            double size = 2
            double3 xformOp:translate = (0, 0, {distance + 0.05})
            double3 xformOp:scale = ({distance * 0.25}, {distance * 0.2}, 0.05)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
            color3f[] primvars:displayColor = [(1, 0, 0)]
        }}
    }}
    """)
    backend = MujocoBackend()
    renderer = MujocoRenderer()
    try:
        backend.load(scene, 2_000_000)
        bodies = backend.read_bodies()
        sample = WorldState(
            header=SampleHeader(),
            model_revision=scene_revision(scene),
            bodies_length=len(bodies),
            bodies=bodies,
        )
        camera = CameraConfig("world/camera", width=201, height=151, fx=150, fy=180, cx=110, cy=65)
        renderer.load(scene, (camera,))
        rgb, depth = renderer.render(sample, ("world/camera",))["world/camera"]
        valid = depth > 0
        assert valid.sum() > 1000
        # A flat surface has constant Z even away from the principal point;
        # Euclidean ray distance would increase toward its edges.
        np.testing.assert_allclose(depth[valid], distance, atol=2e-4)
        assert depth[0, 0] == 0
        r, g, b = np.moveaxis(rgb.astype(float), -1, 0)
        red = (r > g * 2) & (r > b * 2) & (r > 20)
        assert np.count_nonzero(red & valid) / np.count_nonzero(red | valid) > 0.95
        rows, cols = np.where(valid)
        np.testing.assert_allclose(
            [cols.mean() + 0.5, rows.mean() + 0.5], [camera.cx, camera.cy], atol=1
        )
    finally:
        renderer.close()
        backend.close()


def test_cancel_stops_native_render_process_and_allows_reloading(world, poses):
    renderer = MujocoRenderer()
    try:
        renderer.load(
            world,
            (
                CameraConfig(
                    "world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera"
                ),
            ),
        )
        process = renderer._process
        renderer.cancel()
        process.wait(timeout=5)
        with pytest.raises(RuntimeError, match="closed"):
            renderer.render(
                poses(), ("world/world/left_mount/cartesian_pusher/base_link/carriage/tool/camera",)
            )
        renderer.close()
        renderer.load(
            world,
            (
                CameraConfig(
                    "world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera",
                    color=False,
                ),
            ),
        )
        _, depth = renderer.render(
            poses(), ("world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera",)
        )["world/world/right_mount/cartesian_pusher/base_link/carriage/tool/camera"]
        assert np.count_nonzero(depth > 0) > 100
    finally:
        renderer.close()
