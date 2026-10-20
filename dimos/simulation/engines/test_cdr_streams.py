# Copyright 2025-2026 Dimensional Inc.
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

"""Actual bounded CPU physics and canonical simulation streams, without assets/GPU."""

from contextlib import ExitStack
from functools import partial
from threading import Condition

import numpy as np
import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.image import image_view
from dimos.msgs.occupancy import occupancy_from_array
from dimos.msgs.pointcloud import pointcloud_rgb, pointcloud_xyz
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo as InternalCameraInfo
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import _g1_nav_path
from dimos.robot.unitree.g1.g1_rerun import g1_costmap, g1_odometry_tf_override
from dimos.simulation.engines.mujoco_engine import CameraFrame, MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_real_headless_physics_emits_canonical_odom_imu_and_owns_cleanup(tmp_path, mocker, backend):
    mocker.patch(
        "dimos.simulation.engines.mujoco_sim_module.time.time_ns",
        return_value=1_789_654_321_000_000_001,
    )
    xml = tmp_path / "minimal.xml"
    xml.write_text("""<mujoco>
      <option timestep="0.01" gravity="0 0 0"/>
      <worldbody><body name="base" pos="0 0 1"><freejoint/>
        <geom type="sphere" size="0.1" mass="1"/><site name="imu"/>
        <body name="arm" pos="0 0 0.2"><joint name="hinge" type="hinge"/>
          <geom type="sphere" size="0.05" mass="0.1"/></body>
      </body></worldbody>
      <actuator><motor joint="hinge"/></actuator>
      <sensor><gyro name="gyro" site="imu"/><accelerometer name="accelerometer" site="imu"/></sensor>
    </mujoco>""")
    download = mocker.patch(
        "dimos.utils.data.get_data",
        side_effect=AssertionError("no model/scene download in this test"),
    )
    options = (
        {"url": "memq://"}
        if backend == "lcm"
        else {
            "scouting": False,
            "multicast": False,
            "gossip": False,
            "connect": [],
            "listen": ["tcp/127.0.0.1:0"],
        }
    )
    values = {}
    condition = Condition()

    def receive(name, value):
        with condition:
            values[name] = value
            condition.notify_all()

    with ExitStack() as cleanup:
        module = MujocoSimModule(
            address=xml,
            headless=True,
            dof=1,
            inject_legacy_assets=False,
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            camera_name="",
        )
        cleanup.callback(module.stop)
        for name, kind in [("odom", PoseStamped), ("imu", Imu)]:
            wire = CdrTransport("physics_" + name, kind, backend=backend, **options)
            cleanup.callback(wire.stop)
            module.set_transport(name, wire)
            cleanup.callback(wire.subscribe(partial(receive, name)))
        module.start()
        engine = module._engine
        assert engine is not None
        physics_thread = engine._sim_thread
        publisher_thread = module._publish_thread
        with condition:
            assert condition.wait_for(lambda: set(values) == {"odom", "imu"}, timeout=5)
        assert type(values["odom"]) is PoseStamped and type(values["imu"]) is Imu
        assert values["odom"].header.frame_id == "world"
        assert values["odom"].header.stamp == Time(1_789_654_321, 1)
        assert values["imu"].header.stamp == Time(1_789_654_321, 1)
        assert values["odom"].pose.position.z == pytest.approx(1.0)
        assert values["odom"].pose.orientation.w == pytest.approx(1.0)
        assert values["imu"].orientation.w == pytest.approx(1.0)
        np.testing.assert_array_equal(values["imu"].orientation_covariance, np.zeros(9))
        assert values["imu"].angular_velocity.x == pytest.approx(0.0)
        assert module._shm_ready_signaled
        assert physics_thread is not None and physics_thread.is_alive()
        download.assert_not_called()
    assert not physics_thread.is_alive()
    assert publisher_thread is not None and not publisher_thread.is_alive()
    assert module._engine is None and module._shm is None


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_actual_rgbd_projection_preserves_pixels_source_calibration_and_cloud_at_cdr_ports(
    mocker, backend
):
    options = (
        {"url": "memq://"}
        if backend == "lcm"
        else {
            "scouting": False,
            "multicast": False,
            "gossip": False,
            "connect": [],
            "listen": ["tcp/127.0.0.1:0"],
        }
    )
    pixels = np.full((2, 2, 3), [255, 0, 0], dtype=np.uint8)
    frame = CameraFrame(
        rgb=pixels,
        depth=np.ones((2, 2), dtype=np.float32),
        cam_pos=np.zeros(3),
        cam_mat=np.eye(3),
        fovy=60,
        timestamp=17.125,
    )
    condition = Condition()
    values = {}

    def receive(name, value):
        with condition:
            values[name] = value
            condition.notify_all()

    with ExitStack() as cleanup:
        module = MujocoSimModule(fps=1000)
        cleanup.callback(module.stop)
        engine = mocker.Mock(spec=MujocoEngine)
        engine.connected = True
        mocker.patch.object(module, "_engine", engine)
        calibration = InternalCameraInfo.from_intrinsics(
            width=2, height=2, fx=1, fy=1, cx=0, cy=0, frame_id=module._color_optical_frame
        )
        mocker.patch.object(module, "_camera_info_base", calibration)
        for name, kind in [
            ("color_image", Image),
            ("depth_image", Image),
            ("pointcloud", PointCloud2),
            ("camera_info", CameraInfo),
            ("depth_camera_info", CameraInfo),
        ]:
            wire = CdrTransport("rgbd_" + name, kind, backend=backend, **options)
            cleanup.callback(wire.stop)
            module.set_transport(name, wire)
            cleanup.callback(wire.subscribe(partial(receive, name)))
        calls = [frame]

        def read_camera(name):
            if calls:
                return calls.pop()
            module._stop_event.set()
            return None

        engine.read_camera.side_effect = read_camera
        module._publish_loop()
        module._publish_camera_info()
        engine.read_camera.side_effect = None
        engine.read_camera.return_value = frame
        module._generate_pointcloud()
        with condition:
            assert condition.wait_for(lambda: len(values) == 5, timeout=5)
        assert values["color_image"].encoding == "rgb8"
        assert values["depth_image"].encoding == "32FC1"
        np.testing.assert_array_equal(image_view(values["color_image"]), pixels)
        assert values["color_image"].header.stamp == Time(17, 125_000_000)
        assert values["camera_info"].header.stamp == Time(17, 125_000_000)
        assert values["depth_camera_info"].header.stamp == Time(17, 125_000_000)
        assert type(module.get_color_camera_info()) is CameraInfo
        assert type(module.get_depth_camera_info()) is CameraInfo
        assert values["pointcloud"].header.stamp == Time(17, 125_000_000)
        xyz = pointcloud_xyz(values["pointcloud"])
        xyz = xyz[np.lexsort((xyz[:, 1], xyz[:, 0]))]
        np.testing.assert_array_equal(xyz, [[0, 0, 1], [0, 1, 1], [1, 0, 1], [1, 1, 1]])
        np.testing.assert_array_equal(
            pointcloud_rgb(values["pointcloud"]), np.tile([255, 0, 0], (4, 1))
        )
        direct = module.get_color_camera_info()
        assert direct is not None
        direct.k[0] = 100
        assert calibration.K[0] == 1
        assert values["camera_info"].k[0] == 1
        np.testing.assert_array_equal(pixels, np.full((2, 2, 3), [255, 0, 0], dtype=np.uint8))


def test_g1_custom_visualizers_accept_canonical_grid_path_and_pose_values():
    grid = occupancy_from_array(
        np.array([[-1, 0, 100]], dtype=np.int8), header=Header(Time(17, 1), "map"), resolution=0.5
    )
    mesh = g1_costmap(grid)
    np.testing.assert_allclose(
        mesh.vertex_positions.as_arrow_array().to_pylist(),
        [[0, 0, 0.02], [1.5, 0, 0.02], [1.5, 0.5, 0.02], [0, 0.5, 0.02]],
    )
    pose = PoseStamped(Header(Time(17, 1), "map"), Pose(Point(1, 2, 3), Quaternion(0, 0, 0, 1)))
    path = _g1_nav_path(Path(Header(Time(17, 1), "map"), [pose]))
    assert path.strips.as_arrow_array().to_pylist() == [[[1.0, 2.0, pytest.approx(3.3)]]]
    transform = g1_odometry_tf_override(pose)[0][1]
    assert transform.translation.as_arrow_array().to_pylist() == [[1.0, 2.0, 0.0]]
    assert grid.data.tolist() == [-1, 0, 100] and pose.pose.position.z == 3
