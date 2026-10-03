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

"""CPU native filtering at the camera-to-mapper boundary."""

import importlib
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
from open3d.core import Tensor
import pytest
import trimesh

pytest.importorskip("roboplan.core")

from dimos.core.stream import Out
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.assets.model import PlanarBaseDefinition, RobotModel

_URDF = """<robot name="test" version="1.0">
<link name="base"/><link name="arm"><collision><geometry>
<box size="0.2 0.2 0.2"/></geometry></collision></link>
<joint name="slide" type="prismatic"><parent link="base"/><child link="arm"/>
<axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="1" velocity="1" acceleration="2"/>
</joint></robot>"""


@pytest.fixture
def make_module(tmp_path, monkeypatch):
    modules = []

    def make(*, xml=_URDF, joint_names=("slide",), planar=False, **overrides):
        model_path = tmp_path / f"robot-{len(modules)}.urdf"
        model_path.write_text(xml)
        model = RobotModel.from_file(model_path)
        base_link = "base"
        if planar:
            definition = PlanarBaseDefinition(
                velocity_limits=(1, 1, 1), acceleration_limits=(2, 2, 2)
            )
            model = model.with_planar_base(definition)
            base_link = definition.root_link
            joint_names = (*definition.joint_names, *joint_names)
        config = RobotModelConfig(
            model=model,
            joint_names=list(joint_names),
            base_link=base_link,
            base_pose=PoseStamped(frame_id="world", position=[1, 0, 0]),
            planning_groups=[
                PlanningGroupDefinition(
                    name="arm", joint_names=tuple(joint_names), base_link=base_link, tip_link="arm"
                )
            ],
        )
        world_type = importlib.reload(
            importlib.import_module("dimos.manipulation.planning.world.roboplan_world")
        ).RoboPlanWorld
        world = world_type()
        world.load_model(prepare_robot_model(config))
        world.finalize()
        module = ManipulationModule(
            model=config,
            filter_robot_points=True,
            pointcloud_match_tolerance_s=0.001,
            **overrides,
        )
        monkeypatch.setattr(module, "_world_monitor", WorldMonitor(world))
        monkeypatch.setattr(module, "_tf", MultiTBuffer())
        modules.append(module)
        return module

    yield make
    for module in modules:
        monkeypatch.setattr(module, "_tf", None)
        module.dispose()


def _state(module, stamp=1.0, position=0.0):
    module._on_joint_state(JointState(ts=stamp, name=["slide"], position=[position]))


def _tf(module, stamp=1.0, translation=(1, 0, 0), rotation=None):
    module.tfbuffer.receive_transform(
        Transform(
            frame_id="world",
            child_frame_id="camera",
            ts=stamp,
            translation=Vector3(*translation),
            rotation=rotation or Quaternion(),
        )
    )


def _cloud(points, stamp=1.0):
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)), frame_id="camera", timestamp=stamp
    )


def test_surface_returns_are_removed_before_mapping_and_fields_survive(make_module):
    module = make_module()
    _state(module)
    _tf(module)
    cloud = _cloud([[0.1, 0, 0], [0.105, 0, 0], [0.2, 0, 0], [0.109, 0.109, 0]])
    cloud.pointcloud_tensor.point["intensities"] = Tensor(
        np.array([[1], [2], [3], [4]], np.float32)
    )
    cloud.pointcloud_tensor.point["labels"] = Tensor(np.array([[7], [8], [9], [10]], np.int32))

    filtered = module._filter_pointcloud(cloud)

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[0.2, 0, 0], [0.109, 0.109, 0]])
    np.testing.assert_array_equal(filtered.intensities_f32().reshape(-1), [3, 4])
    np.testing.assert_array_equal(filtered.pointcloud_tensor.point["labels"].numpy(), [[9], [10]])
    assert filtered.frame_id == "camera" and filtered.ts == 1.0


def test_capture_state_is_used_instead_of_latest_state(make_module):
    module = make_module()
    _state(module, position=0.0)
    _state(module, stamp=2.0, position=0.5)
    module._world_monitor.world.sync_from_joint_state(JointState(name=["slide"], position=[0.5]))
    _tf(module)

    filtered = module._filter_pointcloud(_cloud([[0.1, 0, 0], [0.6, 0, 0]]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[0.6, 0, 0]])
    assert module._pointcloud_states.find_closest(1.0, 0.001).ts == 1.0


def test_capture_tf_is_received_from_camera_transport(make_module, monkeypatch):
    module = make_module()
    monkeypatch.setattr(module, "_tf", None)
    transport = MagicMock()
    callbacks = []

    def subscribe(callback, _stream):
        callbacks.append(callback)
        return lambda: callbacks.remove(callback)

    transport.subscribe.side_effect = subscribe
    transport.broadcast.side_effect = lambda _stream, msg: [cb(msg) for cb in callbacks]
    module.tf.transport = transport
    camera_tf = Out(TFMessage, "tf")
    camera_tf.transport = transport
    _state(module)
    # Subscribe before the independent camera publishes its capture transform.
    assert module.tfbuffer.get("world", "camera", time_point=1.0) is None
    camera_tf.publish(
        TFMessage(
            Transform(
                frame_id="world",
                child_frame_id="camera",
                ts=1.0,
                translation=Vector3(1, 0, 0),
                rotation=Quaternion(),
            )
        )
    )

    filtered = module._filter_pointcloud(_cloud([[0.1, 0, 0], [0.2, 0, 0]]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[0.2, 0, 0]])
    module.tfbuffer.dispose()


@pytest.mark.parametrize("missing", ["state", "tf", "stale_state", "stale_tf"])
def test_missing_or_stale_capture_alignment_drops_the_capture(make_module, missing):
    module = make_module()
    if missing != "state":
        _state(module, stamp=0.9 if missing == "stale_state" else 1.0)
    if missing != "tf":
        _tf(module, stamp=0.9 if missing == "stale_tf" else 1.0)

    assert module._filter_pointcloud(_cloud([[0.1, 0, 0]])) is None


def test_capture_rotation_and_prepared_base_pose_are_respected(make_module):
    module = make_module()
    _state(module)
    _tf(module, rotation=Quaternion.from_euler(Vector3(0, 0, np.pi / 2)))

    filtered = module._filter_pointcloud(_cloud([[0, -0.1, 0], [0, -0.2, 0]]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[0, -0.2, 0]])


def test_surface_mesh_filter_needs_no_solid_volume_sampling(make_module, tmp_path):
    path = Path(tmp_path) / "cube.stl"
    trimesh.creation.box(extents=[0.2, 0.2, 0.2]).export(path)
    module = make_module(
        xml=_URDF.replace('<box size="0.2 0.2 0.2"/>', f'<mesh filename="{path}"/>')
    )
    _state(module)
    _tf(module)

    filtered = module._filter_pointcloud(_cloud([[0.1, 0, 0], [0.105, 0, 0], [0.2, 0, 0]]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[0.2, 0, 0]])


def test_late_state_cannot_resurrect_expired_capture(make_module):
    module = make_module(pointcloud_state_history_s=1.0)
    _state(module, stamp=3.0)
    _state(module, stamp=1.0)
    _tf(module)

    assert module._filter_pointcloud(_cloud([[0.1, 0, 0]])) is None


def test_corrected_state_replaces_same_timestamp(make_module):
    module = make_module()
    _state(module, position=0.5)
    _state(module, position=0.0)
    _tf(module)

    filtered = module._filter_pointcloud(_cloud([[0.1, 0, 0]]))

    assert filtered is not None
    assert len(filtered) == 0


def test_out_of_order_cloud_cannot_publish_after_newer_capture(make_module):
    module = make_module()
    _state(module, stamp=2.0)
    _tf(module, stamp=2.0)
    assert module._filter_pointcloud(_cloud([[0.2, 0, 0]], stamp=2.0)) is not None
    _state(module, stamp=1.0)
    _tf(module, stamp=1.0)

    assert module._filter_pointcloud(_cloud([[0.2, 0, 0]], stamp=1.0)) is None


@pytest.mark.parametrize("positions", [[float("nan")], [], [0, 1]])
def test_malformed_joint_states_do_not_authorize_filtering(make_module, positions):
    module = make_module()
    module._on_joint_state(JointState(ts=1.0, name=["slide"], position=positions))
    _tf(module)

    assert module._filter_pointcloud(_cloud([[0.1, 0, 0]])) is None


@pytest.mark.parametrize(
    "shape,inside,outside",
    [
        ('<sphere radius="0.1"/>', [0.105, 0, 0], [0.12, 0, 0]),
        ('<cylinder radius="0.1" length="0.2"/>', [0, 0, 0.105], [0, 0, 0.12]),
    ],
)
def test_native_primitives_exclude_surface_returns(make_module, shape, inside, outside):
    module = make_module(xml=_URDF.replace('<box size="0.2 0.2 0.2"/>', shape))
    _state(module)
    _tf(module)

    filtered = module._filter_pointcloud(_cloud([inside, outside]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [outside])


def test_continuous_joint_maps_full_native_configuration(make_module):
    xml = (
        _URDF.replace('type="prismatic"', 'type="continuous"')
        .replace('lower="-2" upper="2" ', "")
        .replace('<axis xyz="1 0 0"/>', '<origin xyz="0 0 0"/><axis xyz="0 0 1"/>')
        .replace("<collision><geometry>", '<collision><origin xyz="0.3 0 0"/><geometry>')
    )
    module = make_module(xml=xml)
    _state(module, position=np.pi / 2)
    _tf(module)

    filtered = module._filter_pointcloud(_cloud([[0, 0.4, 0], [0.4, 0, 0]]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[0.4, 0, 0]])


def test_mimic_joint_is_derived_by_the_prepared_native_model(make_module):
    xml = _URDF.replace(
        "</robot>",
        """
<link name="replica"><collision><geometry><box size="0.2 0.2 0.2"/>
</geometry></collision></link>
<joint name="z_copy" type="prismatic"><parent link="base"/><child link="replica"/>
<origin xyz="0.5 0 0"/><axis xyz="1 0 0"/>
<limit lower="-2" upper="2" effort="1" velocity="1" acceleration="2"/>
<mimic joint="slide" multiplier="2" offset="0"/></joint></robot>
""",
    )
    module = make_module(xml=xml)
    _state(module, position=0.1)
    _tf(module)

    filtered = module._filter_pointcloud(_cloud([[0.2, 0, 0], [0.8, 0, 0], [1, 0, 0]]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[1, 0, 0]])


def test_fixed_collision_link_follows_its_moving_parent(make_module):
    xml = _URDF.replace(
        "</robot>",
        """
<link name="fixed_body"><collision><geometry><box size="0.2 0.2 0.2"/>
</geometry></collision></link><joint name="mount" type="fixed">
<parent link="arm"/><child link="fixed_body"/><origin xyz="0.5 0 0"/>
</joint></robot>
""",
    )
    module = make_module(xml=xml)
    _state(module, position=0.2)
    _tf(module)

    filtered = module._filter_pointcloud(_cloud([[0.3, 0, 0], [0.8, 0, 0], [1, 0, 0]]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[1, 0, 0]])


def test_prepared_planar_base_uses_capture_coordinates(make_module):
    module = make_module(planar=True)
    module._on_joint_state(
        JointState(
            ts=1.0,
            name=["base/x", "base/y", "base/yaw", "slide"],
            position=[0.5, 0, np.pi / 2, 0.2],
        )
    )
    _tf(module)

    filtered = module._filter_pointcloud(_cloud([[0.5, 0.3, 0], [0.5, 0.5, 0]]))

    assert filtered is not None
    np.testing.assert_allclose(filtered.points_f32(), [[0.5, 0.5, 0]])


def test_empty_and_nonfinite_clouds_are_handled_without_updating_alignment(make_module):
    module = make_module()
    _state(module)
    _tf(module)

    assert module._filter_pointcloud(_cloud([[float("nan"), 0, 0]])) is None
    filtered = module._filter_pointcloud(_cloud([]))
    assert filtered is not None and len(filtered) == 0


def test_only_filtered_points_are_published_to_the_mapper(make_module, mocker):
    module = make_module()
    _state(module)
    _tf(module)
    publish = mocker.patch.object(module.filtered_pointcloud, "publish")

    module._publish_filtered_pointcloud(_cloud([[0.1, 0, 0], [0.2, 0, 0]]))

    publish.assert_called_once()
    np.testing.assert_allclose(publish.call_args.args[0].points_f32(), [[0.2, 0, 0]])
