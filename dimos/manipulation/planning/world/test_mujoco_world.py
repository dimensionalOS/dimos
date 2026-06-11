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

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.planning.world.mujoco_world import MujocoWorld
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

_MINI_ARM_XML = """
<mujoco model="mini_arm">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base">
      <geom name="base_geom" type="cylinder" size="0.05 0.05" pos="0 0 0.05"/>
      <body name="link1" pos="0 0 0.1">
        <joint name="j1" type="hinge" axis="0 0 1" range="-3.0 3.0"/>
        <geom name="l1_geom" type="capsule" fromto="0 0 0 0.2 0 0" size="0.03"/>
        <body name="link2" pos="0.2 0 0">
          <joint name="j2" type="hinge" axis="0 1 0" range="-2.5 2.5"/>
          <geom name="l2_geom" type="capsule" fromto="0 0 0 0.2 0 0" size="0.03"/>
          <body name="link3" pos="0.2 0 0">
            <joint name="j3" type="hinge" axis="0 1 0" range="-2.5 2.5"/>
            <geom name="l3_geom" type="capsule" fromto="0 0 0 0.15 0 0" size="0.025"/>
            <body name="ee_link" pos="0.15 0 0">
              <geom name="ee_geom" type="sphere" size="0.02"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

_FLOAT_BOT_XML = """
<mujoco model="floatbot">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base">
      <freejoint name="root"/>
      <geom name="base_geom" type="box" size="0.1 0.1 0.1"/>
      <body name="arm">
        <joint name="j1" type="hinge" axis="0 0 1" range="-3 3"/>
        <geom name="arm_geom" type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""

_JOINTS = ["j1", "j2", "j3"]
_EE_HOME = np.array([0.55, 0.0, 0.1])

_G1_MJCF = Path(__file__).parents[4] / "data" / "mujoco_sim" / "g1_gear_wbc.xml"
_G1_MESHDIR = Path(__file__).parents[4] / "data" / "g1_urdf" / "meshes"


def _identity_pose() -> PoseStamped:
    return PoseStamped(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])


def _arm_config(tmp_path: Path, **overrides) -> RobotModelConfig:
    model = tmp_path / "mini_arm.xml"
    if not model.exists():
        model.write_text(_MINI_ARM_XML)
    kwargs = dict(
        name="mini",
        model_path=model,
        base_pose=_identity_pose(),
        joint_names=_JOINTS,
        end_effector_link="ee_link",
        base_link="base",
    )
    kwargs.update(overrides)
    return RobotModelConfig(**kwargs)


def _make_world(tmp_path: Path, **world_kwargs):
    world = MujocoWorld(**world_kwargs)
    robot_id = world.add_robot(_arm_config(tmp_path))
    return world, robot_id


def _js(positions: list[float]) -> JointState:
    return JointState(name=_JOINTS, position=positions)


def test_fk_home_pose(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.finalize()
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, _js([0.0, 0.0, 0.0]))
        pose = world.get_ee_pose(ctx, robot_id)
    assert np.allclose(list(pose.position), _EE_HOME, atol=1e-9)
    assert pose.frame_id == "world"


def test_fk_rotated_base_joint(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.finalize()
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, _js([np.pi / 2, 0.0, 0.0]))
        pose = world.get_ee_pose(ctx, robot_id)
    assert np.allclose(list(pose.position), [0.0, 0.55, 0.1], atol=1e-9)


def test_grasp_offset_shifts_ee_pose(tmp_path: Path) -> None:
    world = MujocoWorld()
    robot_id = world.add_robot(_arm_config(tmp_path, grasp_offset_xyz=(0.0, 0.0, 0.1)))
    world.finalize()
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, _js([0.0, 0.0, 0.0]))
        pose = world.get_ee_pose(ctx, robot_id)
    assert np.allclose(list(pose.position), _EE_HOME + np.array([0.0, 0.0, 0.1]), atol=1e-9)


def test_joint_limits_from_model(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.finalize()
    lower, upper = world.get_joint_limits(robot_id)
    assert np.allclose(lower, [-3.0, -2.5, -2.5])
    assert np.allclose(upper, [3.0, 2.5, 2.5])


def test_jacobian_matches_numeric_fk(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.finalize()
    q0 = np.array([0.3, -0.4, 0.7])
    h = 1e-6

    def fk(q: np.ndarray) -> np.ndarray:
        with world.scratch_context() as ctx:
            world.set_joint_state(ctx, robot_id, _js(q.tolist()))
            return np.array(list(world.get_ee_pose(ctx, robot_id).position))

    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, _js(q0.tolist()))
        jac = world.get_jacobian(ctx, robot_id)

    assert jac.shape == (6, 3)
    numeric = np.column_stack([(fk(q0 + h * e) - fk(q0)) / h for e in np.eye(3)])
    assert np.allclose(jac[:3], numeric, atol=1e-5)
    # j1 is the base z-hinge: its angular column is exactly ẑ.
    assert np.allclose(jac[3:, 0], [0.0, 0.0, 1.0], atol=1e-9)


def test_sync_from_joint_state_seeds_scratch_contexts(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.finalize()
    world.sync_from_joint_state(robot_id, _js([0.5, 0.1, -0.2]))
    with world.scratch_context() as ctx:
        state = world.get_joint_state(ctx, robot_id)
    assert np.allclose(state.position, [0.5, 0.1, -0.2])


def test_scratch_context_isolation(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.finalize()
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, _js([1.0, 1.0, 1.0]))
    live_state = world.get_joint_state(world.get_live_context(), robot_id)
    assert np.allclose(live_state.position, [0.0, 0.0, 0.0])


def _blocking_obstacle() -> Obstacle:
    # A wall in front of the stretched arm (EE path along +x at z=0.1).
    return Obstacle(
        name="wall",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=[0.4, 0.0, 0.1], orientation=[0.0, 0.0, 0.0, 1.0]),
        dimensions=(0.05, 0.4, 0.4),
    )


def test_prefinalize_obstacle_blocks_stretched_config(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.add_obstacle(_blocking_obstacle())
    world.finalize()
    assert not world.check_config_collision_free(robot_id, _js([0.0, 0.0, 0.0]))
    # Folded up and away from the wall.
    assert world.check_config_collision_free(robot_id, _js([np.pi, 0.0, 0.0]))


def test_subtree_scoping_ignores_base_contacts(tmp_path: Path) -> None:
    """An obstacle overlapping only the (non-planned) base body is invisible
    to the arm's collision check — the feet-on-floor property."""
    world, robot_id = _make_world(tmp_path)
    world.add_obstacle(
        Obstacle(
            name="base_hugger",
            obstacle_type=ObstacleType.BOX,
            pose=PoseStamped(position=[0.0, 0.0, 0.04], orientation=[0.0, 0.0, 0.0, 1.0]),
            dimensions=(0.12, 0.12, 0.06),
        )
    )
    world.finalize()
    # Arm folded straight up: only the base penetrates the obstacle.
    assert world.check_config_collision_free(robot_id, _js([0.0, -np.pi / 2, 0.0]))
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, _js([0.0, -np.pi / 2, 0.0]))
        assert world.get_colliding_pairs(ctx)  # the penetration exists, just out of scope


def test_edge_collision_check(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.add_obstacle(_blocking_obstacle())
    world.finalize()
    # Sweeping through the wall must fail even though both endpoints are free.
    start, end = _js([1.2, 0.0, 0.0]), _js([-1.2, 0.0, 0.0])
    assert world.check_config_collision_free(robot_id, start)
    assert world.check_config_collision_free(robot_id, end)
    assert not world.check_edge_collision_free(robot_id, start, end, step_size=0.05)
    # A sweep on the far side of the workspace is fine.
    assert world.check_edge_collision_free(robot_id, _js([1.8, 0.0, 0.0]), _js([2.8, 0.0, 0.0]))


def test_postfinalize_obstacle_slot_lifecycle(tmp_path: Path) -> None:
    world, robot_id = _make_world(tmp_path)
    world.finalize()
    stretched = _js([0.0, 0.0, 0.0])
    assert world.check_config_collision_free(robot_id, stretched)

    world.add_obstacle(_blocking_obstacle())
    assert not world.check_config_collision_free(robot_id, stretched)

    moved = PoseStamped(position=[0.4, 2.0, 0.1], orientation=[0.0, 0.0, 0.0, 1.0])
    assert world.update_obstacle_pose("wall", moved)
    assert world.check_config_collision_free(robot_id, stretched)

    back = PoseStamped(position=[0.4, 0.0, 0.1], orientation=[0.0, 0.0, 0.0, 1.0])
    world.update_obstacle_pose("wall", back)
    assert not world.check_config_collision_free(robot_id, stretched)

    assert world.remove_obstacle("wall")
    assert world.check_config_collision_free(robot_id, stretched)
    assert world.get_obstacles() == []


def test_postfinalize_mesh_obstacle_rejected(tmp_path: Path) -> None:
    world, _ = _make_world(tmp_path)
    world.finalize()
    with pytest.raises(NotImplementedError):
        world.add_obstacle(
            Obstacle(
                name="mesh_thing",
                obstacle_type=ObstacleType.MESH,
                pose=_identity_pose(),
                mesh_path="nonexistent.obj",
            )
        )


def test_floating_base_pose(tmp_path: Path) -> None:
    model = tmp_path / "floatbot.xml"
    model.write_text(_FLOAT_BOT_XML)
    world = MujocoWorld()
    robot_id = world.add_robot(
        RobotModelConfig(
            name="floatbot",
            model_path=model,
            base_pose=_identity_pose(),
            joint_names=["j1"],
            end_effector_link="arm",
            base_link="base",
            weld_base=False,
        )
    )
    world.finalize()
    world.set_floating_base_pose(
        robot_id, PoseStamped(position=[1.0, 2.0, 3.0], orientation=[0.0, 0.0, 0.0, 1.0])
    )
    with world.scratch_context() as ctx:
        tf = world.get_link_pose(ctx, robot_id, "base")
    assert np.allclose(tf[:3, 3], [1.0, 2.0, 3.0])


def test_entity_pose_sync_changes_collision_verdict(tmp_path: Path) -> None:
    entity = {
        "id": "crate",
        "initial_pose": {"x": 0.4, "y": 0.0, "z": 0.1, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0},
        "aabb": {"min": [0.35, -0.2, 0.0], "max": [0.45, 0.2, 0.2]},
        "descriptor": {
            "entity_id": "crate",
            "kind": "dynamic",
            "shape_hint": "box",
            "extents": [0.1, 0.4, 0.4],
            "mass": 1.0,
        },
        "physics": {"shape": "box"},
    }
    world = MujocoWorld(scene_entities=[entity])
    robot_id = world.add_robot(_arm_config(tmp_path))
    world.finalize()
    stretched = _js([0.0, 0.0, 0.0])
    assert not world.check_config_collision_free(robot_id, stretched)

    world.sync_entity_poses(
        {"crate": PoseStamped(position=[0.4, 2.0, 0.1], orientation=[0.0, 0.0, 0.0, 1.0])}
    )
    assert world.check_config_collision_free(robot_id, stretched)


def test_jacobian_ik_converges(tmp_path: Path) -> None:
    from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK

    world, robot_id = _make_world(tmp_path)
    world.finalize()
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, _js([0.4, -0.3, 0.5]))
        target = world.get_ee_pose(ctx, robot_id)

    result = JacobianIK().solve(
        world,
        robot_id,
        target,
        seed=_js([0.0, 0.0, 0.0]),
        check_collision=False,
    )
    assert result.is_success(), result.message
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, result.joint_state)
        reached = world.get_ee_pose(ctx, robot_id)
    assert np.allclose(list(reached.position), list(target.position), atol=5e-3)


def test_rrt_plans_around_obstacle(tmp_path: Path) -> None:
    from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner

    world, robot_id = _make_world(tmp_path)
    world.add_obstacle(_blocking_obstacle())
    world.finalize()
    result = RRTConnectPlanner().plan_joint_path(
        world, robot_id, _js([1.2, 0.0, 0.0]), _js([-1.2, 0.0, 0.0]), timeout=20.0
    )
    assert result.status == PlanningStatus.SUCCESS, result.message
    assert len(result.path) >= 2


@pytest.mark.skipif(not _G1_MJCF.exists(), reason="G1 MJCF assets not present")
def test_g1_dual_arm_shared_model() -> None:
    def g1_arm(side: str) -> RobotModelConfig:
        joints = [
            f"{side}_shoulder_pitch_joint",
            f"{side}_shoulder_roll_joint",
            f"{side}_shoulder_yaw_joint",
            f"{side}_elbow_joint",
            f"{side}_wrist_roll_joint",
            f"{side}_wrist_pitch_joint",
            f"{side}_wrist_yaw_joint",
        ]
        return RobotModelConfig(
            name="g1",
            model_path=_G1_MJCF,
            model_meshdir=_G1_MESHDIR,
            base_pose=PoseStamped(position=[0.0, 0.0, 0.793], orientation=[0.0, 0.0, 0.0, 1.0]),
            joint_names=joints,
            end_effector_link=f"{side}_wrist_yaw_link",
            base_link="pelvis",
            weld_base=False,
        )

    world = MujocoWorld()
    left = world.add_robot(g1_arm("left"))
    right = world.add_robot(g1_arm("right"), share_model_with=left)
    world.finalize()

    with world.scratch_context() as ctx:
        zeros = JointState(name=world.get_robot_config(left).joint_names, position=[0.0] * 7)
        world.set_joint_state(ctx, left, zeros)
        left_pose = world.get_ee_pose(ctx, left)
        right_pose = world.get_ee_pose(ctx, right)
        assert world.get_jacobian(ctx, left).shape == (6, 7)
    # Mirror symmetry of the arms at zero pose.
    assert left_pose.position[1] > 0.05 > -0.05 > right_pose.position[1]
    assert abs(left_pose.position[1] + right_pose.position[1]) < 1e-3

    # Arms at zero config don't self-collide.
    assert world.check_config_collision_free(left, zeros)


@pytest.mark.skipif(not _G1_MJCF.exists(), reason="G1 MJCF assets not present")
def test_g1_feet_on_floor_is_not_an_arm_collision(tmp_path: Path) -> None:
    floor_xml = tmp_path / "floor.xml"
    floor_xml.write_text(
        '<mujoco model="floor"><worldbody>'
        '<geom name="floor" type="plane" size="0 0 0.01"/>'
        "</worldbody></mujoco>"
    )
    world = MujocoWorld(scene_xml=floor_xml)
    left = world.add_robot(
        RobotModelConfig(
            name="g1",
            model_path=_G1_MJCF,
            model_meshdir=_G1_MESHDIR,
            base_pose=PoseStamped(position=[0.0, 0.0, 0.793], orientation=[0.0, 0.0, 0.0, 1.0]),
            joint_names=[
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint",
                "left_elbow_joint",
                "left_wrist_roll_joint",
                "left_wrist_pitch_joint",
                "left_wrist_yaw_joint",
            ],
            end_effector_link="left_wrist_yaw_link",
            base_link="pelvis",
            weld_base=False,
        )
    )
    world.finalize()
    # Drop the pelvis until the feet penetrate the floor.
    world.set_floating_base_pose(
        left, PoseStamped(position=[0.0, 0.0, 0.70], orientation=[0.0, 0.0, 0.0, 1.0])
    )
    zeros = JointState(name=world.get_robot_config(left).joint_names, position=[0.0] * 7)
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, left, zeros)
        assert world.get_colliding_pairs(ctx), "expected feet/floor penetration"
        assert world.is_collision_free(ctx, left), "feet contacts must not block arm planning"
