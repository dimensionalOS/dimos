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

"""The robot API as it would read in use. For review; nothing here runs yet.

    RobotModel   what a robot is: links, joints, limits, base. Immutable, saved as URDF.
    RobotState   where it is at a moment: a model + joint values. Immutable.
    Robot        a model read from a source, live or recorded: model.attach(source, name).
    RobotSource  where a Robot reads: tf + joint states + model parts (LiveSource,
                 StoreSource).
    RobotAware   a module mixin: `self.robot` to read, `publish_robot` for what you own.

Sections: 1 model, 2 state, 3 modules, 4 several robots, 5 memory, 6 config.
"""

# ruff: noqa: B018
# mypy: ignore-errors
# (calls the target API with placeholder data, e.g. URDFs that aren't in the repo yet)

from __future__ import annotations

from dataclasses import replace
import math
from typing import Annotated, Any

from dimos.core.module import Module
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.robot.aware import RobotAware, RobotAwareConfig
from dimos.protocol.robot.config import Named
from dimos.protocol.robot.joints import joint_values
from dimos.protocol.robot.model import Joint, JointType, Limits, RobotModel
from dimos.protocol.robot.rerun import RobotLogger
from dimos.protocol.robot.source import StoreSource
from dimos.utils.data import get_data

# 1. The model: what the robot is


def load_and_inspect() -> RobotModel:
    go2 = RobotModel.from_urdf(get_data("go2_description") / "urdf" / "go2.urdf")
    go2 = go2.rename_frames({"base": "base_link", "radar": "utlidar_lidar"})
    go2.urdf_name  # "go2", the URDF's <robot name>; kept for to_urdf, nothing more

    calf = go2.joints["FR_calf_joint"]
    calf.type  # JointType.REVOLUTE
    calf.parent, calf.child  # ("FR_thigh", "FR_calf")
    calf.limits.velocity  # 15.7 rad/s
    calf.limits.lower, calf.limits.upper  # (-2.72, -0.84) rad
    calf.limits.contains(-1.5)  # True

    go2.joints.movable.names  # the 12 leg joints, URDF order (mimics excluded)
    go2.joints.select("*_calf_joint")  # the four calves
    go2.links["base_link"].inertial  # mass, inertia
    go2.base.twist.vx  # 1.5 m/s, what planners condition on
    go2.chain("base_link", "FR_foot")  # hip, thigh, calf, foot joints
    go2.digest  # sha256 of the canonical URDF; equal models, equal digests
    return go2


def edit(go2: RobotModel) -> RobotModel:
    # Every edit returns a new model. One path-based editor for any field:
    go2 = go2.edit("joints.FR_hip_joint.limits.lower", -0.8)
    go2 = go2.edit("joints.*.limits.velocity", lambda v: v * 0.5)  # derate everything
    go2 = go2.edit("base.twist.vx", 1.0)  # a slower rig, carrying a payload
    go2 = go2.edit("links.base_link.inertial.mass", 6.9)
    go2 = go2.edit("joints.FR_hip_joint.origin", Pose(0.1934, -0.0465, 0.0))

    # ...and a typed one for joints, when you'd rather have mypy check the fields.
    go2 = go2.map_joints(
        "*_calf_joint",
        lambda j: replace(j, limits=replace(j.limits, velocity=12.0, acceleration=200.0)),
    )

    # A joint built by hand: a pan unit for the camera.
    go2 = go2.with_joint(
        Joint(
            name="camera_pan_joint",
            type=JointType.REVOLUTE,
            parent="front_camera",
            child="camera_pan",
            limits=Limits(lower=-math.pi / 2, upper=math.pi / 2, velocity=3.0),
        )
    )
    return go2.fix("camera_pan_joint", at=0.0)  # changed our mind: lock it


def rigs(go2: RobotModel) -> tuple[RobotModel, RobotModel]:
    # Mounts are plain tf edges. with_frames overrides: utlidar_lidar already exists, so
    # its joint moves (the URDF's is ~120 deg off); the rest are new fixed joints.
    sf = go2.with_frames(
        Transform.from_xyz_rpy("base_link", "front_camera", xyz=(0.32715, 0.0, 0.04297)),
        Transform.from_xyz_rpy(
            "front_camera", "mid360_link", xyz=(-0.032, 0.0, 0.12), rpy=(0, 60, 0), degrees=True
        ),
        Transform.from_xyz_rpy(
            "front_camera", "camera_optical", rpy=(-math.pi / 2, 0.0, -math.pi / 2)
        ),
        Transform.from_xyz_rpy(
            "base_link",
            "utlidar_lidar",
            xyz=(0.226, 0.025, 0.089),
            rpy=(-167.877, -7.907, -59.588),
            degrees=True,
        ),
    )
    athens = sf.with_frames(  # differs by one edge
        Transform.from_xyz_rpy(
            "front_camera", "mid360_link", xyz=(-0.032, 0.0, 0.12), rpy=(-60, 0, -90), degrees=True
        ),
    )
    athens.get("base_link", "mid360_link")  # mounts answer straight off the model
    athens.to_urdf("go2_athens.urdf")  # rerun, MuJoCo, MoveIt all see the rig
    return sf, athens


def compose(go2: RobotModel) -> RobotModel:
    # Robots and tf edges are the same kind of thing; merge joins them on frame names.
    lite6 = RobotModel.from_urdf(get_data("lite6_description") / "lite6.urdf")

    # The Lite 6 names nothing the Go2 does (link_base, link1..6), so it merges as is.
    with_arm = go2.merge(
        lite6,
        Transform.from_xyz_rpy("base_link", "link_base", xyz=(0.0, 0.0, 0.08)),
    )
    with_arm.without("link_base")  # and off again

    # An arm whose root is also called base_link clashes with the dog's:
    #   go2.merge(other_arm, ...)  raises MergeConflictError, suggesting other_arm.prefixed(...)
    other_arm = RobotModel.from_urdf(get_data("other_arm") / "arm.urdf")
    go2.merge(
        other_arm.prefixed("arm/"),
        Transform.from_xyz_rpy("base_link", "arm/base_link", xyz=(0.0, 0.0, 0.08)),
    )

    # Two of the same arm clash on every name: prefixing is how they coexist.
    dual = go2.merge(
        lite6.prefixed("left/"),
        Transform.from_xyz_rpy("base_link", "left/link_base", xyz=(0.0, 0.1, 0.08)),
        lite6.prefixed("right/"),
        Transform.from_xyz_rpy("base_link", "right/link_base", xyz=(0.0, -0.1, 0.08)),
    )
    dual.joints.select("left/*").movable.names  # one arm's 6 joints
    return with_arm


def without_a_urdf() -> RobotModel:
    # Rigs and robots without a URDF start from one frame.
    return RobotModel.root("base_link").with_frames(
        Transform.from_xyz_rpy("base_link", "mid360_link", xyz=(0.20, -0.20, 0.30)),
    )


# 2. The state: where the robot is


def state(go2: RobotModel, msg: JointState, cmd: Twist) -> None:
    crouch = go2.state({"FR_thigh_joint": 0.8, "FR_calf_joint": -1.5})
    crouch.get("base_link", "FR_foot")  # forward kinematics
    crouch.joints["RL_calf_joint"].position  # None: not given, not a fake 0

    now = go2.state(msg)  # a JointState or {joint: position}
    calf = now.joints["FR_calf_joint"]
    calf.position, calf.velocity  # this moment
    calf.limits.velocity  # the model's, through the view
    calf.within_limits  # None if the position is unknown

    for v in now.violations():  # unknown joints are skipped
        print(f"{v.joint}: {v.quantity} {v.value:.2f} past {v.limit:.2f}")
    now.clamped()  # positions pulled back into their limits
    now.model.base.twist.clamp(cmd)  # a command the base can actually do

    RobotLogger(go2).log(now)  # rerun: meshes once, link poses each call

    # Hardware joint names that differ from the model's.
    joint_values(msg, names=lambda hw: hw.removeprefix("motor_"))


# 3. Modules: read the robot, publish the parts you own


def modules(go2: RobotModel, arm: RobotModel) -> None:
    class Planner(Module, RobotAware):
        # self.robot is config.robot attached to this module's ports (self.source).
        def step(self) -> None:
            self.robot.joints["FR_calf_joint"].position  # quick reads: latest
            self.robot.model.base.twist.vx

            s = self.robot.state(max_age=0.1)  # one snapshot; StaleError if joints are old
            s.joints["FR_calf_joint"].position
            s.get("map", "FR_foot")  # world frames through the source at s.ts
            then = self.robot.state(s.ts - 0.2)  # 200 ms ago, joints interpolated
            s.joints["FR_calf_joint"].position - then.joints["FR_calf_joint"].position

    class Go2Connection(Module, RobotAware):
        # Owns the body: publishes the dog's model, its legs and its odometry.
        def start(self) -> None:
            self.publish_robot(model=self.config.robot)  # re-sent at model_hz for late joiners

        def on_lowstate(self, q: dict[str, float], qd: dict[str, float]) -> None:
            self.publish_robot(positions=q, velocities=qd)

        def on_odom(self, odom_to_base: Transform) -> None:
            self.publish_robot(pose=odom_to_base)  # the only robot edge that goes on tf

    class ArmConnection(Module, RobotAware):
        # Owns the arm: its subtree, its mount and its joints. Same robot, second owner.
        def start(self) -> None:
            mount = Transform.from_xyz_rpy("base_link", "link_base", xyz=(0, 0, 0.08))
            self.publish_robot(model=arm.merge(mount))

        def on_feedback(self, q: dict[str, float]) -> None:
            self.publish_robot(positions=q)  # only the arm's joints; the legs are untouched

    class L1Calibrator(Module, RobotAware):
        # Publishes a corrected mount: a one-edge model part, not tf.
        def on_fit(self, edge: Transform) -> None:
            self.publish_robot(frames=[edge], part="l1_calibration")

    # Which model a reader sees:
    #   - Planner with config.robot set: that model, plus received parts it lacks (the
    #     arm). Its own mounts are authoritative: the calibrator doesn't move them.
    #   - Planner with config.robot=None: the received model, all parts merged; the
    #     calibrator's edge applies (newest distinct content wins).
    # If the arm's connection dies its joints go stale; state(max_age=...) says so.


# 4. Several robots: one set of topics, names keep them apart


def fleet(go2: RobotModel, source: Any) -> None:
    # The name is where the robot runs, not what it is: one GO2_ATHENS serves every dog.
    # A connection with robot_name="go2b" publishes go2b/base_link, go2b/FR_calf_joint,
    # go2b/odom. Frames outside the model (map) stay global; localization ties each
    # dog's odom to it (map -> go2b/odom).
    dog = go2.attach(source, name="go2b")
    dog.joints["FR_calf_joint"].position  # reads go2b/FR_calf_joint
    dog.get("map", "FR_foot")  # FR_foot -> go2b/FR_foot; map stays map

    # Attached without a name when the bus only has named robots, the first read raises:
    #   no frame "base_link"; did you mean name="go2a" or name="go2b"?

    # In a RobotAware module: self.robots["go2b"] is go2b's received model, attached.

    # A dog with an arm, named go2b, publishes the arm as go2b/link6 (or go2b/arm/link6
    # if the arm was prefixed to avoid a clash).


# 5. Memory: the same reads over a recording


def memory(store: Any, go2: RobotModel, go2_fixed: RobotModel) -> None:
    # The recorder is RobotAware: it records tf, joint_state, and robot_model on change.
    source = StoreSource.from_store(store)  # the three streams, loaded per window together
    robot = go2.attach(source)  # same attach as live; only the source differs

    t = 1791993294.5
    robot.joints["FR_calf_joint"].position  # latest = end of the recording
    robot.get("odom", "front_camera", t)
    RobotLogger(go2).log(robot.state(t))

    for s in robot.states(t, t + 10.0, hz=50):  # a replay or analysis loop
        s.joints["FR_calf_joint"].velocity

    # The rig the recording was made with, from its robot_model stream.
    recorded = RobotModel.root().merge(*(p.model for p in source.parts()))
    recorded.attach(source)

    # Re-process old data with a corrected calibration: the attached model wins.
    go2_fixed.attach(source).get("odom", "utlidar_lidar", t)


# 6. Config: rigs by name


def config() -> None:
    # In go2/structure.py: the named rigs and a field type that accepts their names.
    go2_rigs: dict[str, RobotModel] = {}  # {"sf": GO2_SF, "athens": GO2_ATHENS}
    go2_robot = Annotated[RobotModel, Named(go2_rigs)]

    class Go2ConnectionConfig(RobotAwareConfig):
        robot: go2_robot = "sf"  # a name, a model, or a URDF path

    # Switching rig and robot name, from any layer:
    #
    #   go2_connection(robot="athens")                          blueprint, by name
    #   go2_connection(robot=GO2_ATHENS.edit(...))              blueprint, a one-off variant
    #   go2_connection(robot="athens", robot_name="go2b")       one dog of a fleet
    #   GO2CONNECTION__ROBOT=athens                             .env on the jetson
    #   --go2connection.robot athens --go2connection.robot-name go2b    CLI
    #
    # Edits to the model itself happen in Python; the config dump prints "athens" while the
    # value equals GO2_ATHENS, else "<RobotModel 23 links, 12 joints>".
