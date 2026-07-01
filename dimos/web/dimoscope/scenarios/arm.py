#!/usr/bin/env python3
# scope-arm — a manipulation (robot arm) scenario. Namespace: /arm/*
#
#   /arm/joint_states  JointState       @250 Hz   7-DOF telemetry, small
#   /arm/ee_pose       PoseStamped      @100 Hz   end-effector pose, tiny
#   /arm/imu           Imu              @500 Hz   wrist IMU, tiny, EXTREME rate
#   /arm/trajectory    JointTrajectory  @  2 Hz   a 10-point plan, bursty
#
# Data-path axis: MESSAGE-RATE CEILING — ~850 tiny msgs/s. This stresses per-message overhead
# (framing, wakeups) rather than bandwidth. /arm/ee_pose renders as an arrow in WorldView; the
# high-rate telemetry is best seen in the Bench tab's live monitor (hz + latency sparkline) and the
# Inspector (JSON). NOTE: JointTrajectory carries no ts/frame_id (it uses .timestamp), so the bench
# reports n/a latency/loss for /arm/trajectory by design — it still streams + inspects. (A force/
# torque WrenchStamped would be the natural F/T topic, but it has no LCM wire codec, so the 500 Hz
# stream is a wrist Imu — the canonical high-rate arm telemetry.)
#
# Run (from dimos/web/dimoscope):  DIMOS_TRANSPORT=zenoh uv run python scenarios/arm.py
from common import (
    IDENT,
    Imu,
    JointState,
    JointTrajectory,
    Module,
    ModuleConfig,
    Out,
    PoseStamped,
    Seq,
    TrajectoryPoint,
    Vector3,
    env_f,
    env_i,
    math,
    rpc,
    run_standalone,
    rx,
    time,
)


class ScopeArmConfig(ModuleConfig):
    joint_hz: float = env_f("SCOPE_ARM_JOINT_HZ", 250.0)
    ee_hz: float = env_f("SCOPE_ARM_EE_HZ", 100.0)
    imu_hz: float = env_f("SCOPE_ARM_IMU_HZ", 500.0)
    traj_hz: float = env_f("SCOPE_ARM_TRAJ_HZ", 2.0)
    n_joints: int = env_i("SCOPE_ARM_JOINTS", 7)


class ScopeArm(Module):
    """Manipulation scenario: joint states + end-effector pose + wrist IMU + trajectory."""

    config: ScopeArmConfig
    joint_states: Out[JointState]
    ee_pose: Out[PoseStamped]
    imu: Out[Imu]
    trajectory: Out[JointTrajectory]

    @rpc
    def start(self) -> None:
        c = self.config
        seq = Seq()
        t0 = time.time()
        names = [f"joint_{i}" for i in range(c.n_joints)]

        def tick_joints(_: int) -> None:
            t = time.time() - t0
            pos = [0.5 * math.sin(0.5 * t + i) for i in range(c.n_joints)]
            self.joint_states.publish(
                JointState(
                    ts=time.time(),
                    frame_id=seq("joints"),
                    name=names,
                    position=pos,
                    velocity=[0.0] * c.n_joints,
                    effort=[0.0] * c.n_joints,
                )
            )

        def tick_ee(_: int) -> None:
            t = time.time() - t0
            self.ee_pose.publish(
                PoseStamped(
                    ts=time.time(),
                    frame_id=seq("ee"),
                    position=(0.5 + 0.1 * math.cos(t), 0.1 * math.sin(t), 0.3),
                    orientation=IDENT,
                )
            )

        def tick_imu(_: int) -> None:
            t = time.time() - t0
            self.imu.publish(
                Imu(
                    ts=time.time(),
                    frame_id=seq("imu"),
                    angular_velocity=Vector3(0.0, 0.0, 0.2 * math.sin(t)),
                    linear_acceleration=Vector3(math.sin(t), math.cos(t), 9.8),
                )
            )

        def tick_traj(_: int) -> None:
            pts = [
                TrajectoryPoint(
                    time_from_start=0.1 * k,
                    positions=[0.1 * k] * c.n_joints,
                    velocities=[0.0] * c.n_joints,
                )
                for k in range(10)
            ]
            self.trajectory.publish(
                JointTrajectory(points=pts, joint_names=names, timestamp=time.time())
            )

        if c.joint_hz > 0:
            self.register_disposable(rx.interval(1.0 / c.joint_hz).subscribe(tick_joints))
        if c.ee_hz > 0:
            self.register_disposable(rx.interval(1.0 / c.ee_hz).subscribe(tick_ee))
        if c.imu_hz > 0:
            self.register_disposable(rx.interval(1.0 / c.imu_hz).subscribe(tick_imu))
        if c.traj_hz > 0:
            self.register_disposable(rx.interval(1.0 / c.traj_hz).subscribe(tick_traj))


scope_arm = ScopeArm.blueprint()

# (attr, topic, MsgType) — the shared topic↔type source of truth (runtime wiring + `gen_types.py`).
PORTS = [
    ("joint_states", "/arm/joint_states", JointState),
    ("ee_pose", "/arm/ee_pose", PoseStamped),
    ("imu", "/arm/imu", Imu),
    ("trajectory", "/arm/trajectory", JointTrajectory),
]

if __name__ == "__main__":
    run_standalone(ScopeArm(), PORTS, "scope-arm")
