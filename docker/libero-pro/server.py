"""Container-owned LIBERO-PRO simulator and split gRPC interfaces."""

from __future__ import annotations

from concurrent import futures
import json
import os
from pathlib import Path
import pickle
import signal
import threading
import time
import zipfile

import grpc  # type: ignore[import-untyped]
import numpy as np

from dimos.benchmark.libero_pro.proto import libero_pro_pb2 as pb2, libero_pro_pb2_grpc as pb2_grpc

GRIPPER_APERTURE_TOLERANCE_M = 1e-4


class Runtime:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.environment = None
        self.snapshot = None
        self.snapshot_version = 0
        self.target = None
        self.result = None
        self.active = False
        self.stop_requested = False
        self.horizon = 0
        self.frequency = 20
        self.policy_ticks = 0
        self.backend_ticks = 0
        self.task_name = ""
        self.instruction = ""
        self.initialize_request = None
        self.initialize_result = None
        self.initialize_error = None
        self.initialization_requested = False
        self.initialize_done = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def initialize(self, request) -> tuple[str, str]:
        with self.condition:
            if self.initialization_requested:
                raise RuntimeError("trial initialization has already been requested")
            self.initialization_requested = True
            self.initialize_request = request
            self.condition.notify_all()
        self.initialize_done.wait()
        if self.initialize_error is not None:
            raise self.initialize_error
        return self.initialize_result

    def _initialize_environment(self, request) -> tuple[str, str]:
        try:
            from libero.envs import OffScreenRenderEnv
        except ImportError:
            from libero.libero.envs import OffScreenRenderEnv

        manifest = json.loads(Path("/task/task.json").read_text())
        task = manifest["task"]
        if (request.suite, request.task_order_index, request.task_index) != (
            task["suite"],
            task["task_order_index"],
            task["task_index"],
        ):
            raise ValueError("trial selection does not match mounted task manifest")
        self.task_name = str(task["task_name"])
        self.instruction = str(task["instruction"])
        self.environment = OffScreenRenderEnv(
            bddl_file_name="/task/task.bddl",
            robots=["Panda"],
            use_camera_obs=True,
            has_renderer=False,
            has_offscreen_renderer=True,
            camera_heights=128,
            camera_widths=128,
            camera_names=["agentview", "robot0_eye_in_hand"],
            controller="JOINT_POSITION",
            control_freq=request.control_frequency_hz,
            horizon=request.horizon_ticks + request.settling_ticks,
        )
        with zipfile.ZipFile("/task/init_states.pruned_init") as archive:
            states = pickle.loads(archive.read("archive/data.pkl"))
        if request.init_state_index >= len(states):
            raise IndexError("initial-state index is outside the mounted tensor")
        self.environment.reset()
        observation = self.environment.set_init_state(states[request.init_state_index])
        for _ in range(request.settling_ticks):
            observation, _, _, _ = self.environment.step(np.zeros(8, dtype=np.float64))
            self.backend_ticks += 1
        self.frequency = request.control_frequency_hz
        self.horizon = request.horizon_ticks
        self.policy_ticks = 0
        self.result = None
        self.active = False
        self.target = self._measured_target(observation)
        self._publish(observation, 0)
        return self.task_name, self.instruction

    def stop(self) -> None:
        with self.condition:
            self.active = False
            self.stop_requested = True
            self.condition.notify_all()
        self.thread.join()

    def start(self) -> None:
        with self.condition:
            if self.environment is None or self.result is not None:
                raise RuntimeError("trial is not ready")
            self.active = True
            self.condition.notify_all()

    def cancel(self) -> object:
        with self.condition:
            if self.result is None:
                self.result = self._terminal(False, 0.0, "cancelled")
            self.active = False
            self.condition.notify_all()
            return self.result

    def wait_result(self) -> object:
        with self.condition:
            while self.result is None:
                self.condition.wait()
            return self.result

    def update_target(self, request) -> None:
        if len(request.joint_position) != 7:
            raise ValueError("exactly seven Panda joint targets are required")
        with self.condition:
            self.target = np.array([*request.joint_position, request.gripper_position])

    def _run(self) -> None:
        try:
            while True:
                with self.condition:
                    while (
                        self.initialize_request is None
                        and not self.active
                        and not self.stop_requested
                    ):
                        self.condition.wait()
                    if self.stop_requested:
                        return
                    initialize_request = self.initialize_request
                    self.initialize_request = None
                if initialize_request is not None:
                    try:
                        self.initialize_result = self._initialize_environment(initialize_request)
                    except BaseException as exc:
                        self.initialize_error = exc
                    finally:
                        self.initialize_done.set()
                    continue
                deadline = time.monotonic()
                while self.active:
                    deadline += 1.0 / self.frequency
                    time.sleep(max(0.0, deadline - time.monotonic()))
                    if not self.active:
                        break
                    try:
                        observation, reward, done, info = self.environment.step(self._action())
                        self.policy_ticks += 1
                        self.backend_ticks += 1
                        self._publish(observation, self.policy_ticks)
                        check_success = getattr(self.environment, "check_success", None)
                        if check_success is None:
                            check_success = self.environment._check_success
                        success = bool(
                            info.get("success") or info.get("is_success") or check_success()
                        )
                        reason = None
                        if success:
                            reason = "success"
                        elif self.policy_ticks >= self.horizon:
                            reason = "horizon"
                        elif done:
                            reason = "backend_done"
                        if reason is not None:
                            with self.condition:
                                self.result = self._terminal(success, float(reward), reason)
                                self.active = False
                                self.condition.notify_all()
                            break
                    except BaseException as exc:
                        with self.condition:
                            self.result = self._terminal(False, 0.0, "failure", str(exc))
                            self.active = False
                            self.condition.notify_all()
                        break
        finally:
            if self.environment is not None:
                self.environment.close()

    def _action(self) -> np.ndarray:
        observation = self.snapshot
        measured = np.asarray(observation.joint_position, dtype=np.float64)
        target = np.asarray(self.target, dtype=np.float64)
        arm = np.clip((target[:7] - measured) / 0.05, -1.0, 1.0)
        gripper_error = target[7] - observation.gripper_position
        if abs(gripper_error) <= GRIPPER_APERTURE_TOLERANCE_M:
            gripper = 0.0
        else:
            gripper = -1.0 if gripper_error > 0.0 else 1.0
        return np.concatenate([arm, [gripper]])

    def _publish(self, observation, tick: int) -> None:
        joints = np.asarray(observation["robot0_joint_pos"], dtype=np.float64)[:7]
        velocities = np.asarray(observation["robot0_joint_vel"], dtype=np.float64)[:7]
        gripper = float(np.mean(np.abs(observation["robot0_gripper_qpos"])))
        images = []
        for camera in ("agentview", "robot0_eye_in_hand"):
            rgb = np.ascontiguousarray(np.flipud(observation[f"{camera}_image"]), dtype=np.uint8)
            images.append(pb2.ImageFrame(camera=camera, width=128, height=128, rgb=rgb.tobytes()))
        with self.condition:
            self.snapshot = pb2.RobotSnapshot(
                tick=tick,
                timestamp_s=time.time(),
                joint_position=joints,
                joint_velocity=velocities,
                gripper_position=gripper,
                images=images,
            )
            self.snapshot_version += 1
            self.condition.notify_all()

    def _measured_target(self, observation) -> np.ndarray:
        return np.array(
            [
                *np.asarray(observation["robot0_joint_pos"], dtype=np.float64)[:7],
                float(np.mean(np.abs(observation["robot0_gripper_qpos"]))),
            ]
        )

    def _terminal(self, success, reward, reason, error=""):
        return pb2.TerminalResult(
            success=success,
            score=1.0 if success else 0.0,
            reward=reward,
            terminal_reason=reason,
            policy_ticks=self.policy_ticks,
            backend_ticks=self.backend_ticks,
            error=error,
        )


class PolicyService:
    def __init__(self, runtime: Runtime) -> None:
        self.runtime = runtime

    def GetHealth(self, _request, _context):
        return pb2.Health(ready=True, detail="policy interface ready")

    def WatchState(self, _request, context):
        version = -1
        while context.is_active():
            with self.runtime.condition:
                while self.runtime.snapshot_version == version and context.is_active():
                    self.runtime.condition.wait(timeout=1)
                if not context.is_active():
                    return
                version = self.runtime.snapshot_version
                snapshot = self.runtime.snapshot
            if snapshot is not None:
                yield snapshot

    def SetJointTargets(self, request, _context):
        self.runtime.update_target(request)
        return pb2.Ack(sequence=request.sequence)


class ControlService:
    def __init__(self, runtime: Runtime, token: str) -> None:
        self.runtime = runtime
        self.token = token

    def _authorize(self, context) -> None:
        metadata = dict(context.invocation_metadata())
        if metadata.get("authorization") != f"Bearer {self.token}":
            context.abort(grpc.StatusCode.PERMISSION_DENIED, "invalid control capability")

    def GetHealth(self, _request, context):
        self._authorize(context)
        return pb2.Health(ready=True, detail="evaluation control ready")

    def InitializeTrial(self, request, context):
        self._authorize(context)
        name, instruction = self.runtime.initialize(request)
        return pb2.TrialReady(task_name=name, instruction=instruction)

    def StartTrial(self, _request, context):
        self._authorize(context)
        self.runtime.start()
        return pb2.Empty()

    def WaitForTerminal(self, _request, context):
        self._authorize(context)
        return self.runtime.wait_result()

    def CancelTrial(self, _request, context):
        self._authorize(context)
        return self.runtime.cancel()

    def GetNativeResult(self, _request, context):
        self._authorize(context)
        return self.runtime.wait_result()


def main() -> None:
    runtime = Runtime()
    policy_server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
    control_server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    pb2_grpc.add_PolicyInterfaceServicer_to_server(PolicyService(runtime), policy_server)
    pb2_grpc.add_EvaluationControlServicer_to_server(
        ControlService(runtime, os.environ["DIMOS_LIBERO_CONTROL_TOKEN"]), control_server
    )
    policy_server.add_insecure_port("0.0.0.0:50051")
    control_server.add_insecure_port("0.0.0.0:50052")
    policy_server.start()
    control_server.start()
    stopped = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    stopped.wait()
    policy_server.stop(2)
    control_server.stop(2)
    runtime.stop()


if __name__ == "__main__":
    main()
