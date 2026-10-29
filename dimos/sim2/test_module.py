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

"""A world module drives a robot through ordinary ports, with nothing sim-specific on the wire."""

from contextlib import ExitStack
from functools import partial
import threading
import time

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.blueprint import simulation
from dimos.sim2.environment import MotorScene
from dimos.sim2.module import SimulationModule, robot_ports


class _World(SimulationModule):
    left_motor_command: In[MotorCommandArray]
    left_motor_states: Out[JointState]
    left_imu: Out[Imu]
    left_odom: Out[PoseStamped]
    left_tf: Out[TFMessage]


class _Driver(Module):
    """Stands in for the coordinator's transport adapter: standard messages both ways."""

    left_motor_states: In[JointState]
    left_imu: In[Imu]
    left_odom: In[PoseStamped]
    left_motor_command: Out[MotorCommandArray]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._lock = threading.Condition()
        self._joints = None
        self._imu = None
        self._odom = None

    @rpc
    def start(self):
        super().start()
        self.left_motor_states.subscribe(self._on_joints)
        self.left_imu.subscribe(self._on_imu)
        self.left_odom.subscribe(self._on_odom)

    def _on_joints(self, msg):
        with self._lock:
            self._joints = msg
            self._lock.notify_all()

    def _on_imu(self, msg):
        with self._lock:
            self._imu = msg
            self._lock.notify_all()

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg
            self._lock.notify_all()

    @rpc
    def observed(self, minimum_position=None, timeout=15):
        with self._lock:
            ready = self._lock.wait_for(
                lambda: self._joints is not None
                and self._imu is not None
                and self._odom is not None
                and self._joints.ts == self._imu.ts == self._odom.ts
                and (minimum_position is None or self._joints.position[0] > minimum_position),
                timeout=timeout,
            )
            if not ready:
                raise TimeoutError("coherent feedback did not arrive")
            return {
                "names": list(self._joints.name),
                "position": list(self._joints.position),
                "frame": self._joints.frame_id,
                "imu_frame": self._imu.frame_id,
                "odom_frame": self._odom.frame_id,
                "same_tick": self._joints.ts == self._imu.ts == self._odom.ts,
            }

    @rpc
    def wait_for_update(self, timeout):
        with self._lock:
            previous = self._joints
            if not self._lock.wait_for(lambda: self._joints is not previous, timeout=timeout):
                raise TimeoutError("motor feedback did not arrive")

    @rpc
    def command(self, q, kp, kd):
        self.left_motor_command.publish(
            MotorCommandArray(q=q, dq=[0.0] * len(q), kp=kp, kd=kd, tau=[0.0] * len(q))
        )


def test_robot_ports_are_matched_by_name_and_unknown_robots_are_an_error():
    ports = robot_ports(_World, ["left"])
    assert ports == {
        "left": {
            "motor_command": "left_motor_command",
            "motor_states": "left_motor_states",
            "imu": "left_imu",
            "odom": "left_odom",
            "tf": "left_tf",
        }
    }
    assert robot_ports(SimulationModule, ["left"]) == {"left": {}}
    with pytest.raises(ValueError, match="names robot 'left'"):
        robot_ports(_World, ["right"])


def _wait(predicate, driver, timeout=15.0):
    deadline = time.monotonic() + timeout
    while True:
        value = predicate()
        if value:
            return value
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise AssertionError("condition not met in time")
        driver.wait_for_update(remaining)


def test_world_publishes_and_consumes_standard_messages_over_streams(
    world_config, scene_path, robot_factory
):
    sensors = world_config["sensors"][:1]
    world = world_config | {
        "environment": partial(MotorScene, scene_path, {"left": robot_factory}, sensors=sensors),
        "sensors": sensors,
    }
    blueprint = autoconnect(simulation(**world, module=_World), _Driver.blueprint())
    with ExitStack() as stack:
        app = ModuleCoordinator.build(
            blueprint.global_config(n_workers=1, viewer="none", transport="zenoh")
        )
        stack.callback(app.stop)
        sim = app.get_instance(_World)
        driver = app.get_instance(_Driver)

        observed = driver.observed()
        assert observed["names"] == ["shoulder", "gripper"]
        assert observed["position"] == pytest.approx([0.1, 850], abs=1e-3)
        assert observed["frame"] == observed["imu_frame"] == "left/base"
        assert observed["odom_frame"] == "world"
        assert observed["same_tick"]

        driver.command([0.5, 850.0], [10.0, 0.0], [1.0, 0.0])
        assert driver.observed(minimum_position=0.3)["position"][0] > 0.3
        status = sim.status()
        assert status["running"]
        assert status["rejected_commands"] == {}

        driver.command([0.5], [10.0], [1.0])
        _wait(lambda: sim.status()["rejected_commands"].get("left", 0) == 1, driver)
        episode = sim.status()["episode"]
        sim.reset()
        assert sim.status()["episode"] == episode + 1


@pytest.fixture
def owner_module(world_config):
    module = SimulationModule(**world_config)
    with ExitStack() as cleanup:
        cleanup.callback(module.stop)
        module.build()
        yield module


def test_stop_fails_queued_requests_before_waiting_for_the_busy_owner(owner_module, mocker):
    module = owner_module
    runtime = module._require_runtime()
    reset_started = threading.Event()
    release_reset = threading.Event()
    queued = threading.Event()
    pending_finished = threading.Event()
    errors = []
    original_reset = runtime.environment.reset
    original_put = module._requests.put

    def blocked_reset():
        reset_started.set()
        if not release_reset.wait(timeout=5):
            raise TimeoutError("test did not release the active reset")
        return original_reset()

    def record_queue(item):
        original_put(item)
        queued.set()

    def pending_reset():
        try:
            module.reset()
        except Exception as error:
            errors.append(error)
        finally:
            pending_finished.set()

    mocker.patch.object(runtime.environment, "reset", side_effect=blocked_reset)
    mocker.patch.object(module._requests, "put", side_effect=record_queue)
    active = threading.Thread(target=module.reset)
    pending = threading.Thread(target=pending_reset)
    stopping = threading.Thread(target=module.stop)
    active.start()
    try:
        assert reset_started.wait(timeout=5)
        queued.clear()
        pending.start()
        assert queued.wait(timeout=5)
        stopping.start()
        assert module._stop.wait(timeout=5)
        assert pending_finished.wait(timeout=1)
        assert len(errors) == 1
        assert isinstance(errors[0], RuntimeError)
        assert str(errors[0]) == "simulation stopped"
        with pytest.raises(RuntimeError, match="simulation stopped"):
            module.reset()
    finally:
        release_reset.set()
        for thread in (active, pending, stopping):
            if thread.ident is not None:
                thread.join(timeout=5)
    assert not active.is_alive()
    assert not pending.is_alive()
    assert not stopping.is_alive()
    assert module.status() == {"running": False, "error": None}


def test_close_failure_is_reported_and_does_not_leave_a_stale_runtime(owner_module, mocker):
    module = owner_module
    runtime = module._require_runtime()
    owner_ident = module._thread.ident
    closed_on = []
    original_close = runtime.close

    def failing_close():
        closed_on.append(threading.get_ident())
        original_close()
        raise ValueError("renderer teardown failed")

    mocker.patch.object(runtime, "close", side_effect=failing_close)

    module.stop()

    assert closed_on == [owner_ident]
    assert module.status() == {
        "running": False,
        "error": "environment close failed: ValueError: renderer teardown failed",
    }
    with pytest.raises(RuntimeError, match="renderer teardown failed"):
        module.reset()
    module.stop()
    assert closed_on == [owner_ident]


def test_owner_request_failure_reaches_the_caller_and_allows_a_later_reset(owner_module, mocker):
    runtime = owner_module._require_runtime()
    original_reset = runtime.environment.reset
    fail = mocker.patch.object(
        runtime.environment, "reset", side_effect=ValueError("invalid randomized scene")
    )
    with pytest.raises(ValueError, match="invalid randomized scene"):
        owner_module.reset()
    fail.side_effect = original_reset

    episode = owner_module.status()["episode"]
    owner_module.reset()

    assert owner_module.status()["episode"] == episode + 1
    assert owner_module.status()["error"] is None


def test_request_admission_cannot_leave_a_future_behind_after_owner_shutdown(owner_module, mocker):
    module = owner_module
    admission_started = threading.Event()
    release_admission = threading.Event()
    request_finished = threading.Event()
    stop_finished = threading.Event()
    original_put = module._requests.put
    errors = []

    def delayed_put(item):
        admission_started.set()
        if not release_admission.wait(timeout=5):
            raise TimeoutError("test did not release request admission")
        original_put(item)

    def reset():
        try:
            module.reset()
        except Exception as error:
            errors.append(error)
        finally:
            request_finished.set()

    def stop():
        try:
            module.stop()
        finally:
            stop_finished.set()

    mocker.patch.object(module._requests, "put", side_effect=delayed_put)
    requesting = threading.Thread(target=reset)
    stopping = threading.Thread(target=stop)
    requesting.start()
    try:
        assert admission_started.wait(timeout=5)
        stopping.start()
        # With unsynchronized admission the owner can finish while put is blocked.
        # The fixed implementation defers shutdown until admission completes.
        stop_finished.wait(timeout=0.1)
        release_admission.set()
        assert stop_finished.wait(timeout=5)
        assert request_finished.wait(timeout=1)
        assert all(isinstance(error, RuntimeError) for error in errors)
        assert module.status() == {"running": False, "error": None}
    finally:
        release_admission.set()
        module.stop()
        # Also unblock the caller if this regression ever leaves an orphaned Future.
        while not module._requests.empty():
            _, future = module._requests.get_nowait()
            if not future.done():
                future.set_exception(RuntimeError("test cleanup"))
        requesting.join(timeout=5)
        if stopping.ident is not None:
            stopping.join(timeout=5)
    assert not requesting.is_alive()
    assert not stopping.is_alive()
