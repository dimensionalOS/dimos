from collections import defaultdict
import time
from types import SimpleNamespace

import pytest

from dimos.msgs.sensor_msgs.Joy import Joy
import dimos.teleop.joystick as joystick


class LocalTransport:
    def __init__(self):
        self.callbacks = []

    def subscribe(self, callback, selfstream=None):
        self.callbacks.append(callback)
        return lambda: self.callbacks.remove(callback)

    def broadcast(self, selfstream, value):
        for callback in list(self.callbacks):
            callback(value)

    def stop(self):
        pass


@pytest.fixture
def rig():
    module = joystick.JoystickTeleopMapper()
    ports, sent = {}, defaultdict(list)
    for name in (*module.inputs, *module.outputs):
        ports[name] = LocalTransport()
        module.set_transport(name, ports[name])
        if name in module.outputs:
            ports[name].subscribe(sent[name].append)
    module.start()
    yield module, ports, sent
    module.stop()


def emit(ports, hand, axes=(-1.0, -1.0, 0.0, 0.0), pressed=1, **kwargs):
    ports[f"{hand}_joy"].broadcast(None, Joy(axes=list(axes), buttons=[0, 0, 0, pressed], **kwargs))


def test_mapping_clutch_and_compatibility(rig):
    module, ports, sent = rig
    emit(ports, "left")
    assert sent["base_twist"][-1].is_zero()
    emit(ports, "right")
    twist = sent["base_twist"][-1]
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == pytest.approx((0.3, 0.3, 0.5))
    assert sent["lift_rate"][-1].data == pytest.approx(0.03)
    emit(ports, "right", pressed=0)
    assert sent["base_twist"][-1].is_zero() and sent["lift_rate"][-1].data == 0
    module.config.enable_button = None
    emit(ports, "left", axes=(0.05, -0.5))
    assert sent["base_twist"][-1].linear.x == pytest.approx(0.15)
    assert sent["base_twist"][-1].linear.y == 0


def test_invalid_source_age_and_independent_freshness(rig, monkeypatch):
    module, ports, sent = rig
    clock = [100.0]
    monkeypatch.setattr(
        joystick, "time", SimpleNamespace(time=lambda: 1000.0, monotonic=lambda: clock[0])
    )
    module.config.enable_button = None
    emit(ports, "left", ts=1000.0)
    clock[0] += 0.21
    emit(ports, "right", ts=1000.0)
    twist = sent["base_twist"][-1]
    assert twist.linear.x == 0 and twist.angular.z == pytest.approx(0.5)
    for kwargs in (
        {"axes": ()},
        {"axes": (float("nan"), 0)},
        {"axes": (2, 0)},
        {"axes": (0, 0, float("inf"))},
        {"ts": 999.0},
        {"ts": 1001.0},
    ):
        emit(ports, "right", **kwargs)
        assert sent["base_twist"][-1].is_zero() and sent["lift_rate"][-1].data == 0


def test_watchdog_only_stops_and_lifecycle(rig):
    module, ports, sent = rig
    emit(ports, "left")
    emit(ports, "right")
    count = len(sent["base_twist"])
    module._watchdog()
    assert len(sent["base_twist"]) == count  # No cached nonzero replay.
    deadline = time.monotonic() + 1.0
    while not sent["base_twist"][-1].is_zero() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert sent["base_twist"][-1].is_zero() and sent["lift_rate"][-1].data == 0
    emit(ports, "right")
    module.stop()
    assert sent["base_twist"][-1].is_zero() and sent["lift_rate"][-1].data == 0
    count = len(sent["base_twist"])
    time.sleep(0.1)
    assert len(sent["base_twist"]) == count
    assert not ports["left_joy"].callbacks and not ports["right_joy"].callbacks
