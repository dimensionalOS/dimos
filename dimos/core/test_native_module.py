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

"""Tests for NativeModule: blueprint wiring, topic collection, CLI arg generation.

Every test launches the real native_echo.py subprocess via ModuleCoordinator.build(blueprint).
The echo script writes received CLI args to a temp file for assertions.
"""

import json
from pathlib import Path
import time

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_ECHO = str(Path(__file__).parent / "tests" / "native_echo.py")


@pytest.fixture
def args_file(tmp_path: Path) -> str:
    """Temp file path where native_echo.py writes the CLI args it received."""
    return str(tmp_path / "native_echo_args.json")


def read_json_file(path: str) -> dict[str, str]:
    """Read and parse --key value pairs from the echo output file."""
    raw: list[str] = json.loads(Path(path).read_text())
    result = {}
    i = 0
    while i < len(raw):
        if raw[i].startswith("--") and i + 1 < len(raw):
            result[raw[i][2:]] = raw[i + 1]
            i += 2
        else:
            i += 1
    return result


class StubNativeConfig(NativeModuleConfig):
    executable: str = _ECHO
    log_format: LogFormat = LogFormat.TEXT
    output_file: str | None = None
    die_after: float | None = None
    some_param: float = 1.5


class StubNativeModule(NativeModule):
    config: StubNativeConfig
    pointcloud: Out[PointCloud2]
    imu: Out[Imu]
    cmd_vel: In[Twist]


class StubConsumer(Module):
    pointcloud: In[PointCloud2]
    imu: In[Imu]

    @rpc
    def start(self) -> None:
        pass


class StubProducer(Module):
    cmd_vel: Out[Twist]

    @rpc
    def start(self) -> None:
        pass


def test_process_crash_triggers_stop() -> None:
    """When the native process dies unexpectedly, the watchdog calls stop()."""
    mod = StubNativeModule(die_after=0.2)
    mod.pointcloud.transport = LCMTransport("/pc", PointCloud2)
    mod.start()

    assert mod._proc is not None
    assert mod._proc.is_alive
    pid = mod._proc.pid

    # Wait for the process to die and the on_exit callback to call stop()
    for _ in range(30):
        time.sleep(0.1)
        if mod._proc is None or not mod._proc.is_alive:
            break

    assert mod._proc is None or not mod._proc.is_alive, (
        f"Watchdog did not clean up after process {pid} died"
    )

    # stop() is idempotent
    mod.stop()

    # Wait for background threads (run_forever, _lcm_loop, _watch_process) to finish
    # after the watchdog-triggered stop(). Without this, monitor_threads catches them.
    time.sleep(0.5)

    # Ensure all threads (LCM transport, event loop) are cleaned up
    mod.stop()


@pytest.mark.slow
def test_manual(dimos_cluster: ModuleCoordinator, args_file: str) -> None:
    native_module = dimos_cluster.deploy(
        StubNativeModule,
        some_param=2.5,
        output_file=args_file,
    )

    native_module.set_transport("pointcloud", LCMTransport("/my/custom/lidar", PointCloud2))
    native_module.set_transport("cmd_vel", LCMTransport("/cmd_vel", Twist))
    native_module.start()
    time.sleep(1)
    native_module.stop()

    assert read_json_file(args_file) == {
        "cmd_vel": "/cmd_vel#geometry_msgs.Twist",
        "pointcloud": "/my/custom/lidar#sensor_msgs.PointCloud2",
        "output_file": args_file,
        "some_param": "2.5",
    }


@pytest.mark.slow
def test_autoconnect(args_file: str) -> None:
    """autoconnect passes correct topic args to the native subprocess."""
    blueprint = autoconnect(
        StubNativeModule.blueprint(
            some_param=2.5,
            output_file=args_file,
        ),
        StubConsumer.blueprint(),
        StubProducer.blueprint(),
    ).transports(
        {
            ("pointcloud", PointCloud2): LCMTransport("/my/custom/lidar", PointCloud2),
        },
    )

    coordinator = ModuleCoordinator.build(blueprint.global_config(viewer="none"))
    try:
        # Validate blueprint wiring: all modules deployed
        native = coordinator.get_instance(StubNativeModule)  # type: ignore[arg-type]
        consumer = coordinator.get_instance(StubConsumer)
        producer = coordinator.get_instance(StubProducer)
        assert native is not None
        assert consumer is not None
        assert producer is not None

        # Out→In topics match between connected modules
        assert native.pointcloud.transport.topic == consumer.pointcloud.transport.topic
        assert native.imu.transport.topic == consumer.imu.transport.topic
        assert producer.cmd_vel.transport.topic == native.cmd_vel.transport.topic

        # Custom transport was applied
        assert native.pointcloud.transport.topic.topic == "/my/custom/lidar"

        # Wait for the native subprocess to write the output file
        for _ in range(50):
            if Path(args_file).exists():
                break
            time.sleep(0.1)
    finally:
        coordinator.stop()

    assert read_json_file(args_file) == {
        "cmd_vel": "/cmd_vel#geometry_msgs.Twist",
        "pointcloud": "/my/custom/lidar#sensor_msgs.PointCloud2",
        "imu": "/imu#sensor_msgs.Imu",
        "output_file": args_file,
        "some_param": "2.5",
    }
