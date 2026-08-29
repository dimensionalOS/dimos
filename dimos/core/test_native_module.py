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

import contextlib
from io import BytesIO
import json
from pathlib import Path
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

from pydantic import ValidationError
import pytest

from dimos.core import native_module as native_module_mod
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, TransportBackend
from dimos.core.module import Module, StreamGroup
from dimos.core.native_module import LogFormat, NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.core.transport_factory import make_transport, transport_topic
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_NEVER_DROP, Topic as ZenohTopic
from dimos.protocol.service.zenohservice import ZenohConfig

_ECHO = str(Path(__file__).parent / "demos" / "native_echo.py")


@pytest.fixture
def args_file(tmp_path: Path) -> str:
    """Temp file path where native_echo.py writes the CLI args it received."""
    return str(tmp_path / "native_echo_args.json")


def parse_cli_args(raw: list[str]) -> dict[str, str]:
    """Parse --key value pairs out of a native module arg list."""
    result = {}
    i = 0
    while i < len(raw):
        if raw[i].startswith("--") and i + 1 < len(raw):
            result[raw[i][2:]] = raw[i + 1]
            i += 2
        else:
            i += 1
    return result


def read_json_file(path: str) -> dict[str, str]:
    """Read and parse --key value pairs from the echo output file."""
    return parse_cli_args(json.loads(Path(path).read_text()))


class StubNativeConfig(NativeModuleConfig):
    executable: str = _ECHO
    output_file: str | None = None
    die_after: float | None = None
    some_param: float = 1.5


class StubFrameIdConfig(NativeModuleConfig):
    executable: str = _ECHO
    stdin_config: bool = True
    base_fields: frozenset[str] = frozenset({"frame_id"})
    some_param: float = 1.5


class StubNativeModule(NativeModule):
    config: StubNativeConfig
    pointcloud: Out[PointCloud2]
    imu: Out[Imu]
    cmd_vel: In[Twist]


class StubBuildModule(NativeModule):
    pass


class StubIoModule(NativeModule):
    config: StubNativeConfig
    cmd_vel: In[Twist]
    tf: IO[TFMessage]


class StubConsumer(Module):
    pointcloud: In[PointCloud2]
    imu: In[Imu]

    @rpc
    def start(self) -> None:
        super().start()


class StubProducer(Module):
    cmd_vel: Out[Twist]

    @rpc
    def start(self) -> None:
        super().start()


_WATCHDOG_POLL_INTERVAL = 0.1
_WATCHDOG_MAX_POLLS = 30
_THREAD_DRAIN_DELAY = 0.5


def test_process_crash_triggers_stop() -> None:
    """When the native process dies unexpectedly, the watchdog calls stop()."""
    module = StubNativeModule(die_after=0.2)
    transport = LCMTransport("/pc", PointCloud2)
    module.pointcloud.transport = transport
    try:
        module.start()

        assert module._process is not None
        pid = module._process.pid

        # Wait for the process to die and the watchdog to call stop()
        for _ in range(_WATCHDOG_MAX_POLLS):
            time.sleep(_WATCHDOG_POLL_INTERVAL)
            if module._process is None:
                break

        assert module._process is None, f"Watchdog did not clean up after process {pid} died"

        # Wait for background threads (run_forever, _lcm_loop, _watch_process) to finish
        # after the watchdog-triggered stop(). Without this, monitor_threads catches them.
        time.sleep(_THREAD_DRAIN_DELAY)
    finally:
        module.stop()
        try:
            transport.stop()
        except Exception:
            pass


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


def test_io_port_topic_reaches_the_native_process() -> None:
    """An IO port is both a subscriber and a publisher, so it needs its topic."""
    module = StubIoModule(executable=_ECHO)
    transports = [LCMTransport("/cmd_vel", Twist), LCMTransport("/tf", TFMessage)]
    try:
        module.set_transport("cmd_vel", transports[0])
        module.set_transport("tf", transports[1])

        assert module._collect_topics() == {
            "cmd_vel": "/cmd_vel#geometry_msgs.Twist",
            "tf": "/tf#tf2_msgs.TFMessage",
        }
    finally:
        module.stop()
        for transport in transports:
            with contextlib.suppress(Exception):
                transport.stop()


def test_tf_topic_comes_from_the_declared_port_only() -> None:
    """No tf port declared means no tf topic, rather than a silently injected one."""
    module = StubNativeModule(executable=_ECHO)
    transport = LCMTransport("/cmd_vel", Twist)
    try:
        module.set_transport("cmd_vel", transport)

        assert module._collect_topics() == {"cmd_vel": "/cmd_vel#geometry_msgs.Twist"}
    finally:
        module.stop()
        with contextlib.suppress(Exception):
            transport.stop()


def test_a_stream_group_reaches_the_native_process_as_a_list(monkeypatch) -> None:
    """A group's channels are resolved without the module declaring a stream each."""
    monkeypatch.setattr(native_module_mod.global_config, "transport", "lcm")
    module = StubNativeModule(
        executable=_ECHO,
        stream_groups={"cams": StreamGroup(names=["cam0/imu", "cam1/imu"], msg_type=Imu)},
    )
    try:
        topics = module._collect_topics()
        assert topics["cams"] == ["/cam0/imu#sensor_msgs.Imu", "/cam1/imu#sensor_msgs.Imu"]
        assert module._argv(topics)[1:3] == [
            "--cams",
            "/cam0/imu#sensor_msgs.Imu,/cam1/imu#sensor_msgs.Imu",
        ]
    finally:
        module.stop()


def test_a_stream_group_rejects_a_backend_topic_string() -> None:
    """Names are stream names, so a leading slash is a transport leaking in."""
    with pytest.raises(ValidationError, match="not topics"):
        StreamGroup(names=["/cam0/imu"], msg_type=Imu)


def test_a_stream_group_cannot_shadow_a_declared_port() -> None:
    module = StubNativeModule(
        executable=_ECHO,
        stream_groups={"cmd_vel": StreamGroup(names=["other"], msg_type=Twist)},
    )
    transport = LCMTransport("/cmd_vel", Twist)
    try:
        module.set_transport("cmd_vel", transport)
        with pytest.raises(ValueError, match="collides"):
            module._collect_topics()
    finally:
        module.stop()
        with contextlib.suppress(Exception):
            transport.stop()


def test_a_stream_group_is_not_a_native_config_field() -> None:
    """The group is wiring, so it belongs in `topics`, not the config struct."""
    module = StubNativeModule(
        executable=_ECHO,
        stream_groups={"cams": StreamGroup(names=["cam0/imu"], msg_type=Imu)},
    )
    try:
        assert "stream_groups" not in module.config.to_config_dict()
        assert "--stream_groups" not in module._argv({})
    finally:
        module.stop()


def test_io_port_publisher_qos_reaches_the_native_process() -> None:
    module = StubIoModule(executable=_ECHO)
    transport = ZenohTransport(ZenohTopic("/tf", TFMessage, qos=QOS_NEVER_DROP))
    try:
        module.set_transport("tf", transport)

        assert module._collect_output_qos() == {
            transport.channel: {"reliability": "reliable", "congestion_control": "block"},
        }
    finally:
        module.stop()
        with contextlib.suppress(Exception):
            transport.stop()


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
            ("pointcloud", PointCloud2): make_transport("/my/custom/lidar", PointCloud2),
        },
    )

    coordinator = ModuleCoordinator.build(blueprint.global_config(viewer="none"))
    try:
        # Validate blueprint wiring: all modules deployed
        native = coordinator.get_instance(StubNativeModule)
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
        assert native.pointcloud.transport.topic.topic == transport_topic("/my/custom/lidar")

        # Wait for the native subprocess to write the output file
        for _ in range(50):
            if Path(args_file).exists():
                break
            time.sleep(_WATCHDOG_POLL_INTERVAL)
    finally:
        coordinator.stop()

    # A native module is handed each stream's wire channel, which the two
    # backends spell differently -- ask the factory rather than pinning one.
    assert read_json_file(args_file) == {
        "cmd_vel": make_transport("/cmd_vel", Twist).channel,
        "pointcloud": make_transport("/my/custom/lidar", PointCloud2).channel,
        "imu": make_transport("/imu", Imu).channel,
        "output_file": args_file,
        "some_param": "2.5",
    }


def run_build(tmp_path: Path, *, build_native: bool) -> Path:
    """Builds a module whose executable already exists. Returns the build sentinel path."""
    sentinel = tmp_path / "build_ran"
    exe = tmp_path / "already_built"
    exe.touch()
    module = StubBuildModule(
        executable=str(exe),
        build_command=f"touch {sentinel}",
        g=GlobalConfig(build_native=build_native),
    )
    try:
        module.build()
    finally:
        module.stop()
    return sentinel


def test_existing_executable_skips_build(tmp_path: Path) -> None:
    assert not run_build(tmp_path, build_native=False).exists()


def test_build_native_forces_build(tmp_path: Path) -> None:
    assert run_build(tmp_path, build_native=True).exists()


def _launch(monkeypatch, transport: TransportBackend, **config_kwargs: Any) -> dict[str, Any]:
    """The launch line the native subprocess would get, without spawning it."""
    monkeypatch.setattr(native_module_mod.global_config, "transport", transport)
    monkeypatch.setattr(native_module_mod.global_config, "robot_ip", "192.0.2.10")
    monkeypatch.setattr(native_module_mod.global_config, "robot_ips", None)
    monkeypatch.setattr(native_module_mod.global_config, "zenoh_interface", "")
    monkeypatch.setattr(native_module_mod.global_config, "zenoh_scouting", False)
    monkeypatch.setattr(native_module_mod.global_config, "zenoh_mode", "peer")
    monkeypatch.setattr(native_module_mod.global_config, "zenoh_connect", "")
    # A port-less module: constructing one with ports opens its transports.
    module = StubBuildModule(executable=_ECHO, stdin_config=True, **config_kwargs)
    try:
        return json.loads(module._stdin_blob({}))
    finally:
        module.stop()


def test_the_launch_line_carries_the_session(monkeypatch) -> None:
    session = _launch(monkeypatch, "zenoh")["session"]
    assert session["mode"] == "peer"
    assert session["connect"] == ["tcp/192.0.2.10:7447"]


def test_lcm_sends_no_session_settings(monkeypatch) -> None:
    assert _launch(monkeypatch, "lcm")["session"] == {}


def test_a_pinned_mode_reaches_the_launch_line(monkeypatch) -> None:
    """A blueprint can give one native a different role, keeping the rest derived."""
    session = _launch(monkeypatch, "zenoh", session=ZenohConfig(mode="client"))["session"]
    assert session["mode"] == "client"
    assert session["connect"] == ["tcp/192.0.2.10:7447"]


def test_a_native_can_be_opened_as_the_router(monkeypatch) -> None:
    pinned = ZenohConfig(mode="router", listen=["tcp/127.0.0.1:17450"], connect=[])
    session = _launch(monkeypatch, "zenoh", session=pinned)["session"]
    assert session["mode"] == "router"
    assert session["listen"] == ["tcp/127.0.0.1:17450"]
    assert session["connect"] == []


def test_a_pinned_session_is_not_a_module_config_field(monkeypatch) -> None:
    """Native config structs reject unknown keys, so it stays out of config."""
    assert _launch(monkeypatch, "zenoh", session=ZenohConfig())["config"] is None


def test_a_session_for_another_transport_is_rejected(monkeypatch) -> None:
    """A module pinned as a zenoh router must not start silently under LCM."""
    with pytest.raises(ValueError, match="but the transport is lcm"):
        _launch(monkeypatch, "lcm", session=ZenohConfig(mode="client"))


def test_a_session_without_the_stdin_line_is_rejected() -> None:
    """The session reaches the module on that line or not at all."""
    with pytest.raises(ValidationError, match="stdin_config off"):
        NativeModuleConfig(executable=_ECHO, session=ZenohConfig(), stdin_config=False)


def test_base_field_not_sent_without_opt_in() -> None:
    """Native config structs reject unknown keys, so a base field stays Python-side."""
    config = StubNativeConfig(frame_id="odom")
    assert "frame_id" not in config.to_config_dict()
    assert "frame_id" not in parse_cli_args(config.to_cli_args())


def test_base_field_sent_when_opted_in() -> None:
    config = StubFrameIdConfig(frame_id="odom")
    assert config.to_config_dict()["frame_id"] == "odom"
    assert parse_cli_args(config.to_cli_args())["frame_id"] == "odom"


def test_framework_fields_never_sent() -> None:
    """Opting one base field in must not carry the subprocess plumbing with it."""
    config = StubFrameIdConfig(frame_id="odom", cwd="/tmp", extra_args=["--x"])
    plumbing = set(NativeModuleConfig.model_fields) - {"frame_id"}
    assert not plumbing & set(config.to_config_dict())
    assert not plumbing & set(parse_cli_args(config.to_cli_args()))


def _capture_logs(
    log_format: LogFormat,
    payload: bytes,
    default_level: str = "info",
) -> list[tuple[str, str, dict]]:
    calls: list[tuple[str, str, dict]] = []

    class FakeLogger:
        def __getattr__(self, name: str):
            def _record(message: str, **kwargs: object) -> None:
                calls.append((name, message, kwargs))

            return _record

    fixture = SimpleNamespace(
        config=SimpleNamespace(log_format=log_format),
        _module_label="test",
    )
    with patch.object(native_module_mod, "logger", FakeLogger()):
        NativeModule._read_log_stream(
            fixture,  # type: ignore[arg-type]
            BytesIO(payload),
            default_level,
            pid=123,
        )
    return calls


def test_text_mode_uses_stream_default_level() -> None:
    calls = _capture_logs(LogFormat.TEXT, b"hello\n", "info")
    assert calls == [("info", "hello", {"module": "test", "pid": 123})]


def test_empty_lines_skipped() -> None:
    calls = _capture_logs(LogFormat.TEXT, b"\n\nhello\n\n", "info")
    assert len(calls) == 1
    assert calls[0][1] == "hello"


def test_json_mode_honors_level_field() -> None:
    calls = _capture_logs(
        LogFormat.JSON,
        b'{"level": "error", "message": "boom"}\n',
        "info",
    )
    assert len(calls) == 1
    assert calls[0][0] == "error"
    assert calls[0][1] == "boom"


def test_json_mode_level_alias_is_case_insensitive() -> None:
    calls = _capture_logs(
        LogFormat.JSON,
        b'{"level": "WARN", "message": "watch out"}\n',
        "info",
    )
    assert calls[0][0] == "warning"


def test_json_mode_reads_tracing_subscriber_fields_message() -> None:
    calls = _capture_logs(
        LogFormat.JSON,
        b'{"level": "INFO", "fields": {"message": "started", "device": "/dev/foo"}}\n',
        "info",
    )
    assert len(calls) == 1
    method, message, kwargs = calls[0]
    assert method == "info"
    assert message == "started"
    assert kwargs["device"] == "/dev/foo"


def test_json_mode_unrecognized_level_falls_back_to_stream_default() -> None:
    calls = _capture_logs(
        LogFormat.JSON,
        b'{"level": "weird", "message": "hi"}\n',
        "warning",
    )
    assert calls[0][0] == "warning"


def test_json_mode_missing_level_falls_back_to_stream_default() -> None:
    calls = _capture_logs(
        LogFormat.JSON,
        b'{"message": "no level here"}\n',
        "warning",
    )
    assert calls[0][0] == "warning"
    assert calls[0][1] == "no level here"


def test_json_mode_malformed_falls_back_to_plain_text() -> None:
    calls = _capture_logs(
        LogFormat.JSON,
        b"not json at all\n",
        "info",
    )
    assert calls[0][0] == "info"
    assert calls[0][1] == "not json at all"
