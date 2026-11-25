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

"""Browser -> actual gateway -> shared control -> fake hardware, plus actual H264.

Run explicitly with the candidate gateway and Desktop roots. The fixture only
uses isolated loopback transports. Physical actuation and XR optics are excluded.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from functools import partial
import json
import math
import os
from pathlib import Path
import selectors
import socket
import subprocess
import threading
import time
from typing import Any

import numpy as np
from playwright.sync_api import sync_playwright
import psutil

from dimos.control._control_test_helpers import FakeArm
from dimos.control.connection_source import ConnectionSource
from dimos.control.task import CoordinatorState, JointCommandOutput
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
)
from dimos.control.tick_loop import TickLoop
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.core.transport import ZenohTransport
from dimos.diagnostics.analysis import analyze, load_bundle
from dimos.diagnostics.evaluation import matched_paths
from dimos.diagnostics.model import current_context
from dimos.diagnostics.report import write_report
from dimos.diagnostics.runtime import configure, get_runtime, publication
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig, RustRecorder
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.pubsub.impl.zenohpubsub import Topic, ZenohPubSubBase
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool


def child(
    stack: ExitStack, command: list[str], log: Path, *, env: dict[str, str] | None = None
) -> subprocess.Popen[str]:
    errors = stack.enter_context(log.open("w"))
    process = subprocess.Popen(
        command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=errors, text=True, env=env
    )

    def close() -> None:
        if process.poll() is not None:
            return
        try:
            process.communicate("stop\n", timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.communicate(timeout=8)

    stack.callback(close)
    return process


def announcement(process: subprocess.Popen[str]) -> dict[str, Any]:
    assert process.stdout
    with selectors.DefaultSelector() as poll:
        poll.register(process.stdout, selectors.EVENT_READ)
        if not poll.select(30):
            raise TimeoutError("fixture startup")
        value = json.loads(process.stdout.readline())
        if not isinstance(value, dict):
            raise ValueError("invalid fixture announcement")
        return value


class MeasuredTask(JointTrajectoryTask):
    """Fixture sidecar records the input actually selected, also with tracing off."""

    sampled_ms: float | None = None
    selected_ms: float | None = None
    delay_ms: float = 0

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if self.delay_ms:
            time.sleep(self.delay_ms / 1000)
        output = super().compute(state)
        self.selected_ms = self.sampled_ms if output is not None else None
        return output


class _RecordingLoad(RustRecorder):
    """Use the existing recorder implementation, with three synthetic cameras."""

    head: In[Image]
    left: In[Image]
    right: In[Image]


def cpu_snapshot() -> dict[int, float]:
    parent = psutil.Process()
    result = {}
    for process in [parent, *parent.children(recursive=True)]:
        try:
            times = process.cpu_times()
            result[process.pid] = times.user + times.system
        except psutil.NoSuchProcess:
            pass
    return result


def run(
    gateway: Path,
    desktop: Path,
    directory: Path,
    chrome: str | None,
    mode: str = "trace",
    steady_seconds: float = 0,
    extra_viewers: int = 0,
    decode_delay_ms: int = 0,
    control_delay_ms: float = 0,
    recorder_binary: Path | None = None,
) -> dict[str, Any]:
    directory.mkdir(parents=True, exist_ok=False)
    run_id = "browser-qualification"
    configure(mode, run_id, str(directory))
    runtime = get_runtime()
    with ExitStack() as cleanup:
        if runtime:
            cleanup.callback(runtime.close)
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
        binary = gateway / "gateway/target/debug/examples/diagnostic_browser"
        server = child(
            cleanup,
            [str(binary), endpoint],
            directory / "gateway.log",
            env={
                **os.environ,
                "DIMOS_DIAGNOSTICS": mode,
                "DIMOS_DIAGNOSTICS_RUN": run_id,
                "DIMOS_DIAGNOSTICS_DIR": str(directory),
            },
        )
        gateway_url = announcement(server)["url"]
        client = directory / "client.js"
        subprocess.run(
            [
                "deno",
                "bundle",
                "--no-config",
                "--platform",
                "browser",
                str(gateway / "client/zenoh_gateway.ts"),
                "-o",
                str(client),
            ],
            check=True,
            capture_output=True,
        )
        web = child(
            cleanup,
            [
                "deno",
                "run",
                "--allow-read",
                "--allow-write",
                "--allow-net",
                str(desktop / "tests/diagnostics-browser-server.ts"),
                str(desktop),
                str(client),
                str(directory),
            ],
            directory / "web.log",
        )
        web_url = announcement(web)["url"]

        pool = ZenohSessionPool()
        cleanup.callback(pool.close_all)
        settings: dict[str, Any] = dict(
            session_pool=pool,
            mode="client",
            connect=[endpoint],
            listen=[],
            scouting=False,
            multicast=False,
            gossip=False,
        )
        pubsub = ZenohPubSubBase(**settings)
        pubsub.start()
        cleanup.callback(pubsub.stop)
        recorder = None
        if recorder_binary is not None:
            if not recorder_binary.is_file():
                raise FileNotFoundError(
                    "recording-load qualification requires a prebuilt recorder binary"
                )
            previous_transport = global_config.transport
            global_config.transport = "zenoh"
            cleanup.callback(setattr, global_config, "transport", previous_transport)
            recorder = _RecordingLoad(
                source_dir=None,
                build_command=None,
                executable=str(recorder_binary),
                store=RustMcapStoreConfig(path=str(directory / "recording.mcap")),
                record_tf=False,
                encoding_threads=2,
                session=ZenohConfig(
                    mode="client", connect=[endpoint], listen=[], multicast=False, gossip=False
                ),
            )
            for name in ("head", "left", "right"):
                getattr(recorder, name).transport = ZenohTransport(
                    f"diagnostic-browser/record-{name}", Image, **settings
                )
            cleanup.callback(recorder.stop)
            recorder.start()
        arms = {}
        sources = {}
        tasks = {}
        routing = {}
        task_lock = threading.Lock()
        for side in ("left", "right"):
            arm = FakeArm(source=side)
            arms[side] = arm
            arm.position_command.transport = ZenohTransport(
                f"diagnostic-browser/unused-{side}", JointState, **settings
            )
            arm.start()
            cleanup.callback(arm.stop)
            source = ConnectionSource(arm.config, ["position_command"], ["joint_state"], 1)
            sources[side] = source
            cleanup.callback(arm.joint_state.subscribe(partial(source.on_message, "joint_state")))
            joints = tuple(f"{side}/joint{i}" for i in range(1, 8))
            tasks[side] = MeasuredTask(JointTrajectoryTaskConfig(joint_names=joints))
            tasks[side].delay_ms = control_delay_ms if side == "left" else 0
            routing.update(dict.fromkeys(joints, side))
        age_bounds: list[tuple[float, float]] = []
        clock_offset: tuple[float, float] | None = None
        measuring = False

        def dispatch(name: str, values: dict[str, float]) -> None:
            sampled = tasks[name].selected_ms
            if measuring and sampled is not None and clock_offset is not None:
                now_ms = time.perf_counter_ns() / 1e6
                age_bounds.append(
                    (now_ms - sampled - clock_offset[1], now_ms - sampled - clock_offset[0])
                )
            arms[name].command(values)

        loop = TickLoop(
            100,
            {},
            threading.Lock(),
            {name: task for name, task in tasks.items()},
            task_lock,
            routing,
            connections=sources,
            connection_command_callback=dispatch,
        )
        loop.start()
        cleanup.callback(loop.stop)
        delivered = {side: threading.Event() for side in arms}

        def command(payload: bytes, topic: Topic) -> None:
            if runtime:
                assert current_context() is not None, "gateway dropped command identity"
            side = topic.pattern.rsplit("-", 1)[-1]
            data = json.loads(payload)
            with task_lock:
                tasks[side].sampled_ms = float(data["sampledMs"])
                tasks[side].on_joint_command(
                    JointState(name=[f"{side}/joint1"], position=[float(data["position"])]),
                    time.perf_counter(),
                )
            delivered[side].set()

        for side in arms:
            cleanup.callback(pubsub.subscribe(Topic(f"diagnostic-browser/control-{side}"), command))
        camera_keys = [f"diagnostic-browser/camera-{name}" for name in ("head", "left", "right")]
        stop = threading.Event()

        def camera() -> None:
            sequence = 0
            while not stop.wait(1 / 30):
                sequence += 1
                for key in camera_keys:
                    with publication(key):
                        if runtime:
                            runtime.event("camera.acquired", channel=key)
                        pubsub.publish(
                            Topic(key),
                            bytes(
                                [sequence % 256, decode_delay_ms if key.endswith("-head") else 0]
                            ),
                        )
                        if recorder is not None:
                            name = key.rsplit("-", 1)[-1]
                            image = Image.from_numpy(
                                np.full((240, 320, 3), sequence % 256, dtype=np.uint8),
                                frame_id=name,
                            )
                            pubsub.publish(
                                Topic(f"diagnostic-browser/record-{name}", Image),
                                image.lcm_encode(),
                            )

        producer = threading.Thread(target=camera)
        producer.start()

        def stop_camera() -> None:
            stop.set()
            producer.join(5)

        cleanup.callback(stop_camera)

        playwright = cleanup.enter_context(sync_playwright())
        browser = playwright.chromium.launch(
            headless=True,
            executable_path=chrome,
            args=[
                "--autoplay-policy=no-user-gesture-required",
                "--use-gl=angle",
                "--use-angle=swiftshader",
                "--enable-unsafe-swiftshader",
            ],
        )
        cleanup.callback(browser.close)
        page = browser.new_page()
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.goto(web_url + "/tests/diagnostics-browser.html")
        page.wait_for_function('typeof startFixture === "function"')
        page.evaluate(
            "config => startFixture(config)",
            {"gatewayUrl": gateway_url, "run": run_id, "mode": mode},
        )
        page.wait_for_function(
            "[...fixture.feeds.values()].every(feed => feed.video.readyState >= 2)"
        )
        for _ in range(extra_viewers):
            viewer = browser.new_page()
            viewer.on("pageerror", lambda error: errors.append(str(error)))
            viewer.goto(web_url + "/tests/diagnostics-browser.html")
            viewer.evaluate(
                "config => startFixture(config)",
                {"gatewayUrl": gateway_url, "run": run_id, "mode": mode, "viewOnly": True},
            )
            viewer.wait_for_function(
                "[...fixture.feeds.values()].every(feed => feed.video.readyState >= 2)"
            )
            cleanup.callback(viewer.evaluate, "fixture.close()")
        page.evaluate("fixture.command(0.25)")
        assert all(signal.wait(5) for signal in delivered.values()), (
            "both commands must reach DimOS"
        )
        page.wait_for_function(
            "[...fixture.diagnostics.watches.values()].every(w => w.shown >= 10)", timeout=20000
        )
        if runtime:
            page.wait_for_function("fixture.diagnostics.traceUpload.bytesSent > 0")
        if steady_seconds:
            # Four timestamps bound the loopback browser/Python clock relation;
            # no wall-clock subtraction or RTT/2 assumption. Reprobe after load.
            def probe_clock() -> tuple[float, float]:
                candidates = []
                for _ in range(5):
                    t0 = time.perf_counter_ns() / 1e6
                    t1, t2 = page.evaluate(
                        "() => {const t1=performance.now(); return [t1,performance.now()]}"
                    )
                    t3 = time.perf_counter_ns() / 1e6
                    candidates.append((t0 - t1, t3 - t2))
                return min(candidates, key=lambda bounds: bounds[1] - bounds[0])

            clock_offset = probe_clock()
            before_cpu = cpu_snapshot()
            before_bytes = page.evaluate("fixture.diagnostics.traceUpload.bytesSent")
            started = time.perf_counter()
            measuring = True
            page.evaluate("fixture.startInputs()")
            page.wait_for_function(
                "deadline => performance.now() >= deadline",
                arg=page.evaluate("performance.now()") + steady_seconds * 1000,
                timeout=(steady_seconds + 5) * 1000,
            )
            measuring = False
            elapsed = time.perf_counter() - started
            after_cpu = cpu_snapshot()
            page.evaluate("fixture.stopInputs()")
            ending_offset = probe_clock()
            # Include the observed endpoint drift rather than silently assuming
            # clocks stayed aligned throughout the measurement window.
            drift = max(abs(ending_offset[i] - clock_offset[i]) for i in (0, 1))
            ages = sorted(age_bounds)
            assert ages, "no adapter dispatch samples in overhead window"
            percentile_index = math.ceil(len(ages) * 0.99) - 1
            bound99 = [sorted(pair[i] for pair in ages)[percentile_index] for i in (0, 1)]
            cpu = sum(
                max(0, value - before_cpu.get(pid, value)) for pid, value in after_cpu.items()
            )
            overhead = {
                "mode": mode,
                "elapsed_s": elapsed,
                "host_cpu_percent": cpu / elapsed / (os.cpu_count() or 1) * 100,
                "process_cpu_s": cpu,
                "cpu_process_count": len(after_cpu),
                "dispatch_samples": len(ages),
                "dispatch_p99_bounds_ms": [bound99[0] - drift, bound99[1] + drift],
                "clock_offset_uncertainty_ms": clock_offset[1] - clock_offset[0],
                "clock_endpoint_change_ms": drift,
                "telemetry_bytes_per_s": (
                    page.evaluate("fixture.diagnostics.traceUpload.bytesSent") - before_bytes
                )
                / elapsed,
                "arms": 2,
                "cameras": 3,
                "extra_viewers": extra_viewers,
                "recording": recorder is not None,
                "decode_delay_ms": decode_delay_ms,
                "control_delay_ms": control_delay_ms,
                "scope": "loopback two-arm input at60Hz, shared control100Hz, three H264 cameras30Hz; CPU includes Python, gateway, Deno and Chrome; no XR optics",
            }
            page.evaluate("fixture.close()")
            assert not errors, errors
            (directory / "result.json").write_text(json.dumps(overhead, indent=2))
            return overhead
        # Stop only renderer submissions: real H264 reception/decoding continues.
        before = page.evaluate("[...fixture.diagnostics.watches.values()].map(w => w.shown)")
        page.evaluate("fixture.rendering(false)")
        page.wait_for_function(
            "before => [...fixture.diagnostics.watches.values()].every((w, i) => w.shown >= before[i] + 20)",
            arg=before,
        )
        page.evaluate("fixture.rendering(true)")
        page.wait_for_function(
            "before => [...fixture.diagnostics.watches.values()].every((w, i) => w.shown >= before[i] + 25)",
            arg=before,
        )
        page.evaluate("fixture.close()")
        assert not errors, errors
        assert all(arm.writes for arm in arms.values()), "both adapters must receive dispatches"
        # Flush before examining immutable capture files.
        loop.stop()
        assert runtime
        runtime.close()
        server.communicate("stop\n", timeout=10)

    manifests, events = load_bundle(directory / run_id)
    commands = matched_paths(events, "operator.input", "control.dispatch")
    frames = matched_paths(events, "camera.acquired", "video.render_submit")
    assert any(p["status"] == "bounded" for p in commands), "command path not clock-correlated"
    assert any(p["status"] == "bounded" for p in frames), "camera-to-render path not correlated"
    for side in ("left", "right"):
        assert any(
            p["status"] == "bounded" and p.get("channel") == f"diagnostic-browser/control-{side}"
            for p in commands
        ), f"missing {side} command lineage"
    for key in camera_keys:
        assert any(p["status"] == "bounded" and p.get("channel") == key for p in frames), (
            f"missing {key} render lineage"
        )
    display_age = [
        e["age_ns"]
        for e in events
        if e["event"] == "freshness.age"
        and e.get("watch", "").startswith("video.render_submit/diagnostic-browser/camera-")
    ]
    decode_age = [
        e["age_ns"]
        for e in events
        if e["event"] == "freshness.age"
        and e.get("watch", "").startswith("video/diagnostic-browser/camera-")
    ]
    assert max(display_age, default=0) > 500_000_000, "renderer stall was not detected"
    assert max(decode_age, default=1_000_000_000) < max(display_age), (
        "healthy decoding was blamed for renderer stall"
    )
    write_report(directory / "report.html", manifests, events)
    result = {
        "arms": 2,
        "cameras": len(camera_keys),
        "matched_per_channel": {
            key: sum(
                p["status"] == "bounded" and p.get("channel") == key for p in commands + frames
            )
            for key in [*(f"diagnostic-browser/control-{side}" for side in arms), *camera_keys]
        },
        "commands": len(commands),
        "matched_commands": sum(p["status"] == "bounded" for p in commands),
        "frames": len(frames),
        "matched_frames": sum(p["status"] == "bounded" for p in frames),
        "event_count": len(events),
        "renderer_stall_max_ms": max(display_age) / 1e6,
        "decode_max_ms": max(decode_age) / 1e6,
        "scope": "actual browser SDK, gateway H264, Python control and fake hardware; no physical actuation or XR optics",
    }
    (directory / "result.json").write_text(json.dumps(result, indent=2))
    return result


def verify_workload(directory: Path, result: dict[str, Any]) -> None:
    """Assert injected faults at their owner, with neighboring processing healthy."""
    if result.get("mode") != "trace":
        return
    manifests, events = load_bundle(directory / "browser-qualification")
    groups = analyze(manifests, events)["stage_groups"]
    decoders = {
        g["labels"].get("channel"): g for g in groups if g["stage"] == "gateway.video_decode"
    }
    checks = []
    delay = result.get("decode_delay_ms", 0)
    if delay:
        assert decoders["diagnostic-browser/camera-head"]["p50_ms"] >= delay * 0.9
        assert all(
            decoders[f"diagnostic-browser/camera-{side}"]["p95_ms"] < delay / 2
            for side in ("left", "right")
        )
        checks.append("head decode delay located; wrist decoding healthy")
    delay = result.get("control_delay_ms", 0)
    if delay:
        assert any(
            g["p95_ms"] >= delay * 0.9 for g in groups if g["stage"] == "control.task_compute"
        )
        assert all(g["p95_ms"] < delay / 2 for g in decoders.values())
        checks.append("control processing delay located; camera decoding healthy")
    viewers = result.get("extra_viewers", 0)
    if viewers:
        by_browser: dict[str, set[str]] = {}
        for event in events:
            if event["event"] == "video.render_submit":
                by_browser.setdefault(event["process"], set()).add(event["channel"])
        assert (
            len([channels for channels in by_browser.values() if len(channels) == 3]) == viewers + 1
        )
        checks.append("all viewers retain separate three-camera render evidence")
    if result.get("recording"):
        assert (directory / "recording.mcap").stat().st_size > 1024
        checks.append(
            "existing Rust recorder produced a flushed artifact under camera/control load"
        )
    result["workload_checks"] = checks
    write_report(directory / "report.html", manifests, events)
    (directory / "result.json").write_text(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--gateway", type=Path, required=True)
    parser.add_argument("--desktop", type=Path, required=True)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--chrome")
    parser.add_argument("--mode", choices=["off", "trace"], default="trace")
    parser.add_argument("--steady-seconds", type=float, default=0)
    parser.add_argument("--extra-viewers", type=int, choices=range(4), default=0)
    parser.add_argument("--decode-delay-ms", type=int, choices=range(201), default=0)
    parser.add_argument("--control-delay-ms", type=float, default=0)
    parser.add_argument("--recorder-binary", type=Path)
    args = parser.parse_args()
    result = run(
        args.gateway.resolve(),
        args.desktop.resolve(),
        args.directory.resolve(),
        args.chrome,
        args.mode,
        args.steady_seconds,
        args.extra_viewers,
        args.decode_delay_ms,
        args.control_delay_ms,
        args.recorder_binary,
    )
    verify_workload(args.directory.resolve(), result)
    print(json.dumps(result))
