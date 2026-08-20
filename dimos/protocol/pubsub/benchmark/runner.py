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

"""Separate-process transport benchmark runner."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager
from dataclasses import asdict, replace
import json
import multiprocessing
import os
from pathlib import Path
import queue
import shutil
import signal
import socket
import subprocess
import threading
import time
from typing import Any, cast

import numpy as np
import psutil

from dimos.protocol.pubsub.benchmark.artifacts import (
    ArtifactWriter,
    collect_container_manifest,
    collect_manifest,
    load_trial_summaries,
)
from dimos.protocol.pubsub.benchmark.matrix import build_matrix
from dimos.protocol.pubsub.benchmark.model import (
    Cohort,
    Environment,
    MessageSample,
    NetworkProfile,
    ProcessUsage,
    Stack,
    TrialRecord,
    TrialSpec,
)
from dimos.protocol.pubsub.benchmark.report import generate_report

_READY_TIMEOUT_S = 30.0
_PROCESS_JOIN_TIMEOUT_S = 15.0


def _metadata(topic: str, sequence: int, publish_start_ns: int) -> str:
    return f"{topic}|{sequence}|{publish_start_ns}"


def _parse_metadata(value: str) -> tuple[str, int, int]:
    topic, sequence, publish_start_ns = value.rsplit("|", 2)
    return topic, int(sequence), int(publish_start_ns)


class ImageBenchmarkBus:
    def __init__(self, spec: TrialSpec, role: str) -> None:
        self._spec = spec
        self._bus: Any
        self._topics: dict[str, Any] = {}
        self._unsubscribers: list[Callable[[], None]] = []
        self._arrays = {
            topic.name: np.zeros((1, max(1, topic.payload_bytes)), dtype=np.uint8)
            for topic in spec.workload.topics
        }

        from dimos.msgs.sensor_msgs.Image import Image

        if spec.stack == Stack.LCM:
            from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic as LCMTopic

            lcm_url = (
                "udpm://239.255.76.67:7667?ttl=1"
                if spec.environment == Environment.EMULATED
                else "udpm://239.255.76.67:7667?ttl=0"
            )
            self._bus = LCM(url=lcm_url, subscription_queue_capacity=1)
            self._topics = {
                topic.name: LCMTopic(f"/dimos/benchmark/{topic.name}", Image)
                for topic in spec.workload.topics
            }
        elif spec.stack == Stack.ZENOH:
            from dimos.protocol.pubsub.impl.zenohpubsub import (
                Topic as ZenohTopic,
                Zenoh,
                ZenohQoS,
            )

            listen = [
                value
                for value in os.environ.get("DIMOS_BENCH_ZENOH_LISTEN", "").split(",")
                if value
            ]
            connect = [
                value
                for value in os.environ.get("DIMOS_BENCH_ZENOH_CONNECT", "").split(",")
                if value
            ]
            if not listen and not connect:
                port = 17448 + int(spec.trial_id.removeprefix("t")) % 1000
                endpoint = f"tcp/127.0.0.1:{port}"
                if role == "publisher":
                    listen = [endpoint]
                else:
                    connect = [endpoint]
            self._bus = Zenoh(listen=listen, connect=connect, scouting=not bool(listen or connect))
            self._topics = {
                topic.name: ZenohTopic(
                    f"dimos/benchmark/{topic.name}",
                    Image,
                    qos=ZenohQoS(
                        reliability=(
                            "reliable"
                            if topic.cohort == Cohort.RELIABLE or spec.cohort == Cohort.RELIABLE
                            else "best_effort"
                        ),
                        congestion_control=(
                            "block"
                            if topic.cohort == Cohort.RELIABLE or spec.cohort == Cohort.RELIABLE
                            else "drop"
                        ),
                    ),
                )
                for topic in spec.workload.topics
            }
        else:
            os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
            os.environ.setdefault("ZENOH_CONFIG_OVERRIDE", "transport/shared_memory/enabled=false")
            from dimos.protocol.pubsub.impl.rospubsub import DimosROS, ROSTopic

            try:
                from rclpy.utilities import get_rmw_implementation_identifier
            except ImportError as error:
                msg = "ROS 2 Jazzy with rmw_zenoh_cpp is required for the ROS benchmark stack"
                raise ImportError(msg) from error

            identifier = get_rmw_implementation_identifier()
            if identifier != "rmw_zenoh_cpp":
                raise RuntimeError(f"Expected rmw_zenoh_cpp, got {identifier}")
            self._bus = DimosROS(node_name=f"dimos_benchmark_{os.getpid()}")
            self._topics = {
                topic.name: ROSTopic(
                    f"/dimos/benchmark/{topic.name}",
                    Image,
                    qos=self._ros_qos(
                        topic.cohort == Cohort.RELIABLE or spec.cohort == Cohort.RELIABLE
                    ),
                )
                for topic in spec.workload.topics
            }

    @staticmethod
    def _ros_qos(reliable: bool) -> Any:
        from rclpy.qos import (
            QoSDurabilityPolicy,
            QoSHistoryPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )

        return QoSProfile(
            reliability=(
                QoSReliabilityPolicy.RELIABLE if reliable else QoSReliabilityPolicy.BEST_EFFORT
            ),
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1024 if reliable else 1,
        )

    def start(self) -> None:
        self._bus.start()

    def publish(self, topic: str, sequence: int, publish_start_ns: int) -> None:
        from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

        message = Image(
            data=self._arrays[topic],
            format=ImageFormat.GRAY,
            frame_id=_metadata(topic, sequence, publish_start_ns),
            ts=publish_start_ns / 1e9,
        )
        self._bus.publish(self._topics[topic], message)

    def subscribe(self, topic: str, callback: Callable[[str, int, int], None]) -> None:
        def on_message(message: Any, _topic: Any) -> None:
            callback(*_parse_metadata(message.frame_id))

        self._unsubscribers.append(self._bus.subscribe(self._topics[topic], on_message))

    def stop(self) -> None:
        for unsubscribe in self._unsubscribers:
            unsubscribe()
        self._unsubscribers.clear()
        self._bus.stop()


def _peak_rss_bytes(process: psutil.Process) -> int:
    for line in Path(f"/proc/{process.pid}/status").read_text().splitlines():
        if line.startswith("VmHWM:"):
            return int(line.split()[1]) * 1024
    return int(process.memory_info().rss)


def _process_usage(role: str, started_cpu: float) -> ProcessUsage:
    process = psutil.Process()
    cpu = process.cpu_times()
    switches = process.num_ctx_switches()
    return ProcessUsage(
        role=role,
        cpu_seconds=cpu.user + cpu.system - started_cpu,
        peak_rss_bytes=_peak_rss_bytes(process),
        voluntary_context_switches=switches.voluntary,
        involuntary_context_switches=switches.involuntary,
    )


class _LocalRosRouter(AbstractContextManager["_LocalRosRouter"]):
    def __init__(self) -> None:
        self._launcher: subprocess.Popen[bytes] | None = None
        self._router: psutil.Process | None = None

    @staticmethod
    def _port_is_open() -> bool:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
            connection.settimeout(0.1)
            return connection.connect_ex(("127.0.0.1", 7447)) == 0

    def __enter__(self) -> _LocalRosRouter:
        if shutil.which("ros2") is None:
            raise RuntimeError(
                "ROS 2 Jazzy is not available; run the ROS stack in the benchmark container"
            )
        if self._port_is_open():
            raise RuntimeError("TCP port 7447 is already occupied; cannot own the ROS Zenoh router")
        environment = os.environ.copy()
        environment["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
        environment["ZENOH_CONFIG_OVERRIDE"] = (
            'listen/endpoints=["tcp/127.0.0.1:7447"];transport/shared_memory/enabled=false'
        )
        self._launcher = subprocess.Popen(
            ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        deadline = time.monotonic() + _READY_TIMEOUT_S
        launcher = psutil.Process(self._launcher.pid)
        while time.monotonic() < deadline:
            if self._launcher.poll() is not None:
                raise RuntimeError("rmw_zenohd exited before becoming ready")
            for process in launcher.children(recursive=True):
                if process.name() == "rmw_zenohd":
                    self._router = process
                    break
            if self._router is not None and self._port_is_open():
                return self
            time.sleep(0.05)
        raise TimeoutError("rmw_zenohd did not become ready")

    def snapshot(self) -> dict[str, float | int]:
        if self._router is None:
            raise RuntimeError("rmw_zenohd is not running")
        cpu = self._router.cpu_times()
        switches = self._router.num_ctx_switches()
        return {
            "cpu_seconds": cpu.user + cpu.system,
            "peak_rss_bytes": _peak_rss_bytes(self._router),
            "voluntary_context_switches": switches.voluntary,
            "involuntary_context_switches": switches.involuntary,
        }

    def __exit__(self, *exc: Any) -> None:
        if self._launcher is None or self._launcher.poll() is not None:
            return
        os.killpg(self._launcher.pid, signal.SIGTERM)
        try:
            self._launcher.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(self._launcher.pid, signal.SIGKILL)
            self._launcher.wait(timeout=5.0)


def _subscriber_process(
    spec: TrialSpec,
    receiver: int,
    ready_queue: Any,
    measure_started: Any,
    done: Any,
    result_queue: Any,
) -> None:
    process = psutil.Process()
    started_cpu = 0.0
    received: list[tuple[str, int, int, int]] = []
    received_lock = threading.Lock()
    try:
        bus = ImageBenchmarkBus(spec, "subscriber")

        def record_message(topic_name: str, sequence: int, publish_start_ns: int) -> None:
            if sequence < 0:
                return
            with received_lock:
                received.append((topic_name, sequence, publish_start_ns, time.perf_counter_ns()))

        bus.start()
        for topic in spec.workload.topics:
            bus.subscribe(topic.name, record_message)
        ready_queue.put((receiver, None))
        measurement_deadline = time.monotonic() + _READY_TIMEOUT_S + spec.warmup_s
        while not measure_started.wait(timeout=0.05):
            if done.is_set():
                raise RuntimeError("publisher stopped before measurement started")
            if time.monotonic() >= measurement_deadline:
                raise TimeoutError("measurement start timed out")
        cpu = process.cpu_times()
        started_cpu = cpu.user + cpu.system
        done.wait()
        usage = asdict(_process_usage("subscriber", started_cpu))
        time.sleep(spec.drain_s)
        bus.stop()
        with received_lock:
            result = received.copy()
        result_queue.put((receiver, result, usage, None))
    except Exception as error:
        ready_queue.put((receiver, str(error)))
        result_queue.put(
            (receiver, received, asdict(_process_usage("subscriber", started_cpu)), str(error))
        )


def _publisher_process(
    spec: TrialSpec,
    publisher_ready: Any,
    subscribers_ready: Any,
    measure_started: Any,
    done: Any,
    result_queue: Any,
) -> None:
    process = psutil.Process()
    started_cpu = 0.0
    sent: list[tuple[str, int, int, int, int, int]] = []
    try:
        bus = ImageBenchmarkBus(spec, "publisher")
        bus.start()
        publisher_ready.set()
        if not subscribers_ready.wait(timeout=_READY_TIMEOUT_S):
            raise TimeoutError("subscriber readiness timed out")
        sequence_by_topic = {topic.name: 0 for topic in spec.workload.topics}

        def run_window(duration_s: float, measured: bool) -> float:
            start_ns = time.perf_counter_ns()
            end_ns = start_ns + int(duration_s * 1e9)
            next_ns = {topic.name: start_ns for topic in spec.workload.topics}
            offered_bytes = 0
            while True:
                now_ns = time.perf_counter_ns()
                if now_ns >= end_ns:
                    break
                if measured and spec.workload.saturation:
                    if (
                        spec.workload.saturation_max_messages is not None
                        and len(sent) >= spec.workload.saturation_max_messages
                    ) or (
                        spec.workload.saturation_max_bytes is not None
                        and offered_bytes >= spec.workload.saturation_max_bytes
                    ):
                        break
                due_topic = min(
                    spec.workload.topics,
                    key=lambda workload: next_ns[workload.name],
                )
                due_ns = next_ns[due_topic.name]
                if not spec.workload.saturation and now_ns < due_ns:
                    time.sleep((due_ns - now_ns) / 1e9)
                    if time.perf_counter_ns() >= end_ns:
                        break
                current_sequence = sequence_by_topic[due_topic.name]
                sequence = current_sequence if measured else -1 - current_sequence
                publish_start_ns = time.perf_counter_ns()
                bus.publish(due_topic.name, sequence, publish_start_ns)
                publish_end_ns = time.perf_counter_ns()
                if measured:
                    sent.append(
                        (
                            due_topic.name,
                            sequence,
                            due_topic.payload_bytes,
                            due_ns,
                            publish_start_ns,
                            publish_end_ns,
                        )
                    )
                    offered_bytes += due_topic.payload_bytes
                sequence_by_topic[due_topic.name] = current_sequence + 1
                if due_topic.rate_hz > 0:
                    next_ns[due_topic.name] += int(1e9 / due_topic.rate_hz)
            return (time.perf_counter_ns() - start_ns) / 1e9

        run_window(spec.warmup_s, measured=False)
        sequence_by_topic = {topic.name: 0 for topic in spec.workload.topics}
        cpu = process.cpu_times()
        started_cpu = cpu.user + cpu.system
        measure_started.set()
        measurement_s = run_window(spec.duration_s, measured=True)
        usage = asdict(_process_usage("publisher", started_cpu))
        bus.stop()
        result_queue.put((sent, usage, measurement_s, None))
    except Exception as error:
        result_queue.put((sent, asdict(_process_usage("publisher", started_cpu)), None, str(error)))
    finally:
        done.set()


def _run_local_trial(spec: TrialSpec, router: _LocalRosRouter | None) -> TrialRecord:
    """Run one trial with independent publisher and subscriber processes."""
    context = multiprocessing.get_context("spawn")
    ready_queue = context.Queue()
    subscriber_queue = context.Queue()
    publisher_queue = context.Queue()
    publisher_ready = context.Event()
    subscribers_ready = context.Event()
    measure_started = context.Event()
    done = context.Event()
    publisher = context.Process(
        target=_publisher_process,
        args=(
            spec,
            publisher_ready,
            subscribers_ready,
            measure_started,
            done,
            publisher_queue,
        ),
        name="transport-bench-publisher",
    )
    subscribers = [
        context.Process(
            target=_subscriber_process,
            args=(spec, receiver, ready_queue, measure_started, done, subscriber_queue),
            name=f"transport-bench-subscriber-{receiver}",
        )
        for receiver in range(spec.subscribers)
    ]
    started = time.perf_counter()
    publisher.start()
    if not publisher_ready.wait(timeout=_READY_TIMEOUT_S):
        errors = ["publisher readiness timed out"]
    else:
        errors = []
    for subscriber in subscribers:
        subscriber.start()

    for _ in subscribers:
        try:
            _, error = ready_queue.get(timeout=_READY_TIMEOUT_S)
        except queue.Empty:
            error = "subscriber readiness timed out"
        if error:
            errors.append(error)
    subscribers_ready.set()
    readiness_s = time.perf_counter() - started
    router_start = None
    if router is not None:
        if not measure_started.wait(timeout=spec.warmup_s + _READY_TIMEOUT_S):
            errors.append("router measurement start timed out")
        else:
            router_start = router.snapshot()
    publisher.join(timeout=spec.warmup_s + spec.duration_s + _PROCESS_JOIN_TIMEOUT_S)
    if publisher.is_alive():
        publisher.terminate()
        publisher.join(timeout=_PROCESS_JOIN_TIMEOUT_S)
        errors.append("publisher timed out")
        done.set()
    router_end = router.snapshot() if router is not None and router_start is not None else None

    for subscriber in subscribers:
        subscriber.join(timeout=spec.drain_s + _PROCESS_JOIN_TIMEOUT_S)
        if subscriber.is_alive():
            subscriber.terminate()
            subscriber.join(timeout=_PROCESS_JOIN_TIMEOUT_S)
            errors.append(f"{subscriber.name} timed out")

    try:
        sent, publisher_usage, measurement_s, publisher_error = publisher_queue.get_nowait()
    except queue.Empty:
        sent, publisher_usage, measurement_s, publisher_error = (
            [],
            None,
            None,
            "publisher returned no result",
        )
    if publisher_error:
        errors.append(publisher_error)

    received_by_key: dict[tuple[int, str, int], list[int]] = {}
    out_of_order: set[tuple[int, str, int]] = set()
    usages = []
    if publisher_usage:
        usages.append(ProcessUsage(**publisher_usage))
    if router_start is not None and router_end is not None:
        usages.append(_usage_delta("router", router_start, router_end))
    for _ in subscribers:
        try:
            receiver, received, usage, error = subscriber_queue.get_nowait()
        except queue.Empty:
            errors.append("subscriber returned no result")
            continue
        usages.append(ProcessUsage(**usage))
        if error:
            errors.append(error)
        last_sequence: dict[str, int] = {}
        for topic, sequence, _publish_start_ns, received_ns in received:
            key = (receiver, topic, sequence)
            received_by_key.setdefault(key, []).append(received_ns)
            if sequence < last_sequence.get(topic, sequence):
                out_of_order.add(key)
            last_sequence[topic] = max(sequence, last_sequence.get(topic, sequence))

    samples = []
    for topic, sequence, payload_bytes, scheduled_ns, publish_start_ns, publish_end_ns in sent:
        for receiver in range(spec.subscribers):
            delivery_times = received_by_key.get((receiver, topic, sequence), [])
            samples.append(
                MessageSample(
                    trial_id=spec.trial_id,
                    topic=topic,
                    receiver=receiver,
                    sequence=sequence,
                    payload_bytes=payload_bytes,
                    scheduled_ns=scheduled_ns,
                    publish_start_ns=publish_start_ns,
                    publish_end_ns=publish_end_ns,
                    received_ns=delivery_times[0] if delivery_times else None,
                    delivery_count=len(delivery_times),
                    out_of_order=(receiver, topic, sequence) in out_of_order,
                )
            )
    return TrialRecord(
        spec=spec,
        samples=samples,
        usage=usages,
        readiness_s=readiness_s,
        drain_s=spec.drain_s,
        measurement_s=measurement_s,
        error="; ".join(dict.fromkeys(errors)) if errors else None,
    )


def run_local_trial(spec: TrialSpec) -> TrialRecord:
    """Run one local trial and own any middleware daemon required by the stack."""
    if spec.stack != Stack.ROS2_ZENOH:
        return _run_local_trial(spec, None)
    with _LocalRosRouter() as router:
        return _run_local_trial(spec, router)


def _docker_worker_command(
    container: str,
    role: str,
    spec: TrialSpec,
    output_name: str,
    *,
    receiver: int | None = None,
) -> list[str]:
    command = ["docker", "exec"]
    if spec.stack == Stack.ZENOH:
        if role == "publisher":
            command.extend(["-e", "DIMOS_BENCH_ZENOH_LISTEN=tcp/10.88.0.10:7448"])
        else:
            command.extend(["-e", "DIMOS_BENCH_ZENOH_CONNECT=tcp/10.88.0.10:7448"])
    elif spec.stack == Stack.ROS2_ZENOH:
        override = (
            'mode="client";connect/endpoints=["tcp/10.88.0.10:7447"];'
            "transport/shared_memory/enabled=false"
        )
        command.extend(
            [
                "-e",
                "RMW_IMPLEMENTATION=rmw_zenoh_cpp",
                "-e",
                f"ZENOH_CONFIG_OVERRIDE={override}",
            ]
        )
    command.extend(
        [
            container,
            "/benchmark-entrypoint.sh",
            "/app/.venv/bin/python",
            "-m",
            "dimos.protocol.pubsub.benchmark.container_worker",
            role,
            "--spec",
            "/results/spec.json",
            "--coordination",
            "/results/coordination",
            "--output",
            f"/results/{output_name}",
        ]
    )
    if receiver is not None:
        command.extend(["--receiver", str(receiver)])
    return command


def _wait_for_router(container: str) -> None:
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["docker", "exec", container, "ss", "-ltn"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        if ":7447" in result.stdout:
            return
        time.sleep(0.1)
    raise TimeoutError("rmw_zenohd did not listen on port 7447")


def _router_snapshot(container: str) -> dict[str, float | int]:
    completed = subprocess.run(
        [
            "docker",
            "exec",
            container,
            "/benchmark-entrypoint.sh",
            "/app/.venv/bin/python",
            "-m",
            "dimos.protocol.pubsub.benchmark.resource_snapshot",
            "rmw_zenohd",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=15.0,
    )
    return cast("dict[str, float | int]", json.loads(completed.stdout))


def _usage_delta(
    role: str, start: dict[str, float | int], end: dict[str, float | int]
) -> ProcessUsage:
    return ProcessUsage(
        role=role,
        cpu_seconds=float(end["cpu_seconds"]) - float(start["cpu_seconds"]),
        peak_rss_bytes=int(end["peak_rss_bytes"]),
        voluntary_context_switches=int(end["voluntary_context_switches"])
        - int(start["voluntary_context_switches"]),
        involuntary_context_switches=int(end["involuntary_context_switches"])
        - int(start["involuntary_context_switches"]),
    )


def run_docker_network_trial(
    spec: TrialSpec, work_dir: Path, image: str, logs_dir: Path
) -> TrialRecord:
    """Run one trial in two containers on a symmetrically impaired bridge."""
    from dimos.protocol.pubsub.benchmark.docker_network import DockerNetworkTrial

    if work_dir.exists():
        raise FileExistsError(f"Trial output already exists: {work_dir}")
    coordination = work_dir / "coordination"
    coordination.mkdir(parents=True)
    (work_dir / "spec.json").write_text(json.dumps(spec.to_dict(), sort_keys=True) + "\n")
    errors = []
    usage = []
    started = time.perf_counter()
    with DockerNetworkTrial(spec, work_dir, image=image) as lab:
        (work_dir / "qdisc.json").write_text(
            json.dumps(lab.qdisc_state(), indent=2, sort_keys=True) + "\n"
        )
        (work_dir / "links.json").write_text(
            json.dumps(lab.link_state(), indent=2, sort_keys=True) + "\n"
        )
        (work_dir / "network.json").write_text(
            json.dumps(lab.network_state(), indent=2, sort_keys=True) + "\n"
        )
        if spec.stack == Stack.ROS2_ZENOH:
            subprocess.run(
                [
                    "docker",
                    "exec",
                    "-d",
                    "-e",
                    "RMW_IMPLEMENTATION=rmw_zenoh_cpp",
                    "-e",
                    'ZENOH_CONFIG_OVERRIDE=listen/endpoints=["tcp/0.0.0.0:7447"];transport/shared_memory/enabled=false',
                    lab.publisher_container,
                    "/benchmark-entrypoint.sh",
                    "ros2",
                    "run",
                    "rmw_zenoh_cpp",
                    "rmw_zenohd",
                ],
                check=True,
                timeout=30,
            )
            _wait_for_router(lab.publisher_container)

        publisher_process = subprocess.Popen(
            _docker_worker_command(
                lab.publisher_container,
                "publisher",
                spec,
                "publisher.json",
            ),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        subscriber_processes = [
            subprocess.Popen(
                _docker_worker_command(
                    lab.subscriber_container,
                    "subscriber",
                    spec,
                    f"subscriber-{receiver}.json",
                    receiver=receiver,
                ),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            for receiver in range(spec.subscribers)
        ]
        measurement_deadline = time.monotonic() + spec.warmup_s + 120.0
        while not (coordination / "measure-started").exists():
            if publisher_process.poll() is not None:
                break
            if time.monotonic() >= measurement_deadline:
                raise TimeoutError("container measurement start timed out")
            time.sleep(0.01)
        readiness_s = max(0.0, time.perf_counter() - started - spec.warmup_s)
        router_start = (
            _router_snapshot(lab.publisher_container) if spec.stack == Stack.ROS2_ZENOH else None
        )
        try:
            publisher_stdout, publisher_stderr = publisher_process.communicate(
                timeout=spec.duration_s + 180.0
            )
        except subprocess.TimeoutExpired:
            publisher_process.terminate()
            publisher_stdout, publisher_stderr = publisher_process.communicate(timeout=15.0)
            errors.append("publisher container timed out")
        if publisher_process.returncode:
            errors.append(publisher_stderr.strip() or "publisher container failed")
        if router_start is not None:
            usage.append(
                _usage_delta(
                    "router",
                    router_start,
                    _router_snapshot(lab.publisher_container),
                )
            )
        logs_dir.mkdir(parents=True, exist_ok=True)
        (logs_dir / f"{spec.trial_id}-publisher.log").write_text(
            publisher_stdout + publisher_stderr
        )
        for process in subscriber_processes:
            try:
                stdout, stderr = process.communicate(timeout=spec.duration_s + 60.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                _, stderr = process.communicate(timeout=15.0)
                errors.append("subscriber container timed out")
            if process.returncode:
                errors.append(stderr.strip() or "subscriber container failed")
            receiver = subscriber_processes.index(process)
            (logs_dir / f"{spec.trial_id}-subscriber-{receiver}.log").write_text(stdout + stderr)

    sent = []
    measurement_s = None
    publisher_path = work_dir / "publisher.json"
    if publisher_path.exists():
        publisher_result = json.loads(publisher_path.read_text())
        sent = publisher_result["sent"]
        measurement_s = publisher_result["measurement_s"]
        usage.append(ProcessUsage(**publisher_result["usage"]))
        if publisher_result["error"]:
            errors.append(publisher_result["error"])
        publisher_path.unlink()
    else:
        errors.append("publisher result is missing")

    received_by_key: dict[tuple[int, str, int], list[int]] = {}
    out_of_order: set[tuple[int, str, int]] = set()
    for receiver in range(spec.subscribers):
        path = work_dir / f"subscriber-{receiver}.json"
        if not path.exists():
            errors.append(f"subscriber {receiver} result is missing")
            continue
        result = json.loads(path.read_text())
        usage.append(ProcessUsage(**result["usage"]))
        if result["error"]:
            errors.append(result["error"])
        last_sequence: dict[str, int] = {}
        for topic, sequence, _publish_start_ns, received_ns in result["received"]:
            key = (receiver, topic, sequence)
            received_by_key.setdefault(key, []).append(received_ns)
            if sequence < last_sequence.get(topic, sequence):
                out_of_order.add(key)
            last_sequence[topic] = max(sequence, last_sequence.get(topic, sequence))
        path.unlink()

    samples = []
    for topic, sequence, payload_bytes, scheduled_ns, publish_start_ns, publish_end_ns in sent:
        for receiver in range(spec.subscribers):
            delivery_times = received_by_key.get((receiver, topic, sequence), [])
            samples.append(
                MessageSample(
                    trial_id=spec.trial_id,
                    topic=topic,
                    receiver=receiver,
                    sequence=sequence,
                    payload_bytes=payload_bytes,
                    scheduled_ns=scheduled_ns,
                    publish_start_ns=publish_start_ns,
                    publish_end_ns=publish_end_ns,
                    received_ns=delivery_times[0] if delivery_times else None,
                    delivery_count=len(delivery_times),
                    out_of_order=(receiver, topic, sequence) in out_of_order,
                )
            )
    return TrialRecord(
        spec=spec,
        samples=samples,
        usage=usage,
        readiness_s=max(0.0, readiness_s),
        drain_s=spec.drain_s,
        measurement_s=measurement_s,
        error="; ".join(dict.fromkeys(errors)) if errors else None,
    )


def run_campaign(
    *,
    suite: str,
    output_dir: Path,
    stacks: set[Stack] | None = None,
    environments: set[Environment] | None = None,
    profiles: set[NetworkProfile] | None = None,
    repetitions: int | None = None,
    warmup_s: float | None = None,
    duration_s: float | None = None,
    drain_s: float | None = None,
    seed: int = 7,
    command: list[str] | None = None,
    image: str = "dimos-transport-benchmark:local",
) -> Path:
    specs = build_matrix(
        suite,
        repetitions=repetitions,
        warmup_s=warmup_s,
        duration_s=duration_s,
        drain_s=drain_s,
        seed=seed,
    )
    if suite == "smoke" and environments == {Environment.EMULATED}:
        specs = [
            replace(
                spec,
                environment=Environment.EMULATED,
                profile=NetworkProfile.CLEAN,
            )
            for spec in specs
        ]
    specs = [
        spec
        for spec in specs
        if (stacks is None or spec.stack in stacks)
        and (environments is None or spec.environment in environments)
        and (profiles is None or spec.profile in profiles)
    ]
    manifest = collect_manifest(
        suite=suite,
        seed=seed,
        command=command or [],
        benchmark_image=image,
    )
    if any(spec.environment == Environment.EMULATED for spec in specs):
        manifest["benchmark_container"] = collect_container_manifest(image)
    manifest["trials"] = [spec.to_dict() for spec in specs]
    writer = ArtifactWriter(output_dir, manifest)
    completed = {
        summary["trial_id"]: summary
        for summary in load_trial_summaries(output_dir)
        if summary.get("error") is None
    }
    pending = [spec for spec in specs if spec.trial_id not in completed]
    if completed:
        print(
            f"Resuming {len(completed)} completed trials; {len(pending)} remain",
            flush=True,
        )
    initial_completed = len(completed)
    campaign_started = time.monotonic()
    for index, spec in enumerate(pending, start=1):
        ordinal = initial_completed + index
        print(
            f"[{ordinal}/{len(specs)}] {spec.stack.value} {spec.cohort.value} "
            f"{spec.workload.name} {spec.environment.value}/{spec.profile.value} "
            f"fanout={spec.subscribers} repetition={spec.repetition + 1}",
            flush=True,
        )
        writer.prepare_retry(spec.trial_id)
        record = None
        for attempt in range(1, 5):
            try:
                if spec.environment == Environment.LOCAL:
                    record = run_local_trial(spec)
                else:
                    record = run_docker_network_trial(
                        spec,
                        writer.topology_dir / spec.trial_id,
                        image,
                        writer.logs_dir,
                    )
                break
            except Exception as error:
                writer.prepare_retry(spec.trial_id)
                if attempt == 4:
                    record = TrialRecord(spec=spec, error=f"trial setup failed: {error}")
                    break
                delay_s = 5 * 2 ** (attempt - 1)
                print(f"setup attempt {attempt}/4 failed; retrying in {delay_s}s: {error}")
                time.sleep(delay_s)
        if record is None:
            raise AssertionError("trial retry loop returned no record")
        completed[spec.trial_id] = writer.write_trial(record)
        elapsed_s = time.monotonic() - campaign_started
        remaining_s = elapsed_s / index * (len(pending) - index)
        delivered = sum(sample.received_ns is not None for sample in record.samples)
        print(
            f"[{ordinal}/{len(specs)}] delivered={delivered} "
            f"error={record.error or 'none'} eta={remaining_s / 3600:.2f}h",
            flush=True,
        )
    summaries = [completed[spec.trial_id] for spec in specs]
    writer.finalize(summaries, seed=seed)
    generate_report(output_dir)
    return output_dir
