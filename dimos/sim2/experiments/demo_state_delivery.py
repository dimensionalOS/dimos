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

"""Cross-process state fan-out using DimOS's existing raw Zenoh pubsub implementation.

Each spawned consumer decodes and reconstructs usable root/joint arrays. No
physics model is copied into consumers. Capture-to-ready latency includes capture,
codec, local TCP transport, decode and projection. All strategies have the same
24-byte benchmark envelope, topic/QoS, route handshake and receiver workload.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime
import json
import multiprocessing as mp
import random
import socket
import struct
import threading
import time
from typing import Any, cast
from uuid import uuid4

import numpy as np

from dimos.protocol.pubsub.impl.zenohpubsub import QOS_NEVER_DROP, Topic, ZenohPubSubBase
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.sim2.experiments.demo_state_scaling import (
    OUTPUT,
    STRATEGIES,
    Pipeline,
    WorldPair,
    command,
    consume_state,
    prepare,
    provenance,
    summary,
)

HEADER = struct.Struct(">QQd")


def subscriber(
    pipe: Any, endpoint: str, topic_name: str, kind: str, descriptor: dict[str, Any], work_ms: float
) -> None:
    modules = prepare()
    mirror = modules.world.WorldMirror()
    condition = threading.Condition()
    rows: list[list[float]] = []
    errors: list[str] = []
    last = 0
    initialised = False
    cpu_start = 0.0
    roots = np.asarray(descriptor["root_rows"])
    pool = ZenohSessionPool()
    bus = ZenohPubSubBase(
        session_pool=pool,
        mode="peer",
        listen=[],
        connect=[endpoint],
        scouting=False,
        multicast=False,
        gossip=False,
        connect_timeout=5,
    )

    def received(data: bytes, topic: Topic) -> None:
        nonlocal last, initialised, cpu_start
        seq, start, expected = HEADER.unpack_from(data)
        arrived = time.perf_counter_ns()
        try:
            result = consume_state(
                kind,
                data[HEADER.size :],
                mirror,
                roots,
                descriptor["names"],
                descriptor["joint_roots"],
                modules,
            )
            # Deliberate additional subscriber CPU, kept separate from normal runs.
            until = time.perf_counter_ns() + round(work_ms * 1e6)
            while time.perf_counter_ns() < until:
                pass
            probe = float(np.mean(result[0][:, 0]))
            finished = time.perf_counter_ns()
            with condition:
                if seq == 0:
                    if not initialised:
                        initialised = True
                        cpu_start = time.process_time()
                        pipe.send({"baseline": True})
                else:
                    rows.append(
                        [
                            seq,
                            (finished - start) / 1000,
                            (finished - arrived) / 1000,
                            abs(probe - expected),
                        ]
                    )
                    last = max(last, seq)
                    condition.notify_all()
        except Exception as error:
            with condition:
                errors.append(repr(error))
                condition.notify_all()

    try:
        bus.start()
        bus.subscribe(Topic(topic_name), received)
        pipe.send({"ready": True})
        requested = pipe.recv()
        with condition:
            condition.wait_for(lambda: last >= requested or bool(errors), timeout=10)
        cpu = time.process_time() - cpu_start
        bus.stop()
        unique = {int(row[0]) for row in rows}
        pipe.send(
            {
                "rows": rows,
                "errors": errors,
                "received": len(unique),
                "missing": requested - len(unique),
                "cpu_seconds": cpu,
                "max_probe_error": max((row[3] for row in rows), default=0),
            }
        )
    except BaseException as error:
        pipe.send({"fatal": repr(error)})
        raise
    finally:
        bus.stop()
        pool.close_all()
        pipe.close()


def stop_process(process: Any) -> None:
    process.join(timeout=3)
    if process.is_alive():
        process.terminate()
        process.join(timeout=3)


def receive(pipe: Any, timeout: float = 15) -> dict[str, Any]:
    if not pipe.poll(timeout):
        raise TimeoutError("benchmark subscriber did not respond")
    result = pipe.recv()
    if "fatal" in result:
        raise RuntimeError(result["fatal"])
    return cast("dict[str, Any]", result)


def delivery_run(
    pair: WorldPair,
    kind: str,
    hz: int,
    readers: int,
    duration: float,
    repeat: int,
    fraction: float = 0.01,
    work_ms: float = 0,
) -> dict[str, Any]:
    pair.reset()
    pipeline = Pipeline(pair, kind)
    topic_name = f"dimos/state-experiment/{uuid4().hex}"
    topic = Topic(topic_name, qos=QOS_NEVER_DROP)
    descriptor = {
        "names": pair.names,
        "root_rows": pair.root_rows.tolist(),
        "joint_roots": [name for name in pair.names if pair.jeff._joints[name][0]],
    }
    context = mp.get_context("spawn")
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
    pool = ZenohSessionPool()
    bus = ZenohPubSubBase(
        session_pool=pool,
        mode="peer",
        listen=[endpoint],
        connect=[],
        scouting=False,
        multicast=False,
        gossip=False,
        connect_timeout=0,
    )
    with ExitStack() as cleanup:
        cleanup.callback(pool.close_all)
        bus.start()
        cleanup.callback(bus.stop)
        pipes = []
        for _ in range(readers):
            parent, child = context.Pipe()
            process = context.Process(
                target=subscriber, args=(child, endpoint, topic_name, kind, descriptor, work_ms)
            )
            process.start()
            child.close()
            cleanup.callback(parent.close)
            cleanup.callback(stop_process, process)
            pipes.append(parent)
        for pipe in pipes:
            assert receive(pipe) == {"ready": True}
        initial = pipeline.encode(pipeline.capture())
        initial_probe = float(np.mean(pair.pim.data.xpos[pair.root_ids, 0]))
        # Confirm actual routing and baseline decoding in every process before timing.
        waiting = set(range(readers))
        deadline = time.monotonic() + 10
        while waiting:
            if time.monotonic() > deadline:
                raise TimeoutError("subscriber baseline acknowledgement")
            bus.publish(topic, HEADER.pack(0, time.perf_counter_ns(), initial_probe) + initial)
            for index in list(waiting):
                if pipes[index].poll(0.05):
                    assert receive(pipes[index]) == {"baseline": True}
                    waiting.remove(index)
        frames = round(hz * duration)
        rows = []
        total_bytes = 0
        cpu_start = time.process_time()
        began = time.perf_counter()
        deadline = began
        pause = threading.Event()
        for seq in range(1, frames + 1):
            deadline += 1 / hz
            pair.pose_sequence(seq, fraction, 1 / hz)
            expected = float(np.mean(pair.pim.data.xpos[pair.root_ids, 0]))
            start = time.perf_counter_ns()
            payload = pipeline.encode(pipeline.capture())
            encoded = time.perf_counter_ns()
            bus.publish(topic, HEADER.pack(seq, start, expected) + payload)
            sent = time.perf_counter_ns()
            total_bytes += len(payload)
            rows.append(
                [
                    seq,
                    (encoded - start) / 1000,
                    (sent - encoded) / 1000,
                    len(payload),
                    max(0, time.perf_counter() - deadline) * 1e6,
                ]
            )
            pause.wait(max(0, deadline - time.perf_counter()))
        publish_seconds = time.perf_counter() - began
        cpu = time.process_time() - cpu_start
        for pipe in pipes:
            pipe.send(frames)
        consumers = [receive(pipe, 20) for pipe in pipes]
        finished = time.perf_counter() - began
    latency = [row[1] for consumer in consumers for row in consumer["rows"]]
    return {
        "kind": kind,
        "props": len(pair.free_q),
        "hz": hz,
        "readers": readers,
        "repeat": repeat,
        "fraction": fraction,
        "consumer_work_ms": work_ms,
        "frames": frames,
        "publication_seconds": publish_seconds,
        "drained_seconds": finished,
        "achieved_hz": frames / publish_seconds,
        "producer_cpu_seconds": cpu,
        "consumer_cpu_seconds": sum(c["cpu_seconds"] for c in consumers),
        "application_bytes": total_bytes,
        "initial_bytes": len(initial),
        "latency_us": summary(latency) if latency else None,
        "producer_rows": rows,
        "consumers": consumers,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default=datetime.now().strftime("delivery-%Y%m%d-%H%M%S"))
    parser.add_argument("--props", nargs="+", type=int, default=[32, 512, 2048])
    parser.add_argument("--hz", nargs="+", type=int, default=[50, 200])
    parser.add_argument("--readers", nargs="+", type=int, default=[1, 4])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--duration", type=float, default=2)
    parser.add_argument("--fraction", type=float, default=0.01)
    parser.add_argument("--consumer-work-ms", type=float, default=0)
    args = parser.parse_args()
    directory = OUTPUT / args.name
    directory.mkdir(parents=True, exist_ok=False)
    record: dict[str, Any] = {"provenance": provenance(), "protocol": vars(args), "delivery": []}
    path = directory / "results.json"
    rng = random.Random(20261102)
    for props in args.props:
        pair = WorldPair.build(directory / f"scene-{props}", props)
        for hz in args.hz:
            for readers in args.readers:
                for repeat in range(args.repeats):
                    kinds = list(STRATEGIES)
                    rng.shuffle(kinds)
                    for kind in kinds:
                        result = delivery_run(
                            pair,
                            kind,
                            hz,
                            readers,
                            args.duration,
                            repeat,
                            args.fraction,
                            args.consumer_work_ms,
                        )
                        record["delivery"].append(result)
                        record["power_end"] = command("pmset", "-g", "batt")
                        path.write_text(json.dumps(record, indent=2, allow_nan=False))
                        print(
                            json.dumps(
                                {
                                    "stage": "delivery",
                                    "kind": kind,
                                    "props": props,
                                    "hz": hz,
                                    "readers": readers,
                                    "repeat": repeat,
                                    "achieved": result["achieved_hz"],
                                    "latency": result["latency_us"],
                                    "missing": sum(c["missing"] for c in result["consumers"]),
                                }
                            ),
                            flush=True,
                        )


if __name__ == "__main__":
    main()
