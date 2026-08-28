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

"""End-to-end: bake ray_tracing + mls_planner and run the binary for real.

Excluded from the default run because it builds rust. Run it with
``pytest -m bake_e2e dimos/cli/bake/test_bake_e2e.py``.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import subprocess
import time

import numpy as np
import pytest

from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

pytestmark = pytest.mark.bake_e2e

SUPPRESSED = ("global_map", "local_map", "region_bounds")
# The sensor sits a meter above a flat patch of floor.
SENSOR_Z = 1.0


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


@pytest.fixture(scope="module")
def baked(tmp_path_factory) -> tuple[Path, dict[str, object]]:
    """The compiled host plus the stdin config bake emitted for it."""
    out = tmp_path_factory.mktemp("bake") / "go2-nav"
    config = out.parent / "go2-nav.json"
    cmd = [
        "dimos",
        "bake",
        "ray-tracing",
        "mls-planner",
        "-o",
        str(out),
        "--debug",
        "--emit-config",
        str(config),
    ]
    for topic in SUPPRESSED:
        cmd += ["--suppress", topic]
    subprocess.run(cmd, check=True)
    return out, json.loads(config.read_text())


def loopback_session(port: int) -> dict[str, object]:
    """A session reachable only over loopback, so the test never scouts the LAN."""
    return {
        "mode": "peer",
        "connect": [],
        "listen": [f"tcp/127.0.0.1:{port}"],
        "multicast": False,
        "scout_addr": "",
        "gossip": False,
        "interface": "lo",
        "connect_timeout_ms": 0,
    }


def spawn_host(binary: Path, config: dict[str, object], port: int) -> subprocess.Popen[bytes]:
    env = {**os.environ, "DIMOS_TRANSPORT": "zenoh", "RUST_LOG": "warn"}
    launch = {**config, "session": loopback_session(port)}
    proc = subprocess.Popen([str(binary)], env=env, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None
    proc.stdin.write(json.dumps(launch).encode() + b"\n")
    proc.stdin.close()
    return proc


def await_listener(proc: subprocess.Popen[bytes], port: int, timeout: float = 20.0) -> None:
    """Block until the host's zenoh endpoint accepts, so our dial cannot race it."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            stderr = proc.stderr.read().decode(errors="replace") if proc.stderr else ""
            raise AssertionError(f"the host exited before it started listening:\n{stderr}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise AssertionError(f"the host never listened on {port}")


def ground_patch(ts: float) -> PointCloud2:
    """A flat floor patch below the sensor, enough for the planner to surface."""
    grid = np.arange(-10, 11) * 0.1
    xs, ys = np.meshgrid(grid, grid)
    points = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -SENSOR_Z)], axis=1).astype(
        np.float32
    )
    return PointCloud2.from_numpy(points, frame_id="lidar", timestamp=ts)


@pytest.fixture
def zenoh_to(monkeypatch):
    """Point dimos's zenoh session at the host's listen endpoint, nothing else."""

    def connect(port: int):
        monkeypatch.setattr(global_config, "transport", "zenoh")
        monkeypatch.setattr(global_config, "robot_ip", f"127.0.0.1:{port}")
        monkeypatch.setattr(global_config, "zenoh_scouting", False)

    return connect


def test_the_host_publishes_its_outputs_and_hides_the_suppressed_hop(baked, zenoh_to):
    binary, config = baked
    port = free_port()
    zenoh_to(port)
    proc = spawn_host(binary, config, port)
    await_listener(proc, port)

    seen: dict[str, int] = {}
    transports = []
    try:
        for name, msg_type in (
            ("surface_map", PointCloud2),
            ("global_map", PointCloud2),
            ("local_map", PointCloud2),
        ):
            transport = make_transport(name, msg_type)
            transport.start()
            transport.subscribe(
                lambda _msg, name=name: seen.__setitem__(name, seen.get(name, 0) + 1)
            )
            transports.append(transport)

        lidar = make_transport("lidar", PointCloud2)
        tf = make_transport("tf", Transform)
        transports += [lidar, tf]
        lidar.start()
        tf.start()

        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and "surface_map" not in seen:
            # Same stamp on both: the mapper drops a cloud it has no transform for.
            ts = time.time()
            tf.broadcast(
                None,
                Transform(
                    translation=Vector3(0.0, 0.0, SENSOR_Z),
                    frame_id="odom",
                    child_frame_id="lidar",
                    ts=ts,
                ),
            )
            time.sleep(0.05)
            lidar.broadcast(None, ground_patch(ts))
            time.sleep(0.25)

        assert proc.poll() is None, "the host exited before publishing anything"
        assert "surface_map" in seen, "the planner never published through the host"
        assert "global_map" not in seen, "a suppressed topic reached the bus"
        assert "local_map" not in seen, "a suppressed topic reached the bus"
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        for transport in transports:
            transport.stop()


def test_a_poisoned_config_kills_the_host(baked):
    binary, config = baked
    poisoned = json.loads(json.dumps(config))
    poisoned["modules"]["ray_tracing"]["config"].pop("voxel_size")
    proc = spawn_host(binary, poisoned, free_port())
    assert proc.wait(timeout=30) == 1
    assert b"voxel_size" in (proc.stderr.read() if proc.stderr else b"")


def test_a_config_baked_for_another_graph_is_refused(baked):
    binary, config = baked
    proc = spawn_host(binary, {**config, "graph": "0123456789abcdef"}, free_port())
    assert proc.wait(timeout=30) == 1
    assert proc.stderr is not None
    stderr = proc.stderr.read()
    assert b"0123456789abcdef" in stderr
    assert b"dimos bake --emit-config" in stderr
