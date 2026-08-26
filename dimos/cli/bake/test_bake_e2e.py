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

"""End-to-end: bake ray_tracing + mls_planner as a deployment and run the binary for real.

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

from dimos.cli.bake.deployment import Deployment
from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

pytestmark = pytest.mark.bake_e2e

SUPPRESSED = ("global_map", "local_map", "region_bounds")
# The sensor sits a meter above a flat patch of floor.
SENSOR_Z = 1.0


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def loopback_session(port: int) -> dict[str, object]:
    """A session reachable only over loopback, so the test never scouts the LAN."""
    return {
        "mode": "peer",
        "connect": [],
        "listen": [f"tcp/127.0.0.1:{port}"],
        "multicast": False,
        "gossip": False,
        "interface": "lo",
        "connect_timeout_ms": 0,
    }


# The bake subprocess imports this module too; the env var keeps both on one port.
PORT = int(os.environ.get("DIMOS_BAKE_E2E_PORT") or free_port())
E2E = Deployment(("ray_tracing", "mls_planner"), session=loopback_session(PORT))


@pytest.fixture(scope="module")
def baked(tmp_path_factory) -> Path:
    """The compiled host, the E2E deployment embedded."""
    out = tmp_path_factory.mktemp("bake") / "go2-nav"
    cmd = ["dimos", "bake", "-o", str(out), "--debug", "--deployment", f"{__name__}:E2E"]
    for topic in SUPPRESSED:
        cmd += ["--suppress", topic]
    subprocess.run(cmd, check=True, env={**os.environ, "DIMOS_BAKE_E2E_PORT": str(PORT)})
    return out


def spawn_host(binary: Path, override: dict[str, object] | None = None) -> subprocess.Popen[bytes]:
    """Run the host on its embedded config; `override` is the sparse stdin line."""
    env = {**os.environ, "DIMOS_TRANSPORT": "zenoh", "RUST_LOG": "warn"}
    stdin = subprocess.DEVNULL if override is None else subprocess.PIPE
    proc = subprocess.Popen([str(binary)], env=env, stdin=stdin, stderr=subprocess.PIPE)
    if override is not None:
        assert proc.stdin is not None
        proc.stdin.write(json.dumps(override).encode() + b"\n")
        proc.stdin.close()
    return proc


def await_listener(proc: subprocess.Popen[bytes], port: int, timeout: float = 20.0) -> None:
    """Block until the host's zenoh endpoint accepts, so our dial cannot race it."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        assert proc.poll() is None, "the host exited before it started listening"
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
    zenoh_to(PORT)
    proc = spawn_host(baked)
    await_listener(proc, PORT)

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
        odom = make_transport("odometry", Odometry)
        transports += [lidar, odom]
        lidar.start()
        odom.start()

        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and "surface_map" not in seen:
            # Same stamp on both: the mapper drops a cloud it has no pose for.
            ts = time.time()
            odom.broadcast(
                None,
                Odometry(ts=ts, frame_id="world", pose=Pose(0.0, 0.0, SENSOR_Z)),
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


def test_a_stdin_override_moves_the_listener(baked):
    port = free_port()
    proc = spawn_host(baked, {"session": {"listen": [f"tcp/127.0.0.1:{port}"]}})
    try:
        await_listener(proc, port)
    finally:
        proc.terminate()
        proc.wait(timeout=10)


def test_an_override_with_a_key_the_struct_lacks_kills_the_host(baked):
    proc = spawn_host(baked, {"modules": {"ray_tracing": {"config": {"voxel_sizes": 0.1}}}})
    assert proc.wait(timeout=30) == 1
    assert b"voxel_sizes" in (proc.stderr.read() if proc.stderr else b"")


def test_an_override_stamped_for_another_graph_is_refused(baked):
    proc = spawn_host(baked, {"graph": "0123456789abcdef"})
    assert proc.wait(timeout=30) == 1
    assert proc.stderr is not None
    stderr = proc.stderr.read()
    assert b"0123456789abcdef" in stderr
    assert b"dimos bake" in stderr
