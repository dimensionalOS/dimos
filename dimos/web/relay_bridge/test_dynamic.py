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

"""Tests for dynamically generated relay bridge classes that cross the worker pipe."""

# No `from __future__ import annotations` here on purpose: under PEP 563 a
# Module class whose annotations become strings silently loses its streams
# (see the module-level note in test_relay_bridge_module.py).

from dataclasses import dataclass
import subprocess
import sys
import threading

import pytest

from dimos.core.coordination.blueprints import BlueprintAtom, StreamRef, autoconnect
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.dynamic_module import PortSpec
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module, PeekNotFound
from dimos.core.stream import In, Out, RemoteIn
from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.encode.cbor import decode, encode
from dimos.web.cockpit import Channel, cockpit
from dimos.web.relay_bridge.dynamic import DynamicPortSpec, make_relay_bridge_class

# Static peers for autoconnect matching against generated ports.


class _PingProducer(Module):
    camera_feed: Out[Vector3]


class _NoteConsumer(Module):
    operator_note: In[str]


_MAIN_SPECS = (
    DynamicPortSpec("sensor_ping", Vector3, "rx"),
    DynamicPortSpec("operator_note", str, "tx"),
)
_AUTOCONNECT_SPECS = (
    DynamicPortSpec("camera_feed", Vector3, "rx"),
    DynamicPortSpec("operator_note", str, "tx"),
)


@pytest.fixture
def worker_manager():
    manager = WorkerManagerPython(g=GlobalConfig(n_workers=1))
    manager.start()
    yield manager
    manager.stop()


@pytest.fixture
def deployed_proxies(worker_manager):
    """Proxies appended here are stopped (module + parent-side RPC client)
    before the worker manager tears down."""
    proxies = []
    yield proxies
    for proxy in reversed(proxies):
        proxy.stop()


@pytest.fixture
def ping_transport():
    transport = pLCMTransport("/test_dynamic/fork_ping")
    transport.start()
    yield transport
    transport.stop()


def test_rx_and_tx_map_to_in_and_out_ports():
    generated = make_relay_bridge_class(_MAIN_SPECS)
    assert generated._dynamic_ports == (
        PortSpec("operator_note", str, "out"),
        PortSpec("sensor_ping", Vector3, "in"),
    )


_RELOAD_SCRIPT = """
import importlib, sys
from dimos.msgs.geometry_msgs.Vector3 import Vector3
import dimos.web.relay_bridge.dynamic as dynamic

C = dynamic.make_relay_bridge_class([dynamic.DynamicPortSpec("reload_feed", Vector3, "rx")])
importlib.reload(sys.modules["dimos.core.dynamic_module"])
again = dynamic.make_relay_bridge_class([dynamic.DynamicPortSpec("reload_feed", Vector3, "rx")])
assert again is C, "factory lost identity after reload"
"""


def test_factory_keeps_identity_after_core_reload():
    """Runs in a subprocess so the reload cannot poison this process's module state."""
    result = subprocess.run(
        [sys.executable, "-c", _RELOAD_SCRIPT], capture_output=True, text=True, timeout=120
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    ("specs", "expected", "match"),
    [
        ([DynamicPortSpec("@control", Vector3, "rx")], ValueError, "reserved"),
        ([DynamicPortSpec("x" * 65, Vector3, "rx")], ValueError, "longer than"),
        ([DynamicPortSpec("ref", Vector3, "rx")], ValueError, "collides"),
        ([DynamicPortSpec("bad_dir", Vector3, "sideways")], ValueError, "direction"),
    ],
)
def test_invalid_specs_rejected(specs, expected, match):
    with pytest.raises(expected, match=match):
        make_relay_bridge_class(specs)


def test_blueprint_atom_discovers_exact_stream_refs():
    generated = make_relay_bridge_class(_MAIN_SPECS)
    atom = BlueprintAtom.create(generated, kwargs={})
    assert atom.module_refs == ()
    assert atom.streams == (
        StreamRef(name="color_image", type=Image, direction="in"),
        StreamRef(name="odom", type=PoseStamped, direction="in"),
        StreamRef(name="global_costmap", type=OccupancyGrid, direction="in"),
        StreamRef(name="tele_cmd_vel", type=Twist, direction="out"),
        StreamRef(name="operator_note", type=str, direction="out"),
        StreamRef(name="sensor_ping", type=Vector3, direction="in"),
    )


def test_autoconnect_matches_generated_ports_with_static_modules():
    generated = make_relay_bridge_class(_AUTOCONNECT_SPECS)
    blueprint = autoconnect(
        _PingProducer.blueprint(), generated.blueprint(), _NoteConsumer.blueprint()
    )
    # Group ports by (name, type) exactly as ModuleCoordinator._connect_streams
    # does; a shared key means a shared transport, i.e. a connection.
    groups: dict[tuple[str, type], list[tuple[str, str]]] = {}
    for atom in blueprint.active_blueprints:
        for ref in atom.streams:
            groups.setdefault((ref.name, ref.type), []).append((atom.name, ref.direction))
    assert groups[("camera_feed", Vector3)] == [
        ("_pingproducer", "out"),
        (generated.name, "in"),
    ]
    assert groups[("operator_note", str)] == [
        (generated.name, "out"),
        ("_noteconsumer", "in"),
    ]


def test_blueprint_roundtrip_preserves_generated_class_identity():
    generated = make_relay_bridge_class(_AUTOCONNECT_SPECS)
    blueprint = autoconnect(
        _PingProducer.blueprint(), generated.blueprint(), _NoteConsumer.blueprint()
    )
    restored = decode(encode(blueprint))
    assert restored == blueprint
    restored_atom = next(a for a in restored.blueprints if a.name == generated.name)
    assert restored_atom.module is generated


@pytest.mark.skipif_flaky_in_ci
def test_forkserver_worker_predates_class_deploy_rpc_and_stream(
    worker_manager, deployed_proxies, ping_transport, retry_until
):
    """The W5 flagship proof: the worker process exists before the class does,
    so the class can only arrive in the worker through the pipe, not fork
    inheritance. Deploy, call an RPC, wire a generated stream, see data."""
    assert worker_manager.workers[0].pid is not None

    generated = make_relay_bridge_class(
        [
            DynamicPortSpec("fork_ping", Vector3, "rx"),
            DynamicPortSpec("fork_note", str, "tx"),
        ]
    )
    proxy = worker_manager.deploy(generated, global_config, {})
    deployed_proxies.append(proxy)

    assert isinstance(proxy.peek_stream("no_such_stream", 0.1), PeekNotFound)

    # Fetched before wiring: the returned RemoteIn carries no transport, so it
    # cannot spawn an untracked LCM thread in this process.
    remote = proxy.fork_ping
    assert isinstance(remote, RemoteIn)
    assert remote.type is Vector3

    assert proxy.set_transport("fork_ping", ping_transport) is True

    received = []
    errors = []
    got = threading.Event()

    def peek():
        try:
            value = proxy.peek_stream("fork_ping", 15.0)
            if value is not None:
                received.append(value)
        except Exception as e:  # surfaced below instead of a bare timeout
            errors.append(e)
        finally:
            got.set()

    peeker = threading.Thread(target=peek)
    peeker.start()
    try:
        retry_until(got, lambda: ping_transport.publish(Vector3(1.0, 2.0, 3.0)), timeout=10.0)
    finally:
        peeker.join(timeout=20.0)
    assert not errors, errors
    assert received == [Vector3(1.0, 2.0, 3.0)]


def test_deploy_fresh_process_reconstructs_class(worker_manager, deployed_proxies):
    generated = make_relay_bridge_class([DynamicPortSpec("fresh_ping", Vector3, "rx")])
    # instance_name keeps the two deployments of the same class from serving
    # identical class-name RPC topics.
    first = worker_manager.deploy(generated, global_config, {"instance_name": "dyn_first"})
    deployed_proxies.append(first)
    fresh = worker_manager.deploy_fresh(generated, global_config, {"instance_name": "dyn_fresh"})
    deployed_proxies.append(fresh)

    pids = [worker.pid for worker in worker_manager.workers]
    assert len(pids) == 2
    assert None not in pids
    assert pids[0] != pids[1]

    assert isinstance(first.peek_stream("no_such_stream", 0.1), PeekNotFound)
    assert isinstance(fresh.peek_stream("no_such_stream", 0.1), PeekNotFound)


def test_two_shapes_deploy_to_one_worker(worker_manager, deployed_proxies):
    shape_a = make_relay_bridge_class([DynamicPortSpec("duo_feed_a", Vector3, "rx")])
    shape_b = make_relay_bridge_class([DynamicPortSpec("duo_feed_b", str, "tx")])
    proxy_a = worker_manager.deploy(shape_a, global_config, {})
    deployed_proxies.append(proxy_a)
    proxy_b = worker_manager.deploy(shape_b, global_config, {})
    deployed_proxies.append(proxy_b)

    assert worker_manager.workers[0].module_count == 2
    # Shape A does not have shape B's port; shape B has it but unwired.
    assert isinstance(proxy_a.peek_stream("duo_feed_b", 0.1), PeekNotFound)
    assert proxy_b.peek_stream("duo_feed_b", 0.1) is None


@dataclass(frozen=True)
class _OpsNote:
    text: str
    priority: int


def test_cockpit_channels_blueprint_deploys_through_forkserver(worker_manager, deployed_proxies):
    """W6: a cockpit(channels=)-compiled atom - generated class plus runtime
    specs carrying by-reference encoder callables in the kwargs - must
    survive the real forkserver deploy path (config re-validates in the
    child)."""
    assert worker_manager.workers[0].pid is not None

    blueprint = cockpit(
        channels=[
            Channel("target_pose", PoseStamped, encoding="pose.json.v1", max_hz=5.0),
            Channel("ops_note", _OpsNote),
        ]
    )
    (atom,) = blueprint.blueprints
    assert atom.module.__name__.startswith("DynamicRelayBridgeModule_")
    proxy = worker_manager.deploy(atom.module, global_config, atom.kwargs)
    deployed_proxies.append(proxy)

    assert isinstance(proxy.peek_stream("no_such_stream", 0.1), PeekNotFound)
    # Both generated ports exist in the worker (unwired: None, not NotFound).
    assert proxy.peek_stream("target_pose", 0.1) is None
    assert proxy.peek_stream("ops_note", 0.1) is None
    remote = proxy.target_pose
    assert isinstance(remote, RemoteIn)
    assert remote.type is PoseStamped


def test_stop_terminates_worker_processes_and_is_idempotent():
    # This test's subject is teardown itself, so it owns the manager inline.
    manager = WorkerManagerPython(g=GlobalConfig(n_workers=1))
    manager.start()
    try:
        generated = make_relay_bridge_class([DynamicPortSpec("teardown_feed", Vector3, "rx")])
        proxy = manager.deploy(generated, global_config, {})
        workers = manager.workers
        proxy.stop()
    finally:
        manager.stop()
    assert all(worker.pid is None for worker in workers)
    assert manager.workers == []
    manager.stop()
