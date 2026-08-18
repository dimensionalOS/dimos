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

"""Tests for pickle-safe dynamically generated relay bridge classes."""

# No `from __future__ import annotations` here on purpose: under PEP 563 a
# Module class whose annotations become strings silently loses its streams
# (see the module-level note in test_relay_bridge_module.py).

import pickle
import re
import subprocess
import sys
import threading

import pytest

from dimos.core.coordination.blueprints import BlueprintAtom, StreamRef, autoconnect
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module, PeekNotFound
from dimos.core.stream import In, Out, RemoteIn
from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.relay_bridge import dynamic
from dimos.web.relay_bridge.dynamic import DynamicPortSpec, make_relay_bridge_class
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeConfig

# Marker message types: distinct classes so a shape differing only by type
# gets a distinct generated class.


class _MarkerA:
    pass


class _MarkerB:
    pass


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


@pytest.fixture
def local_modules():
    """In-process module instances; stop_module also drains the asyncio
    default executor so the thread-leak check stays clean."""
    modules = []
    yield modules
    for module in reversed(modules):
        stop_module(module)


def test_class_name_is_deterministic_and_importable():
    generated = make_relay_bridge_class(_MAIN_SPECS)
    again = make_relay_bridge_class(tuple(reversed(_MAIN_SPECS)))
    assert re.fullmatch(r"DynamicRelayBridge_[0-9a-f]{10}", generated.__name__)
    assert generated.__module__ == "dimos.web.relay_bridge.dynamic"
    assert generated.__qualname__ == generated.__name__
    assert again.__name__ == generated.__name__


def test_memoization_and_canonical_order():
    generated = make_relay_bridge_class(_MAIN_SPECS)
    again = make_relay_bridge_class(tuple(reversed(_MAIN_SPECS)))
    assert again is generated
    assert generated.dynamic_port_specs == tuple(sorted(_MAIN_SPECS, key=lambda s: s.stream))


def test_pickle_roundtrip_returns_identical_class_object():
    generated = make_relay_bridge_class(_MAIN_SPECS)
    assert pickle.loads(pickle.dumps(generated)) is generated


def test_two_shapes_coexist_in_one_process():
    shape_a = make_relay_bridge_class([DynamicPortSpec("marker_feed", _MarkerA, "rx")])
    shape_b = make_relay_bridge_class([DynamicPortSpec("marker_feed", _MarkerB, "rx")])
    assert shape_a is not shape_b
    assert shape_a.__name__ != shape_b.__name__
    assert pickle.loads(pickle.dumps(shape_a)) is shape_a
    assert pickle.loads(pickle.dumps(shape_b)) is shape_b


def test_generated_class_is_published_for_reload_lookup():
    generated = make_relay_bridge_class(_MAIN_SPECS)
    # The coordinator's restart path reloads the module and looks the class
    # up again by name; the factory publishes it for exactly that.
    assert getattr(sys.modules["dimos.web.relay_bridge.dynamic"], generated.__name__) is generated


def test_republish_after_memo_reset_returns_published_class(monkeypatch):
    generated = make_relay_bridge_class([DynamicPortSpec("memo_reset_feed", Vector3, "rx")])
    # A reload of dynamic.py rebinds _memo to a fresh dict while published
    # classes survive in the module namespace; identity must be restored.
    monkeypatch.setattr(dynamic, "_memo", {})
    again = make_relay_bridge_class([DynamicPortSpec("memo_reset_feed", Vector3, "rx")])
    assert again is generated


def test_digest_name_collision_with_different_specs_raises(monkeypatch):
    shape_a = make_relay_bridge_class([DynamicPortSpec("collide_a", Vector3, "rx")])
    shape_b = make_relay_bridge_class([DynamicPortSpec("collide_b", Vector3, "rx")])
    monkeypatch.setattr(dynamic, "_memo", {})
    monkeypatch.setattr(dynamic, shape_a.__name__, shape_b)
    with pytest.raises(ValueError, match="digest collision"):
        make_relay_bridge_class([DynamicPortSpec("collide_a", Vector3, "rx")])


_RELOAD_SCRIPT = """
import importlib, pickle, sys
from dimos.msgs.geometry_msgs.Vector3 import Vector3
import dimos.web.relay_bridge.dynamic as dynamic

C = dynamic.make_relay_bridge_class([dynamic.DynamicPortSpec("reload_feed", Vector3, "rx")])
data = pickle.dumps(C)
mod = importlib.reload(sys.modules["dimos.web.relay_bridge.dynamic"])
assert getattr(mod, C.__name__) is C, "restart getattr lost the class"
assert pickle.loads(data) is C, "pre-reload pickle lost identity"
assert pickle.loads(pickle.dumps(C)) is C, "re-pickle after reload failed"
again = mod.make_relay_bridge_class([mod.DynamicPortSpec("reload_feed", Vector3, "rx")])
assert again is C, "factory lost identity after reload"
"""


def test_reload_then_repickle_preserves_identity():
    """The coordinator's restart path reloads dynamic.py and then re-deploys
    the surviving class object, which re-pickles it. Runs in a subprocess so
    the reload cannot poison this process's module state."""
    result = subprocess.run(
        [sys.executable, "-c", _RELOAD_SCRIPT], capture_output=True, text=True, timeout=120
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    ("specs", "expected", "match"),
    [
        ([DynamicPortSpec("@control", Vector3, "rx")], ValueError, "reserved"),
        ([DynamicPortSpec("not valid", Vector3, "rx")], ValueError, "identifier"),
        ([DynamicPortSpec("class", Vector3, "rx")], ValueError, "identifier"),
        ([DynamicPortSpec("_hidden", Vector3, "rx")], ValueError, "underscore"),
        ([DynamicPortSpec("x" * 65, Vector3, "rx")], ValueError, "longer than"),
        ([DynamicPortSpec("color_image", Vector3, "rx")], ValueError, "collides"),
        ([DynamicPortSpec("ref", Vector3, "rx")], ValueError, "collides"),
        ([DynamicPortSpec("peek_stream", Vector3, "rx")], ValueError, "collides"),
        (
            [DynamicPortSpec("dup", Vector3, "rx"), DynamicPortSpec("dup", str, "tx")],
            ValueError,
            "duplicate",
        ),
        ([DynamicPortSpec("bad_dir", Vector3, "sideways")], ValueError, "direction"),
        ([DynamicPortSpec("not_a_class", 42, "rx")], TypeError, "must be a class"),
        ([DynamicPortSpec("generic_alias", list[int], "rx")], TypeError, "must be a class"),
    ],
)
def test_invalid_specs_rejected(specs, expected, match):
    with pytest.raises(expected, match=match):
        make_relay_bridge_class(specs)


def test_main_module_message_type_rejected():
    main_only = type("MainOnly", (), {"__module__": "__main__"})
    with pytest.raises(ValueError, match="__main__"):
        make_relay_bridge_class([DynamicPortSpec("main_feed", main_only, "rx")])


def test_function_local_message_type_rejected():
    class LocalType:
        pass

    with pytest.raises(ValueError, match="inside a function"):
        make_relay_bridge_class([DynamicPortSpec("local_feed", LocalType, "rx")])


def test_unimportable_message_type_rejected():
    ghost = type("GhostType", (), {"__module__": __name__})
    with pytest.raises(ValueError, match="pickled by reference"):
        make_relay_bridge_class([DynamicPortSpec("ghost_feed", ghost, "rx")])


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


def test_blueprint_pickle_preserves_generated_class_identity():
    generated = make_relay_bridge_class(_AUTOCONNECT_SPECS)
    blueprint = autoconnect(
        _PingProducer.blueprint(), generated.blueprint(), _NoteConsumer.blueprint()
    )
    restored = pickle.loads(pickle.dumps(blueprint))
    assert restored == blueprint
    restored_atom = next(a for a in restored.blueprints if a.name == generated.name)
    assert restored_atom.module is generated


def test_module_init_constructs_runtime_streams(local_modules):
    generated = make_relay_bridge_class(_MAIN_SPECS)
    module = generated()
    local_modules.append(module)
    assert sorted(module.inputs) == ["color_image", "global_costmap", "odom", "sensor_ping"]
    assert sorted(module.outputs) == ["operator_note", "tele_cmd_vel"]
    assert isinstance(module.sensor_ping, In)
    assert module.sensor_ping.type is Vector3
    assert isinstance(module.operator_note, Out)
    assert module.operator_note.type is str
    assert isinstance(module.config, RelayBridgeConfig)
    assert module.config.relay_url is None


@pytest.mark.skipif_macos_bug
def test_forkserver_worker_predates_class_deploy_rpc_and_stream(
    worker_manager, deployed_proxies, ping_transport, retry_until
):
    """The W5 flagship proof: the worker process exists before the class does,
    so the class can only arrive in the worker through pickle, not fork
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
    got = threading.Event()

    def peek():
        value = proxy.peek_stream("fork_ping", 15.0)
        if value is not None:
            received.append(value)
        got.set()

    peeker = threading.Thread(target=peek)
    peeker.start()
    try:
        retry_until(got, lambda: ping_transport.publish(Vector3(1.0, 2.0, 3.0)), timeout=10.0)
    finally:
        peeker.join(timeout=20.0)
    assert received == [Vector3(1.0, 2.0, 3.0)]


@pytest.mark.skipif_macos_bug
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


@pytest.mark.skipif_macos_bug
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


@pytest.mark.skipif_macos_bug
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
