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

from dataclasses import replace
import multiprocessing
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

import psutil
import pytest
from pytest_mock import MockerFixture

from dimos.core.module import Module, ModuleConfig
from dimos.hosted.config import HostConfig
from dimos.hosted.daemon import HostDaemon, _run_fragment, _terminate
from dimos.hosted.fragment import (
    HostFragment,
    PythonFragmentPayload,
    RemoteModuleReference,
)


def _spawn_sleeper() -> None:
    subprocess.Popen(["sleep", "60"]).wait()


def test_terminate_takes_the_process_tree_down() -> None:
    process = multiprocessing.get_context("spawn").Process(target=_spawn_sleeper)
    process.start()
    root = psutil.Process(process.pid)
    deadline = time.monotonic() + 10
    while not root.children() and time.monotonic() < deadline:
        time.sleep(0.01)
    (sleeper,) = root.children()

    _terminate(process, timeout=5)

    assert not process.is_alive()
    assert not sleeper.is_running()


class DaemonModule(Module):
    pass


class RemoteProvider(Module):
    pass


class RemoteConsumer(Module):
    provider: RemoteProvider


@pytest.fixture
def host_setup(tmp_path: Path, mocker: MockerFixture) -> dict[str, Any]:
    process = mocker.MagicMock(pid=4242, exitcode=None)
    process.is_alive.return_value = True
    ready = mocker.MagicMock()
    ready.poll.return_value = True
    ready.recv.return_value = (True, None)

    context = mocker.MagicMock()
    context.Pipe.return_value = (ready, mocker.MagicMock())
    context.Process.return_value = process
    mocker.patch("dimos.hosted.daemon.multiprocessing.get_context", return_value=context)
    tree = mocker.patch("dimos.hosted.daemon.psutil.Process")
    tree.return_value.children.return_value = []

    fragment = HostFragment("run-1", "host-1", "revision-1", b"serialized blueprint")
    daemon = HostDaemon(
        HostConfig(name="lab-host"),
        host_id="host-1",
        revision="revision-1",
        tags={"gpu": ""},
        log_root=tmp_path,
    )
    return {
        "daemon": daemon,
        "fragment": fragment,
        "process": process,
        "context": context,
        "ready": ready,
        "tree": tree,
    }


def test_describe_reports_available_host(host_setup: dict[str, Any]) -> None:
    descriptor = host_setup["daemon"].describe()

    assert descriptor.host_id == "host-1"
    assert descriptor.tags["gpu"] == ""
    assert {"free_cpus", "free_ram_gb"} <= descriptor.tags.keys()
    assert descriptor.revision == "revision-1"
    assert descriptor.active_run_ids == ()


def test_sync_checks_out_when_idle_and_refuses_while_busy(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    synced: list[tuple[str, str]] = []
    with pytest.raises(RuntimeError, match="cannot sync"):
        daemon.sync("main", "abc")
    daemon._sync_checkout = lambda branch, commit: synced.append((branch, commit))

    assert daemon.sync("main", "abc") == "abc"
    assert synced == [("main", "abc")]
    daemon.start(host_setup["fragment"])
    with pytest.raises(RuntimeError, match="Busy with runs run-1"):
        daemon.sync("main", "def")
    assert synced == [("main", "abc")]


@pytest.mark.parametrize(
    ("listen", "dialed"),
    (
        ("tcp/0.0.0.0:7447", "tcp/127.0.0.1:7447"),
        ("tcp/10.0.0.5:7447", "tcp/10.0.0.5:7447"),
        ("tcp/0.0.0.0:7447#iface=lo", "tcp/127.0.0.1:7447"),
        ("iface/lo", "tcp/127.0.0.1:7447"),
        ("iface/lo:7450", "tcp/127.0.0.1:7450"),
        ("tcp/127.0.0.1:7447,tcp/0.0.0.0:7448#iface=nope", "tcp/127.0.0.1:7447"),
    ),
)
def test_fragments_dial_the_first_listener_where_it_answers(listen: str, dialed: str) -> None:
    daemon = HostDaemon(HostConfig(listen=listen), host_id="host-1", revision="r")
    assert daemon.client_endpoint == dialed


def test_iface_shorthand_becomes_a_zenoh_endpoint() -> None:
    config = HostConfig(
        listen="iface/wlan0, iface/eth0:7450, tcp/127.0.0.1:7447, tcp/0.0.0.0, tcp/[::], tcp/[::1]:7450"
    )
    assert config.listen_endpoints == [
        "tcp/0.0.0.0:7447#iface=wlan0",
        "tcp/0.0.0.0:7450#iface=eth0",
        "tcp/127.0.0.1:7447",
        "tcp/0.0.0.0:7447",
        "tcp/[::]:7447",
        "tcp/[::1]:7450",
    ]


def test_a_listener_on_an_interface_without_ipv4_is_refused() -> None:
    daemon = HostDaemon(HostConfig(listen="tcp/0.0.0.0:7447#iface=nope"), host_id="h", revision="r")
    with pytest.raises(ValueError, match="'nope' has no IPv4"):
        daemon.client_endpoint  # noqa: B018


def test_start_is_idempotent(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]

    started = daemon.start(fragment)
    repeated = daemon.start(fragment)

    assert started.state == "running"
    assert started.run_id == "run-1"
    assert repeated == started
    host_setup["context"].Process.assert_called_once()


def test_runs_multiple_blueprints_and_stops_them_independently(
    host_setup: dict[str, Any], mocker: MockerFixture
) -> None:
    daemon = host_setup["daemon"]
    first_fragment = host_setup["fragment"]
    second_fragment = replace(first_fragment, run_id="run-2", payload=b"another blueprint")
    first_process = host_setup["process"]
    second_process = mocker.MagicMock(pid=4343, exitcode=None)
    second_process.is_alive.return_value = True
    first_ready = host_setup["ready"]
    second_ready = mocker.MagicMock()
    second_ready.poll.return_value = True
    second_ready.recv.return_value = (True, None)
    first_send_ready = mocker.MagicMock()
    second_send_ready = mocker.MagicMock()
    host_setup["context"].Pipe.side_effect = [
        (first_ready, first_send_ready),
        (second_ready, second_send_ready),
    ]
    host_setup["context"].Process.side_effect = [first_process, second_process]

    first_status = daemon.start(first_fragment)
    second_status = daemon.start(second_fragment)

    assert first_status.state == "running"
    assert second_status.state == "running"
    assert daemon.describe().active_run_ids == ("run-1", "run-2")
    assert daemon.status("run-1").pid == 4242
    assert daemon.status("run-2").pid == 4343
    assert host_setup["context"].Process.call_count == 2

    first_process.is_alive.side_effect = [True, False]
    stopped = daemon.stop("run-1")

    assert stopped.state == "absent"
    assert daemon.describe().active_run_ids == ("run-2",)
    assert daemon.status("run-2").state == "running"
    first_process.terminate.assert_called_once_with()
    second_process.terminate.assert_not_called()


def test_start_validates_host(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]

    with pytest.raises(ValueError, match="targets Host"):
        daemon.start(replace(fragment, host_id="host-2"))


def test_start_validates_revision(host_setup: dict[str, Any]) -> None:
    fragment = host_setup["fragment"]
    daemon = HostDaemon(HostConfig(), host_id="host-1", revision="revision-2")

    with pytest.raises(ValueError, match="revision"):
        daemon.start(fragment)


def test_status_marks_an_exited_process_failed(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    daemon.start(fragment)
    host_setup["process"].exitcode = 7

    status = daemon.status(fragment.run_id)

    assert status.state == "failed"
    assert status.error == "Deployment process exited with code 7"
    # The corpse is gone and the error was reported once.
    assert daemon.describe().active_run_ids == ()
    assert daemon.status(fragment.run_id).state == "absent"
    host_setup["process"].exitcode = None
    assert daemon.start(fragment).state == "running"


def test_start_reports_child_failure(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    host_setup["ready"].recv.return_value = (False, "module failed")
    host_setup["process"].is_alive.return_value = False

    status = daemon.start(fragment)

    assert status.state == "failed"
    assert status.error == "module failed"
    host_setup["tree"].assert_called_once_with(4242)
    assert daemon.describe().active_run_ids == ()


def test_stop_releases_host(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    daemon.start(fragment)
    host_setup["process"].is_alive.side_effect = [True, False]

    status = daemon.stop("run-1")

    assert status.state == "absent"
    host_setup["process"].terminate.assert_called_once_with()
    host_setup["tree"].assert_called_once_with(4242)


def test_shutdown_cleans_up_all_active_deployments(
    host_setup: dict[str, Any], mocker: MockerFixture
) -> None:
    daemon = host_setup["daemon"]
    first_fragment = host_setup["fragment"]
    second_fragment = replace(first_fragment, run_id="run-2", payload=b"another blueprint")
    second_process = mocker.MagicMock(pid=4343, exitcode=None)
    second_process.is_alive.side_effect = [True, False]
    host_setup["context"].Process.side_effect = [host_setup["process"], second_process]
    host_setup["process"].is_alive.side_effect = [True, False]
    daemon.start(first_fragment)
    daemon.start(second_fragment)

    daemon.shutdown()

    assert daemon.describe().active_run_ids == ()
    host_setup["process"].terminate.assert_called_once_with()
    second_process.terminate.assert_called_once_with()
    assert host_setup["tree"].call_args_list == [mocker.call(4242), mocker.call(4343)]


def test_run_fragment_runs_the_payload_through_run_coordinator(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    blueprint = DaemonModule.blueprint()
    fragment = HostFragment.create(
        run_id="20260101-000000-abcd-daemon",
        host_id="host-1",
        revision="revision-1",
        payload=PythonFragmentPayload(blueprint=blueprint),
    )
    run = mocker.patch("dimos.hosted.daemon.run_coordinator")
    mocker.patch("dimos.hosted.daemon.set_run_log_dir")
    ready = mocker.MagicMock()

    _run_fragment(fragment, tmp_path, ready, router="tcp/127.0.0.1:7447")

    built_blueprint, built_config = run.call_args.args
    built_config.assert_matches(built_blueprint)
    assert built_config.global_config["zenoh_domain"] == "20260101-000000-abcd-daemon"
    assert built_config.global_config["zenoh_mode"] == "client"
    assert run.call_args.kwargs["run_id"] == fragment.run_id
    assert run.call_args.kwargs["log_dir"] == tmp_path
    assert run.call_args.kwargs["rpc_name"] == f"runs/{fragment.run_id}/hosts/host-1/coordinator"
    assert run.call_args.kwargs["remote_module_refs"] == {}
    run.call_args.kwargs["on_ready"](None)
    ready.send.assert_called_once_with((True, None))
    ready.close.assert_called_once_with()


def test_run_fragment_reports_a_failed_start(tmp_path: Path, mocker: MockerFixture) -> None:
    blueprint = DaemonModule.blueprint()
    fragment = HostFragment.create(
        run_id="run-1",
        host_id="host-1",
        revision="revision-1",
        payload=PythonFragmentPayload(blueprint=blueprint),
    )
    mocker.patch("dimos.hosted.daemon.run_coordinator", side_effect=RuntimeError("no worker"))
    mocker.patch("dimos.hosted.daemon.set_run_log_dir")
    ready = mocker.MagicMock()

    # The error is reported on the pipe and still unwinds, so the process exits non-zero.
    with pytest.raises(RuntimeError, match="no worker"):
        _run_fragment(fragment, tmp_path, ready)

    ready.send.assert_called_once_with((False, "no worker"))
    ready.close.assert_called_once_with()


def test_run_fragment_passes_remote_module_targets_to_coordinator(
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    blueprint = RemoteConsumer.blueprint()
    rpc_name = "runs/run-1/hosts/host-a/modules/remoteprovider"
    fragment = HostFragment.create(
        run_id="run-1",
        host_id="host-b",
        revision="revision-1",
        payload=PythonFragmentPayload(
            blueprint=blueprint,
            remote_module_references=(
                RemoteModuleReference(
                    consumer_name="remoteconsumer",
                    reference_name="provider",
                    provider_name="remoteprovider",
                    provider_host_id="host-a",
                    rpcs=frozenset(RemoteProvider.rpcs),
                    rpc_name=rpc_name,
                ),
            ),
        ),
    )
    run = mocker.patch("dimos.hosted.daemon.run_coordinator")
    mocker.patch("dimos.hosted.daemon.set_run_log_dir")

    _run_fragment(fragment, tmp_path, mocker.MagicMock())

    assert [a.module for a in run.call_args.args[0].blueprints] == [RemoteConsumer]
    assert run.call_args.kwargs["remote_module_refs"] == {
        ("remoteconsumer", "provider"): (frozenset(RemoteProvider.rpcs), rpc_name),
    }


class HostConfigured(Module):
    config: "HostConfiguredConfig"


class HostConfiguredConfig(ModuleConfig):
    mount: str = "SF"
    iface: str = "eth0"


def test_fragment_config_is_host_env_then_root_values_then_pins(
    tmp_path: Path, mocker: MockerFixture, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("HOSTCONFIGURED__MOUNT", "ATHENS")
    monkeypatch.setenv("HOSTCONFIGURED__IFACE", "wlan0")
    monkeypatch.setenv("TRANSPORT", "lcm")
    fragment = HostFragment.create(
        run_id="run-1",
        host_id="host-1",
        revision="revision-1",
        payload=PythonFragmentPayload(
            blueprint=HostConfigured.blueprint(),
            overrides={"hostconfigured": {"iface": "eth1"}},
            global_overrides={"viewer": "none"},
        ),
    )
    run = mocker.patch("dimos.hosted.daemon.run_coordinator")
    mocker.patch("dimos.hosted.daemon.set_run_log_dir")

    _run_fragment(fragment, tmp_path, mocker.MagicMock())

    config = run.call_args.args[1]
    assert config.module_kwargs("hostconfigured") == {"mount": "ATHENS", "iface": "eth1"}
    assert config.global_config["viewer"] == "none"
    assert config.global_config["transport"] == "zenoh"


def _free_port() -> int:
    import socket

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def test_a_run_stops_once_its_root_is_gone(host_setup: dict[str, Any], tmp_path: Path) -> None:
    from dimos.hosted.protocol import RUN_LIVELINESS_KEY
    from dimos.protocol.rpc.zenohrpc import ZenohRPC
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    daemon = HostDaemon(
        HostConfig(
            name="lab",
            listen=f"tcp/127.0.0.1:{_free_port()}",
            autodiscovery=False,
            run_loss_grace=0.2,
        ),
        host_id="host-1",
        revision="revision-1",
        log_root=tmp_path,
    )
    fragment = host_setup["fragment"]
    process = host_setup["process"]
    with daemon.serve():
        pool = ZenohSessionPool()
        root = ZenohRPC(
            session_pool=pool, mode="client", connect=[daemon.client_endpoint], multicast=False
        )
        root.start()
        # Held for the block: a dropped token is undeclared at once.
        token = root.session.liveliness().declare_token(RUN_LIVELINESS_KEY.format(run_id="run-1"))
        assert daemon.start(fragment).state == "running"
        time.sleep(0.5)
        assert daemon._root_alive("run-1")
        assert daemon.describe().active_run_ids == ("run-1",)
        process.is_alive.side_effect = [True, False]

        # The blueprint root dies without stopping anything.
        del token
        root.stop()
        pool.close_all()

        deadline = time.monotonic() + 10
        while daemon.describe().active_run_ids and time.monotonic() < deadline:
            time.sleep(0.05)
        assert daemon.describe().active_run_ids == ()
        process.terminate.assert_called_once_with()


def test_startup_timeout_comes_from_the_host_config(
    host_setup: dict[str, Any], tmp_path: Path
) -> None:
    daemon = HostDaemon(
        HostConfig(name="lab-host", startup_timeout=0.01),
        host_id="host-1",
        revision="revision-1",
        log_root=tmp_path,
    )
    host_setup["ready"].poll.return_value = False
    host_setup["process"].is_alive.side_effect = [True, False]

    status = daemon.start(host_setup["fragment"])

    host_setup["ready"].poll.assert_called_once_with(0.01)
    assert status.state == "failed"
    assert status.error == "Deployment startup timed out after 0.01s"


def test_shutdown_terminates_deployments_side_by_side(
    host_setup: dict[str, Any], mocker: MockerFixture
) -> None:
    daemon = host_setup["daemon"]
    second_fragment = replace(host_setup["fragment"], run_id="run-2", payload=b"b")
    second_process = mocker.MagicMock(pid=4343, exitcode=None)
    host_setup["context"].Process.side_effect = [host_setup["process"], second_process]
    # Both joins must be in flight at once, or the barrier breaks.
    both_stopping = threading.Barrier(2, timeout=5)
    for process in (host_setup["process"], second_process):
        process.is_alive.side_effect = [True, False]
        process.join.side_effect = lambda timeout=None: both_stopping.wait()
    daemon.start(host_setup["fragment"])
    daemon.start(second_fragment)

    daemon.shutdown()

    assert daemon.describe().active_run_ids == ()
    assert not both_stopping.broken


def test_a_watch_tick_reaps_an_exited_fragment(host_setup: dict[str, Any]) -> None:
    daemon = host_setup["daemon"]
    fragment = host_setup["fragment"]
    daemon.start(fragment)
    host_setup["process"].exitcode = 3

    daemon._watch_tick({})

    assert daemon.describe().active_run_ids == ()
    assert daemon.status(fragment.run_id).error == "Deployment process exited with code 3"


def test_a_run_stops_once_its_root_is_gone_across_linked_routers(
    host_setup: dict[str, Any], tmp_path: Path
) -> None:
    from dimos.hosted.protocol import RUN_LIVELINESS_KEY
    from dimos.protocol.rpc.zenohrpc import ZenohRPC
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    edge_port = _free_port()
    edge = HostDaemon(
        HostConfig(
            name="edge",
            listen=f"tcp/127.0.0.1:{edge_port}",
            autodiscovery=False,
            run_loss_grace=0.2,
        ),
        host_id="host-1",
        revision="revision-1",
        log_root=tmp_path / "edge",
    )
    base = HostDaemon(
        HostConfig(
            name="base",
            listen=f"tcp/127.0.0.1:{_free_port()}",
            connect=f"tcp/127.0.0.1:{edge_port}",
            autodiscovery=False,
        ),
        host_id="base-id",
        revision="revision-1",
        log_root=tmp_path / "base",
    )
    fragment = host_setup["fragment"]
    process = host_setup["process"]
    with edge.serve(), base.serve():
        pool = ZenohSessionPool()
        root = ZenohRPC(
            session_pool=pool, mode="client", connect=[base.client_endpoint], multicast=False
        )
        root.start()
        token = root.session.liveliness().declare_token(RUN_LIVELINESS_KEY.format(run_id="run-1"))
        # The edge Host sees the blueprint root's token through the base router.
        deadline = time.monotonic() + 10
        while not edge._root_alive("run-1") and time.monotonic() < deadline:
            time.sleep(0.05)
        assert edge._root_alive("run-1")
        assert edge.start(fragment).state == "running"
        time.sleep(0.5)
        assert edge.describe().active_run_ids == ("run-1",)
        process.is_alive.side_effect = [True, False]

        del token
        root.stop()
        pool.close_all()

        deadline = time.monotonic() + 10
        while edge.describe().active_run_ids and time.monotonic() < deadline:
            time.sleep(0.05)
        assert edge.describe().active_run_ids == ()
        process.terminate.assert_called_once_with()


def _hold_ready(ready: Any) -> None:
    """A fragment that never reports ready; it keeps the pipe so the Host waits on it."""
    time.sleep(60)
    ready.close()


def test_a_shutdown_during_a_start_in_flight_is_prompt_and_leaves_no_fragment(
    tmp_path: Path,
) -> None:
    from types import SimpleNamespace

    from dimos.hosted import machine
    from dimos.hosted.client import HostClient
    from dimos.protocol.rpc.zenohrpc import ZenohRPC
    from dimos.protocol.service.zenohservice import ZenohSessionPool

    daemon = HostDaemon(
        HostConfig(
            name="lab",
            listen=f"tcp/127.0.0.1:{_free_port()}",
            autodiscovery=False,
            startup_timeout=20.0,
            stop_timeout=5.0,
        ),
        host_id="host-1",
        revision="revision-1",
        log_root=tmp_path,
    )
    context = multiprocessing.get_context("spawn")
    daemon._process_context = SimpleNamespace(  # type: ignore[assignment]
        Pipe=context.Pipe,
        Process=lambda target, args, daemon: context.Process(
            target=_hold_ready, args=(args[2],), daemon=daemon
        ),
    )
    fragment = HostFragment("run-1", "host-1", "revision-1", b"x")
    outcome: dict[str, Any] = {}
    pool = ZenohSessionPool()
    with daemon.serve():
        root = ZenohRPC(
            session_pool=pool, mode="client", connect=[daemon.client_endpoint], multicast=False
        )
        root.start()
        client = HostClient(root, "host-1", timeout=60.0)
        starter = threading.Thread(target=lambda: outcome.update(status=client.start(fragment)))
        starter.start()
        deadline = time.monotonic() + 10
        while daemon.status("run-1").state != "starting" and time.monotonic() < deadline:
            time.sleep(0.05)
        process = daemon._deployments["run-1"].process
        assert process.is_alive()
        left_at = time.monotonic()

    assert time.monotonic() - left_at < machine.STOP_TIMEOUT / 4
    assert not process.is_alive()
    assert daemon.describe().active_run_ids == ()
    starter.join(15.0)
    assert outcome["status"].state == "absent"
    root.stop()
    pool.close_all()
