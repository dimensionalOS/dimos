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

"""Docker Compose scenarios for real multi-Host placement validation."""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import AsyncIterator
from contextlib import suppress
from dataclasses import dataclass
from importlib.metadata import version as package_version
import json
import math
import os
from pathlib import Path
import signal
import socket
import threading
import time
from typing import Any, Literal, cast
from urllib.parse import quote

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hosted.client import HostClient, discover_hosts
from dimos.hosted.daemon import (
    HOST_CONTROL_RPC_NAME,
    HOST_LIVELINESS_KEY,
    HOST_PROTOCOL_VERSION,
    HostDaemon,
    HostDescriptor,
)
from dimos.hosted.fragment import FRAGMENT_SCHEMA_VERSION, HostFragment
from dimos.hosted.fragment_compiler import compile_fragments
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.std_msgs.String import String
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service.zenohservice import ZenohSessionPool
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT, RERUN_WEB_VIEWER_PORT

ScenarioName = Literal["basic", "replay", "sim", "visual"]

DEFAULT_DISCOVERY_TIMEOUT = 30.0
DEFAULT_RESULT_TIMEOUT = 30.0
RPC_TIMEOUT = 90.0
ROUTER_LISTEN = "tcp/0.0.0.0:7447"
RESULT_ROOT = Path("/tmp/dimos-hosted-validation")
SIM_MODEL = Path("/app/experimental/hosted/compose/free_body.xml")
VISUAL_RERUN_SOURCE = f"rerun+http://localhost:{RERUN_GRPC_PORT}/proxy"
VISUAL_VIEWER_URL = (
    f"http://localhost:{RERUN_WEB_VIEWER_PORT}/?url={quote(VISUAL_RERUN_SOURCE, safe='')}"
)


class BasicSourceConfig(ModuleConfig):
    interval_seconds: float = 0.05


class BasicSource(Module):
    """Continuously publish a known value so late subscribers can recover."""

    config: BasicSourceConfig
    basic_events: Out[String]

    async def main(self) -> AsyncIterator[None]:
        publisher = asyncio.create_task(self._publish())
        try:
            yield
        finally:
            publisher.cancel()
            with suppress(asyncio.CancelledError):
                await publisher

    async def _publish(self) -> None:
        while True:
            self.basic_events.publish(String("basic:ready"))
            await asyncio.sleep(self.config.interval_seconds)


class BasicTransform(Module):
    basic_events: In[String]
    validation_events: Out[String]

    async def handle_basic_events(self, message: String) -> None:
        self.validation_events.publish(String(message.data.upper()))


class ReplaySourceConfig(ModuleConfig):
    database_path: str
    speed: float = 20.0


class ReplaySource(Module):
    """Publish a real SQLite-backed DimOS Replay over a hosted boundary."""

    config: ReplaySourceConfig
    replay_events: Out[String]

    async def main(self) -> AsyncIterator[None]:
        store = SqliteStore(path=self.config.database_path)
        subscription = None
        try:
            stream = store.stream("validation", str)
            for index, value in enumerate(("alpha", "beta", "gamma")):
                stream.append(value, ts=1000.0 + index * 0.05)
            replay = store.replay(speed=self.config.speed, loop=True)
            subscription = (
                replay.stream("validation")
                .observable()
                .subscribe(lambda value: self.replay_events.publish(String(value)))
            )
            yield
        finally:
            if subscription is not None:
                subscription.dispose()
            store.stop()


class ReplayTransform(Module):
    replay_events: In[String]
    validation_events: Out[String]

    async def handle_replay_events(self, message: String) -> None:
        self.validation_events.publish(String(f"REPLAY:{message.data.upper()}"))


class SimOdomProbe(Module):
    odom: In[PoseStamped]
    validation_events: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._reported = False

    async def handle_odom(self, _message: PoseStamped) -> None:
        if self._reported:
            return
        self._reported = True
        self.validation_events.publish(String("SIM:ODOM"))


def _circle_pose(
    *, elapsed_seconds: float, radius: float, period_seconds: float
) -> tuple[float, float, float]:
    """Return position and tangential yaw for a constant-speed circle."""
    phase = math.tau * (elapsed_seconds % period_seconds) / period_seconds
    return (
        radius * math.cos(phase),
        radius * math.sin(phase),
        phase + math.pi / 2.0,
    )


class MovingMujocoSimModule(MujocoSimModule):
    """Visual-demo simulator whose real root state follows a smooth circle."""

    _MOTION_RADIUS = 1.0
    _MOTION_PERIOD_SECONDS = 8.0
    _MOTION_Z = 0.18

    def _publish_shm_and_lcm(self, engine: MujocoEngine) -> None:
        qpos_adr = self._root_base_qpos_adr
        if qpos_adr is not None:
            data = engine.data
            x, y, yaw = _circle_pose(
                elapsed_seconds=float(data.time),
                radius=self._MOTION_RADIUS,
                period_seconds=self._MOTION_PERIOD_SECONDS,
            )
            data.qpos[qpos_adr : qpos_adr + 7] = [
                x,
                y,
                self._MOTION_Z,
                math.cos(yaw * 0.5),
                0.0,
                0.0,
                math.sin(yaw * 0.5),
            ]

            qvel_adr = engine.root_qvel_adr
            if qvel_adr is not None:
                angular_speed = math.tau / self._MOTION_PERIOD_SECONDS
                data.qvel[qvel_adr : qvel_adr + 6] = [
                    -self._MOTION_RADIUS * angular_speed * math.sin(yaw - math.pi / 2.0),
                    self._MOTION_RADIUS * angular_speed * math.cos(yaw - math.pi / 2.0),
                    0.0,
                    0.0,
                    0.0,
                    angular_speed,
                ]

        super()._publish_shm_and_lcm(engine)


def _visual_rerun_blueprint() -> Any:
    """Keep the browser focused on the simulated world."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="Hosted MuJoCo scene"),
        collapse_panels=True,
    )


def _visual_scene_static(rerun_module: Any) -> list[tuple[str, Any]]:
    """Create lightweight geometry that follows the real MuJoCo odometry."""
    return [
        (
            "world/ground",
            rerun_module.Boxes3D(
                centers=[[0.0, 0.0, -0.03]],
                half_sizes=[[3.0, 3.0, 0.03]],
                colors=[[55, 65, 75, 255]],
            ),
        ),
        ("world", rerun_module.ViewCoordinates.RIGHT_HAND_Z_UP),
        (
            "world/odom/robot/body",
            rerun_module.Boxes3D(
                centers=[[0.0, 0.0, 0.0]],
                half_sizes=[[0.28, 0.18, 0.12]],
                colors=[[40, 145, 255, 255]],
            ),
        ),
        (
            "world/odom/robot/front",
            rerun_module.Arrows3D(
                origins=[[0.0, 0.0, 0.0]],
                vectors=[[0.55, 0.0, 0.0]],
                colors=[[255, 195, 40, 255]],
                radii=0.025,
            ),
        ),
        (
            "world/target",
            rerun_module.Points3D(
                positions=[[1.5, 0.0, 0.12]],
                colors=[[255, 80, 90, 255]],
                radii=0.12,
                labels=["target"],
            ),
        ),
    ]


def _visual_topic_to_entity(topic: Any) -> str:
    """Collapse run-scoped hosted odometry onto the visual robot transform."""
    raw_topic = getattr(topic, "topic", None) or getattr(topic, "name", None) or str(topic)
    topic_name = str(raw_topic).split("#", 1)[0]
    if topic_name.endswith("/odom"):
        return "world/odom"

    if topic_name.startswith("dimos/"):
        topic_name = topic_name.removeprefix("dimos/")
    return f"world/{topic_name.lstrip('/')}"


class ValidationSinkConfig(ModuleConfig):
    scenario: str
    expected_values: list[str]
    result_path: str


class ValidationSink(Module):
    config: ValidationSinkConfig
    validation_events: In[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._seen: set[str] = set()
        self._complete = False

    async def handle_validation_events(self, message: String) -> None:
        if self._complete:
            return
        self._seen.add(message.data)
        expected = set(self.config.expected_values)
        if not expected.issubset(self._seen):
            return

        self._complete = True
        path = Path(self.config.result_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(
                {
                    "scenario": self.config.scenario,
                    "sink_host": socket.gethostname(),
                    "values": sorted(self._seen),
                },
                sort_keys=True,
            )
        )
        temporary.replace(path)


@dataclass(frozen=True)
class Scenario:
    name: ScenarioName
    blueprint: Blueprint
    result_path: Path
    expected_hosts: dict[str, str]
    producer_modules: frozenset[str]
    keep_running: bool = False
    viewer_url: str | None = None


def build_scenario(name: ScenarioName, run_id: str) -> Scenario:
    """Build one deterministic scenario without opening runtime resources."""
    result_path = RESULT_ROOT / f"{run_id}.json"
    sink = ValidationSink.blueprint(
        scenario=name,
        expected_values={
            "basic": ["BASIC:READY"],
            "replay": ["REPLAY:ALPHA", "REPLAY:BETA", "REPLAY:GAMMA"],
            "sim": ["SIM:ODOM"],
            "visual": ["SIM:ODOM"],
        }[name],
        result_path=str(result_path),
    )

    if name == "basic":
        blueprint = autoconnect(
            BasicSource.blueprint().hosted(host="edge-a"),
            BasicTransform.blueprint().hosted(tags={"compute"}),
            sink,
        )
        expected_hosts = {
            BasicSource.name: "edge-a",
            BasicTransform.name: "compute-a",
            ValidationSink.name: "controller",
        }
        producers = frozenset({BasicSource.name})
    elif name == "replay":
        blueprint = autoconnect(
            ReplaySource.blueprint(
                database_path=f"/tmp/{run_id}-replay.db",
            ).hosted(tags={"replay"}),
            ReplayTransform.blueprint().hosted(tags={"compute"}),
            sink,
        )
        expected_hosts = {
            ReplaySource.name: "replay-a",
            ReplayTransform.name: "compute-a",
            ValidationSink.name: "controller",
        }
        producers = frozenset({ReplaySource.name})
    else:
        simulator = MovingMujocoSimModule if name == "visual" else MujocoSimModule
        modules = [
            simulator.blueprint(
                address=str(SIM_MODEL),
                headless=True,
                dof=0,
                enable_color=False,
                enable_depth=False,
            ).hosted(tags={"sim"}),
            SimOdomProbe.blueprint().hosted(tags={"compute"}),
            sink,
        ]
        if name == "visual":
            modules.append(
                RerunBridgeModule.blueprint(
                    pubsubs=[Zenoh()],
                    connect_url=f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy",
                    rerun_open="none",
                    rerun_web=True,
                    web_port=RERUN_WEB_VIEWER_PORT,
                    blueprint=_visual_rerun_blueprint,
                    static={"world/scene": _visual_scene_static},
                    topic_to_entity=_visual_topic_to_entity,
                    max_hz={"world/odom": 30.0},
                )
            )
        blueprint = autoconnect(*modules)
        expected_hosts = {
            simulator.name: "sim-a",
            SimOdomProbe.name: "compute-a",
            ValidationSink.name: "controller",
        }
        if name == "visual":
            expected_hosts[RerunBridgeModule.name] = "controller"
        producers = frozenset({simulator.name})

    return Scenario(
        name,
        blueprint,
        result_path,
        expected_hosts,
        producers,
        keep_running=name == "visual",
        viewer_url=VISUAL_VIEWER_URL if name == "visual" else None,
    )


def _wait_for_hosts(
    rpc: ZenohRPC,
    required_names: set[str],
    timeout: float,
) -> tuple[HostDescriptor, ...]:
    deadline = time.monotonic() + timeout
    last_descriptors: tuple[HostDescriptor, ...] = ()
    while time.monotonic() < deadline:
        try:
            last_descriptors = discover_hosts(rpc, timeout=min(1.0, timeout))
        except TimeoutError:
            last_descriptors = ()
        names = [descriptor.name for descriptor in last_descriptors]
        duplicates = {name for name in names if names.count(name) > 1}
        if duplicates:
            raise RuntimeError(f"Duplicate Host names: {', '.join(sorted(duplicates))}")
        if required_names.issubset(names):
            return last_descriptors
        threading.Event().wait(0.1)

    found = ", ".join(sorted(descriptor.name for descriptor in last_descriptors)) or "none"
    missing = ", ".join(sorted(required_names - {d.name for d in last_descriptors}))
    raise TimeoutError(f"Timed out discovering Hosts; found={found}; missing={missing}")


def _fragment_module_names(fragment: HostFragment) -> frozenset[str]:
    return frozenset(atom.name for atom in fragment.load_payload().blueprint.active_blueprints)


def _check_placement(
    scenario: Scenario,
    fragments: dict[str, HostFragment],
    descriptors: tuple[HostDescriptor, ...],
    local_host_id: str,
) -> dict[str, str]:
    host_names = {descriptor.host_id: descriptor.name for descriptor in descriptors}
    host_names[local_host_id] = "controller"
    actual = {
        module_name: host_names[host_id]
        for host_id, fragment in fragments.items()
        for module_name in _fragment_module_names(fragment)
    }
    if actual != scenario.expected_hosts:
        raise RuntimeError(
            f"Unexpected placement for {scenario.name}: expected={scenario.expected_hosts}, "
            f"actual={actual}"
        )
    return actual


def _wait_for_result(path: Path, scenario: ScenarioName, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.exists():
            result = cast("dict[str, Any]", json.loads(path.read_text()))
            if result.get("scenario") != scenario:
                raise RuntimeError(f"Result belongs to the wrong scenario: {result}")
            return result
        threading.Event().wait(0.1)
    raise TimeoutError(f"Timed out waiting for scenario result: {path}")


def _start_priority(
    host_id: str,
    fragment: HostFragment,
    local_host_id: str,
    producers: frozenset[str],
) -> tuple[int, str]:
    names = _fragment_module_names(fragment)
    if host_id == local_host_id:
        return (0, host_id)
    if names.intersection(producers):
        return (2, host_id)
    return (1, host_id)


def _wait_for_visual_shutdown(viewer_url: str) -> None:
    """Keep a visual scenario alive until Compose asks it to stop."""
    stopped = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopped.set()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    print(f"VISUAL:READY open {viewer_url} (Ctrl-C to stop)", flush=True)
    try:
        stopped.wait()
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


def run_scenario(
    name: ScenarioName,
    *,
    discovery_timeout: float = DEFAULT_DISCOVERY_TIMEOUT,
    result_timeout: float = DEFAULT_RESULT_TIMEOUT,
) -> dict[str, Any]:
    """Discover Hosts, auto-place a scenario, run it, verify data, and stop it."""
    run_id = f"compose-{name}-{int(time.time() * 1000)}"
    revision = package_version("dimos")
    scenario = build_scenario(name, run_id)
    scenario.result_path.unlink(missing_ok=True)
    local_host_id = f"controller-{run_id}"
    required_names = set(scenario.expected_hosts.values()) - {"controller"}

    pool = ZenohSessionPool()
    rpc = ZenohRPC(session_pool=pool)
    local_daemon = HostDaemon(
        local_host_id,
        name="controller",
        tags={"controller"},
        versions={"application_revision": revision},
        log_root=RESULT_ROOT / "logs",
        startup_timeout=RPC_TIMEOUT,
    )
    started: list[tuple[str, HostFragment]] = []
    clients: dict[str, HostClient] = {}
    local_epoch = local_daemon.describe().epoch
    rpc.start()
    try:
        descriptors = _wait_for_hosts(rpc, required_names, discovery_timeout)
        clients = {
            descriptor.host_id: HostClient(rpc, descriptor, timeout=RPC_TIMEOUT)
            for descriptor in descriptors
        }
        config = BlueprintConfigParser(scenario.blueprint).parse(environ=os.environ)
        fragments = compile_fragments(
            scenario.blueprint,
            config,
            run_id=run_id,
            generation=1,
            application_name=f"hosted-compose-{name}",
            application_revision=revision,
            hosts=descriptors,
            local_host_id=local_host_id,
        )
        placement = _check_placement(scenario, fragments, descriptors, local_host_id)

        ordered = sorted(
            fragments.items(),
            key=lambda item: _start_priority(
                item[0], item[1], local_host_id, scenario.producer_modules
            ),
        )
        for host_id, fragment in ordered:
            status = (
                local_daemon.start(local_epoch, fragment)
                if host_id == local_host_id
                else clients[host_id].start(fragment)
            )
            if status.state != "running":
                raise RuntimeError(f"Host {host_id} failed to start: {status}")
            started.append((host_id, fragment))

        result = _wait_for_result(scenario.result_path, name, result_timeout)
        report = {
            "scenario": name,
            "run_id": run_id,
            "placement": placement,
            "result": result,
        }
        if scenario.viewer_url is not None:
            report["viewer_url"] = scenario.viewer_url
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        if scenario.keep_running:
            assert scenario.viewer_url is not None
            _wait_for_visual_shutdown(scenario.viewer_url)
        return report
    finally:
        for host_id, fragment in reversed(started):
            try:
                if host_id == local_host_id:
                    stop_status = local_daemon.stop(
                        local_epoch,
                        fragment.run_id,
                        fragment.generation,
                        fragment.payload_digest,
                    )
                else:
                    stop_status = clients[host_id].stop(fragment)
                if stop_status.state != "available":
                    print(
                        f"Host {host_id} stopped in state {stop_status.state}",
                        flush=True,
                    )
            except Exception as error:
                print(f"Failed to stop Host {host_id}: {error}", flush=True)
        local_daemon.shutdown()
        rpc.stop()
        pool.close_all()


def run_router() -> None:
    """Run a dedicated Zenoh router using the same library version as DimOS."""
    pool = ZenohSessionPool()
    rpc = ZenohRPC(
        session_pool=pool,
        mode="router",
        listen=[ROUTER_LISTEN],
        scouting=False,
        multicast=False,
        gossip=True,
        connect_timeout=0,
    )
    stopped = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rpc.start()
    print(f"Hosted validation router listening on {ROUTER_LISTEN}", flush=True)
    try:
        stopped.wait()
    finally:
        rpc.stop()
        pool.close_all()


def run_host(name: str, tags: set[str]) -> None:
    """Serve one container-scoped Host on the validation fabric."""
    revision = package_version("dimos")
    daemon = HostDaemon(
        name,
        name=name,
        tags=tags,
        versions={
            "protocol": HOST_PROTOCOL_VERSION,
            "fragment_schema": FRAGMENT_SCHEMA_VERSION,
            "dimos": revision,
        },
    )
    pool = ZenohSessionPool()
    rpc = ZenohRPC(session_pool=pool)
    stopped = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rpc.start()
    control_name = HOST_CONTROL_RPC_NAME.format(host_id=name)
    rpc.serve_rpc(daemon.describe, f"{control_name}/describe")  # type: ignore[arg-type]
    rpc.serve_rpc(daemon.start, f"{control_name}/start")  # type: ignore[arg-type]
    rpc.serve_rpc(daemon.status, f"{control_name}/status")  # type: ignore[arg-type]
    rpc.serve_rpc(daemon.stop, f"{control_name}/stop")  # type: ignore[arg-type]
    token = rpc.session.liveliness().declare_token(HOST_LIVELINESS_KEY.format(host_id=name))
    print(f"Hosted validation Host {name} is available with tags {sorted(tags)}", flush=True)
    try:
        stopped.wait()
    finally:
        token.undeclare()
        daemon.shutdown()
        rpc.stop()
        pool.close_all()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("router")
    host_parser = subparsers.add_parser("host")
    host_parser.add_argument("--name", required=True)
    host_parser.add_argument("--tag", action="append", default=[])
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("scenario", choices=("basic", "replay", "sim", "visual"))
    run_parser.add_argument("--discovery-timeout", type=float, default=DEFAULT_DISCOVERY_TIMEOUT)
    run_parser.add_argument("--result-timeout", type=float, default=DEFAULT_RESULT_TIMEOUT)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.command == "router":
        run_router()
        return 0
    if args.command == "host":
        run_host(args.name, set(args.tag))
        return 0
    run_scenario(
        args.scenario,
        discovery_timeout=args.discovery_timeout,
        result_timeout=args.result_timeout,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
