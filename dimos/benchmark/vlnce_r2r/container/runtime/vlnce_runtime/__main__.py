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

import argparse
import gzip
import json
import os
import signal
import sys
from threading import Event
import time

import habitat_sim

from .environment import OfficialEpisodeEnvironment
from .gateway import EpisodeGateway, GatewayServer
from .native_renderer import NativeEpisodeRenderer
from .private_case import IDENTITY_FIELDS, load_private_case
from .result import build_result, publish_result


def _configuration(scene_path):
    simulator = habitat_sim.SimulatorConfiguration()
    simulator.scene_id = scene_path
    simulator.enable_physics = False

    color = habitat_sim.SensorSpec()
    color.uuid = "rgb"
    color.sensor_type = habitat_sim.SensorType.COLOR
    color.resolution = [224, 224]
    color.position = [0.0, 1.25, 0.0]
    color.hfov = 90.0

    depth = habitat_sim.SensorSpec()
    depth.uuid = "depth"
    depth.sensor_type = habitat_sim.SensorType.DEPTH
    depth.resolution = [256, 256]
    depth.position = [0.0, 1.25, 0.0]
    depth.hfov = 90.0

    agent = habitat_sim.agent.AgentConfiguration()
    agent.height = 1.5
    agent.radius = 0.1
    agent.sensor_specifications = [color, depth]
    return habitat_sim.Configuration(simulator, [agent])


def smoke(scene_path, navmesh_path):
    simulator = habitat_sim.Simulator(_configuration(scene_path))
    try:
        if not simulator.pathfinder.load_nav_mesh(navmesh_path):
            raise RuntimeError("failed to load the pinned navmesh")
        state = habitat_sim.AgentState()
        state.position = simulator.pathfinder.get_random_navigable_point()
        simulator.initialize_agent(0, state)
        observations = simulator.get_sensor_observations()
        report = {
            "habitat_sim": getattr(habitat_sim, "__version__", "0.1.7"),
            "scene": os.path.basename(scene_path),
            "navmesh_loaded": simulator.pathfinder.is_loaded,
            "agent_height": 1.5,
            "agent_radius": 0.1,
            "camera_height": 1.25,
            "rgb_shape": list(observations["rgb"].shape),
            "depth_shape": list(observations["depth"].shape),
        }
        print(json.dumps(report, sort_keys=True))
    finally:
        simulator.close()


def official_smoke(dataset_path, scenes_dir, work_dir, episode_id):
    with gzip.open(dataset_path, "rt", encoding="utf-8") as handle:
        episodes = [
            episode
            for episode in json.load(handle)["episodes"]
            if str(episode["episode_id"]) == episode_id
        ]
    if len(episodes) != 1:
        raise RuntimeError("official smoke requires exactly one selected episode")
    episode = episodes[0]
    private_case = {
        "split": "train",
        "episode_id": episode_id,
        "scene_id": episode["scene_id"],
        "instruction": episode["instruction"]["instruction_text"],
    }
    environment = OfficialEpisodeEnvironment(
        private_case,
        episode,
        dataset_path,
        scenes_dir,
        work_dir,
    )
    try:
        occupancy = environment.static_occupancy()
        metrics = environment.submit_route()
        print(
            json.dumps(
                {
                    "episode_id": episode_id,
                    "rgb_shape": list(environment.observations["rgb"].shape),
                    "depth_shape": list(environment.observations["depth"].shape),
                    "map_shape": [occupancy["height"], occupancy["width"]],
                    "metrics": metrics,
                },
                sort_keys=True,
            )
        )
    finally:
        environment.close()


def serve(
    private_case_path,
    expected_identity_path,
    dataset_path,
    scenes_dir,
    work_dir,
    socket_path,
    result_path,
    render_output=None,
    render_metadata=None,
):
    """Run one verified episode and publish only a scoreable terminal result."""

    stopping = Event()
    previous_handlers = {}

    def request_stop(signum, _frame):
        _diagnostic("signal", signal=signum)
        stopping.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_stop)

    environment = None
    server = None
    renderer = None
    try:
        expected_identity = _expected_identity(expected_identity_path)
        private_case, episode = load_private_case(
            private_case_path,
            dataset_path,
            expected_identity,
        )
        _diagnostic(
            "private-case-verified",
            attempt_id=private_case["attempt_id"],
            case_id=private_case["case_id"],
            episode_id=private_case["episode_id"],
        )
        environment = OfficialEpisodeEnvironment(
            private_case,
            episode,
            dataset_path,
            scenes_dir,
            work_dir,
        )
        if render_output is not None:
            if render_metadata is None:
                raise RuntimeError("native rendering requires a metadata path")
            renderer = NativeEpisodeRenderer(render_output, render_metadata, environment)
        gateway = EpisodeGateway(private_case, environment, renderer=renderer)
        server = GatewayServer(socket_path, gateway)
        server.start()
        _diagnostic("ready", socket=socket_path)

        while not gateway.begun.is_set():
            gateway.process_pending(timeout=0.1)
            if stopping.is_set():
                _diagnostic("interrupted-before-begin")
                return 130

        started = time.monotonic()
        _diagnostic("episode-started")
        deadline = started + private_case["timeout_seconds"]
        while not gateway.finished.is_set():
            gateway.process_pending(timeout=0.1)
            if stopping.is_set():
                _diagnostic("interrupted")
                return 130
            if time.monotonic() >= deadline:
                gateway.finish_timeout()
                break

        if gateway.failure is not None:
            raise gateway.failure
        if gateway.terminal_reason == "cancelled":
            _diagnostic("cancelled")
            return 2
        if gateway.terminal_reason not in ("submitted", "timeout"):
            raise RuntimeError("gateway terminated without a scoreable reason")
        terminal_result = build_result(
            private_case,
            gateway.terminal_reason,
            environment.trajectory,
            gateway.native_metrics,
            _runtime_provenance(private_case),
            time.monotonic() - started,
        )
        publish_result(result_path, terminal_result)
        _diagnostic(
            "result-published",
            path=result_path,
            terminal_reason=gateway.terminal_reason,
        )
        return 0
    finally:
        if server is not None:
            server.stop()
        if renderer is not None:
            renderer.close()
        if environment is not None:
            environment.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        _diagnostic("shutdown-complete")


def _expected_identity(path):
    try:
        with open(path, encoding="utf-8") as handle:
            identity = json.load(handle)
    except (OSError, ValueError) as error:
        raise RuntimeError("could not read expected attempt identity") from error
    if not isinstance(identity, dict) or frozenset(identity) != frozenset(IDENTITY_FIELDS):
        raise RuntimeError("expected attempt identity fields are invalid")
    return identity


def _runtime_provenance(private_case):
    return {
        "runtime_image_digest": private_case["runtime_image_digest"],
        "habitat_sim": getattr(habitat_sim, "__version__", "0.1.7"),
        "protocol_revision": private_case["protocol_revision"],
        "python": "{}.{}.{}".format(*sys.version_info[:3]),
    }


def _diagnostic(event, **fields):
    payload = {"event": event, "monotonic_seconds": time.monotonic()}
    payload.update(fields)
    sys.stderr.write(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")
    sys.stderr.flush()


def main():
    parser = argparse.ArgumentParser()
    subcommands = parser.add_subparsers(dest="command")
    smoke_parser = subcommands.add_parser("smoke")
    smoke_parser.add_argument("--scene", required=True)
    smoke_parser.add_argument("--navmesh", required=True)
    official_parser = subcommands.add_parser("official-smoke")
    official_parser.add_argument("--dataset", required=True)
    official_parser.add_argument("--scenes-dir", required=True)
    official_parser.add_argument("--work-dir", required=True)
    official_parser.add_argument("--episode-id", required=True)
    serve_parser = subcommands.add_parser("serve")
    serve_parser.add_argument("--private-case", required=True)
    serve_parser.add_argument("--expected-identity", required=True)
    serve_parser.add_argument("--dataset", required=True)
    serve_parser.add_argument("--scenes-dir", required=True)
    serve_parser.add_argument("--work-dir", required=True)
    serve_parser.add_argument("--socket", required=True)
    serve_parser.add_argument("--result", required=True)
    serve_parser.add_argument("--render-output")
    serve_parser.add_argument("--render-metadata")
    args = parser.parse_args()
    if args.command == "smoke":
        smoke(args.scene, args.navmesh)
        return
    if args.command == "official-smoke":
        official_smoke(args.dataset, args.scenes_dir, args.work_dir, args.episode_id)
        return
    if args.command == "serve":
        raise SystemExit(
            serve(
                args.private_case,
                args.expected_identity,
                args.dataset,
                args.scenes_dir,
                args.work_dir,
                args.socket,
                args.result,
                args.render_output,
                args.render_metadata,
            )
        )
    parser.error("a command is required")


if __name__ == "__main__":
    main()
