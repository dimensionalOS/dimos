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

from dimos.protocol.pubsub.benchmark.docker_network import DockerNetworkTrial
from dimos.protocol.pubsub.benchmark.matrix import build_matrix
from dimos.protocol.pubsub.benchmark.model import Environment, NetworkProfile, Stack
from dimos.protocol.pubsub.benchmark.runner import _docker_worker_command


def _degraded_spec():
    return next(
        spec
        for spec in build_matrix("public", repetitions=1)
        if spec.environment == Environment.EMULATED and spec.profile == NetworkProfile.DEGRADED
    )


def test_endpoint_command_uses_dedicated_bridge_capability_and_fixed_address(
    mocker, tmp_path
) -> None:
    run = mocker.patch("dimos.protocol.pubsub.benchmark.docker_network._run")
    trial = DockerNetworkTrial(_degraded_spec(), tmp_path, image="benchmark@sha256:abc")

    trial._run_endpoint(trial.publisher_container, "10.88.0.10")

    command = run.call_args_list[0].args[0]
    assert command[:3] == ["docker", "run", "--detach"]
    assert "NET_ADMIN" in command
    assert "10.88.0.10" in command
    assert "benchmark@sha256:abc" in command
    assert "net.core.rmem_max=67108864" in run.call_args_list[1].args[0]


def test_degraded_profile_applies_symmetrically_to_both_endpoints(mocker, tmp_path) -> None:
    run = mocker.patch("dimos.protocol.pubsub.benchmark.docker_network._run")
    trial = DockerNetworkTrial(_degraded_spec(), tmp_path)

    trial.apply_profile()

    commands = [call.args[0] for call in run.call_args_list]
    assert len(commands) == 2
    assert {command[2] for command in commands} == {
        trial.publisher_container,
        trial.subscriber_container,
    }
    assert all(
        "eth0" in command and "30ms" in command and "10ms" in command for command in commands
    )
    assert all("1%" in command and "25%" in command and "20mbit" in command for command in commands)


def test_cleanup_removes_both_containers_and_network(mocker, tmp_path) -> None:
    run = mocker.patch("dimos.protocol.pubsub.benchmark.docker_network.subprocess.run")
    trial = DockerNetworkTrial(_degraded_spec(), tmp_path)

    trial.__exit__(None, None, None)

    commands = [call.args[0] for call in run.call_args_list]
    assert ["docker", "rm", "--force", trial.publisher_container] in commands
    assert ["docker", "rm", "--force", trial.subscriber_container] in commands
    assert ["docker", "network", "rm", trial.network_name] in commands


def test_ros_endpoints_both_connect_to_owned_router() -> None:
    spec = next(spec for spec in build_matrix("smoke") if spec.stack == Stack.ROS2_ZENOH)

    for role in ("publisher", "subscriber"):
        command = _docker_worker_command("endpoint", role, spec, f"{role}.json")
        override = next(
            value.removeprefix("ZENOH_CONFIG_OVERRIDE=")
            for value in command
            if value.startswith("ZENOH_CONFIG_OVERRIDE=")
        )
        assert 'mode="client"' in override
        assert 'connect/endpoints=["tcp/10.88.0.10:7447"]' in override
        assert "transport/shared_memory/enabled=false" in override
