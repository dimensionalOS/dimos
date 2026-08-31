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

"""Exercise one real Host deployment from a separate controller process."""

from __future__ import annotations

import hashlib
import json
import pickle
import socket
import time
from typing import Any
import uuid

from dimos.cli.commands.host import (
    _discover_host_ids,
    _get_descriptor,
    _host_rpc,
)
from dimos.core.global_config import global_config
from dimos.hosted.daemon import (
    HOST_CONTROL_RPC_NAME,
    DeploymentStatus,
    HostDescriptor,
    HostFragment,
)
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from experimental.hosted.remote_execution_probe import RemoteExecutionProbe

DISCOVERY_TIMEOUT = 20.0
RPC_TIMEOUT = 60.0
PROBE_INSTANCE = "remote_execution_probe"
PROBE_VALUE = 12345


def _call(rpc: ZenohRPC, name: str, args: list[Any], timeout: float) -> Any:
    result, unsubscribe = rpc.call_sync(name, (args, {}), rpc_timeout=timeout)
    try:
        return result
    finally:
        unsubscribe()


def _wait_for_available_host(rpc: ZenohRPC) -> HostDescriptor:
    deadline = time.monotonic() + DISCOVERY_TIMEOUT
    while time.monotonic() < deadline:
        for host_id in _discover_host_ids(rpc, timeout=1.0):
            descriptor = _get_descriptor(rpc, host_id, timeout=2.0)
            if descriptor.state == "available":
                return descriptor
        time.sleep(0.25)
    raise TimeoutError("No available Host found")


def _make_fragment(descriptor: HostDescriptor, run_id: str) -> HostFragment:
    blueprint = RemoteExecutionProbe.blueprint(instance_name=PROBE_INSTANCE).global_config(
        transport="zenoh",
        viewer="none",
        n_workers=1,
        zenoh_mode=global_config.zenoh_mode,
        zenoh_connect=global_config.zenoh_connect,
        zenoh_scouting=global_config.zenoh_scouting,
        zenoh_interface=global_config.zenoh_interface,
        zenoh_multicast=global_config.zenoh_multicast,
        zenoh_gossip=global_config.zenoh_gossip,
        zenoh_connect_timeout=global_config.zenoh_connect_timeout,
    )
    payload = pickle.dumps(blueprint, protocol=pickle.HIGHEST_PROTOCOL)
    return HostFragment(
        run_id=run_id,
        generation=1,
        host_id=descriptor.host_id,
        payload_digest=hashlib.sha256(payload).hexdigest(),
        blueprint_payload=payload,
    )


def main() -> None:
    controller_hostname = socket.gethostname()
    run_id = f"remote-exec-{uuid.uuid4().hex}"
    nonce = uuid.uuid4().hex

    with _host_rpc() as rpc:
        descriptor = _wait_for_available_host(rpc)
        fragment = _make_fragment(descriptor, run_id)
        control = HOST_CONTROL_RPC_NAME.format(host_id=descriptor.host_id)
        cleanup_needed = False
        start_status: DeploymentStatus | None = None
        running_status: DeploymentStatus | None = None
        stop_status: DeploymentStatus | None = None
        remote_result: dict[str, str | int] | None = None
        remote_error: str | None = None
        try:
            start_status = _call(
                rpc,
                f"{control}/start",
                [descriptor.epoch, fragment],
                RPC_TIMEOUT,
            )
            if not isinstance(start_status, DeploymentStatus):
                raise TypeError("Host returned an invalid start status")
            cleanup_needed = start_status.run_id == run_id
            if start_status.state != "running":
                raise RuntimeError(f"Remote deployment did not start: {start_status}")

            result = _call(
                rpc,
                f"{PROBE_INSTANCE}/execute",
                [nonce, PROBE_VALUE],
                RPC_TIMEOUT,
            )
            if not isinstance(result, dict):
                raise TypeError("Remote module returned an invalid result")
            remote_result = result
            if remote_result.get("nonce") != nonce:
                raise RuntimeError("Remote result has the wrong nonce")
            if remote_result.get("result") != PROBE_VALUE * PROBE_VALUE + 1:
                raise RuntimeError("Remote calculation returned the wrong result")
            if remote_result.get("hostname") == controller_hostname:
                raise RuntimeError("Probe executed in the controller container")

            try:
                _call(
                    rpc,
                    f"{PROBE_INSTANCE}/fail",
                    [nonce],
                    RPC_TIMEOUT,
                )
            except RuntimeError as exc:
                remote_error = str(exc)
                if remote_error != f"remote probe failure: {nonce}":
                    raise RuntimeError(f"Remote error changed in transit: {remote_error}") from exc
            else:
                raise RuntimeError("Remote failure did not reach the controller")

            running_status = _call(
                rpc,
                f"{control}/status",
                [descriptor.epoch, run_id],
                RPC_TIMEOUT,
            )
            if not isinstance(running_status, DeploymentStatus):
                raise TypeError("Host returned an invalid deployment status")
            if running_status.state != "running":
                raise RuntimeError(f"Remote deployment is not running: {running_status}")
        finally:
            if cleanup_needed:
                stop_status = _call(
                    rpc,
                    f"{control}/stop",
                    [descriptor.epoch, run_id, 1, fragment.payload_digest],
                    RPC_TIMEOUT,
                )

        final_descriptor = _get_descriptor(rpc, descriptor.host_id, timeout=2.0)
        if not isinstance(stop_status, DeploymentStatus) or stop_status.state != "available":
            raise RuntimeError(f"Remote deployment did not stop cleanly: {stop_status}")
        if final_descriptor.state != "available":
            raise RuntimeError(f"Host did not return to available: {final_descriptor.state}")

    print(
        json.dumps(
            {
                "controller_hostname": controller_hostname,
                "host_id": descriptor.host_id,
                "host_name": descriptor.name,
                "run_id": run_id,
                "start_state": start_status.state if start_status else None,
                "running_state": running_status.state if running_status else None,
                "remote_error": remote_error,
                "remote_result": remote_result,
                "stop_state": stop_status.state,
                "final_host_state": final_descriptor.state,
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
