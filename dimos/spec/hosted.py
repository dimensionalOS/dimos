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
"""Usage sketch for the hosted API.

The APIs and some names below do not exist yet. This file only shows how we
expect hosted placement to be used.

This file is only for documentation purposes and is not meant to be executed.
"""

from __future__ import annotations

# Top level Blueprint
# Keep the robot stack local and run marker detection on one GPU Host.
unitree_go2_markers = autoconnect(
    unitree_go2,
    MarkerDetectionStreamModule.blueprint(
        marker_length_m=0.1,
        camera_info=GO2Connection.camera_info_static,
    ).placement(tags={"gpu"}),
)

# Select one G1 Host and one GPU Host; navigation remains local.
multi_host_g1 = autoconnect(
    unitree_g1.blueprint().placement(tags={"g1"}),
    navigation.blueprint(),
    expensive.blueprint().placement(tags={"gpu"}),
)

# Pin the robot fragment to one exact Host name or ID.
pinned_g1 = autoconnect(
    unitree_g1.blueprint().placement(host="g1-01"),
    navigation.blueprint(),
    expensive.blueprint().placement(tags={"gpu"}),
)

# Select one matching embodiment; this does not create replicas.
available_g1 = unitree_g1.blueprint().placement(tags={"g1"})

# Hosted Zenoh namespaces
#
# ZenohRPC adds the ``dimos/rpc/`` prefix to the logical RPC names below.
HOST_LIVELINESS_KEY = "dimos/hosts/{host_id}/live"
HOST_CONTROL_RPC_NAME = "hosts/{host_id}"
RUN_STREAM_KEY = "dimos/runs/{run_id}/streams/{stream}/{message_type}"
RUN_MODULE_RPC_NAME = "runs/{run_id}/hosts/{host_id}/modules/{module}"


# Host control plane
# ```bash
# dimos host serve --name g1-01 --tag g1 --tag lab-a


class HostControl:
    """One controller-side Zenoh RPC channel shared by all Host clients."""

    def __init__(self, rpc: RPCSpec) -> None:
        self._rpc = rpc

    def client(self, host_id: str, epoch: str, timeout: float = 5.0) -> HostClient:
        """Create the startup proxy used internally by ``dimos run``."""
        ...

    def serve(self, server: HostServer) -> None:
        """Expose a HostServer under ``hosts/<host_id>`` on this channel."""
        ...

    def call(
        self,
        host_id: str,
        method: str,
        *args: object,
        timeout: float,
    ) -> object:
        """Call ``dimos/rpc/hosts/<host_id>/<method>``."""
        ...


class HostClient:
    """Startup-only control proxy created internally by ``dimos run``."""

    def __init__(
        self,
        host_id: str,
        epoch: str,
        control: HostControl,
        timeout: float = 5.0,
    ) -> None:
        self._host_id = host_id
        self._epoch = epoch
        self._control = control
        self._timeout = timeout

    def describe(self) -> HostDescriptor: ...

    def start(self, fragment: HostFragment) -> DeploymentStatus: ...

    def status(self, run_id: str) -> DeploymentStatus: ...

    def stop(self, run_id: str, generation: int, fragment_digest: str) -> DeploymentStatus: ...


class HostServer:
    """Persistent Host supervisor whose methods are exposed over Zenoh RPC."""

    def __init__(
        self,
        host_id: str,
        name: str,
        tags: set[str],
    ) -> None:
        self._host_id = host_id
        self._name = name
        self._tags = tags
        self._epoch = uuid.uuid4().hex
        self._current_fragment: HostFragment | None = None

    @rpc
    def describe(self) -> HostDescriptor: ...

    @rpc
    def start(self, epoch: str, fragment: HostFragment) -> DeploymentStatus: ...

    @rpc
    def status(self, epoch: str, run_id: str) -> DeploymentStatus: ...

    @rpc
    def stop(
        self,
        epoch: str,
        run_id: str,
        generation: int,
        fragment_digest: str,
    ) -> DeploymentStatus: ...


# Host process bootstrap registers HostServer under HOST_CONTROL_RPC_NAME.
# DistributedRunner owns HostControl and creates HostClient instances only for
# discovery, start, status, and stop. Once start succeeds, modules use
# RUN_STREAM_KEY and RUN_MODULE_RPC_NAME directly over Zenoh; HostClient is not
# part of the application data path.


# Placement Method


class Blueprint:
    def placement(self, host: str | None = None, tags: set[str] | None = None) -> Blueprint:
        """Specify placement constraints for this blueprint.

        Args:
            host: The exact host name or ID to place on.
            tags: A set of tags to match against available hosts.

        Returns:
            A new Blueprint with the specified placement constraints.
        """
        # Implementation would go here
        return self
