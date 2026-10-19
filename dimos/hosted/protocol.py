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

"""The wire contract between a Host and its controllers: keys, states, descriptors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

DeploymentState = Literal["absent", "starting", "running", "stopping", "failed"]

HOST_LIVELINESS_KEY = "dimos/hosts/{host_id}/live"
HOST_CONTROL_RPC_NAME = "hosts/{host_id}"
# Held by a run's controller; a Host stops the run's fragment once it is gone.
RUN_LIVELINESS_KEY = "dimos/runs/{run_id}/live"
# Every output line of a fragment, published while its run's controller listens.
RUN_LOG_KEY = "dimos/runs/{run_id}/logs/{host}"
RUN_MODULE_RPC_NAME = "runs/{run_id}/hosts/{host_id}/modules/{module}"
RUN_COORDINATOR_RPC_NAME = "runs/{run_id}/hosts/{host_id}/coordinator"


def run_module_rpc_name(run_id: str, host_id: str, module_name: str) -> str:
    """Return the run- and Host-scoped logical RPC name for a module."""
    return RUN_MODULE_RPC_NAME.format(
        run_id=run_id,
        host_id=host_id,
        module=module_name.lstrip("/"),
    )


def run_coordinator_rpc_name(run_id: str, host_id: str) -> str:
    """Return the run- and Host-scoped Coordinator RPC name."""
    return RUN_COORDINATOR_RPC_NAME.format(run_id=run_id, host_id=host_id)


@dataclass(frozen=True, slots=True)
class HostDescriptor:
    host_id: str
    name: str
    # Key/value facts; a bare tag has an empty value.
    tags: dict[str, str]
    # Code revision the Host runs; a fragment must be compiled from the same one.
    revision: str
    # The load signal for placement; a Host takes any number of runs.
    active_run_ids: tuple[str, ...]
    # The zid of the router this Host serves, and that router's listen endpoints.
    router_zid: str = ""
    listen: tuple[str, ...] = ()
    # A start call is answered within this many seconds, the fragment started or killed.
    start_timeout: float = 0.0


@dataclass(frozen=True, slots=True)
class DeploymentStatus:
    state: DeploymentState
    run_id: str | None
    pid: int | None
    log_dir: str | None
    error: str | None
