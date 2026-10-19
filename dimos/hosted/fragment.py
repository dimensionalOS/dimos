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

"""Immutable deployment payloads accepted by a DimOS Host."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import pickle
from typing import Any

from dimos.core.coordination.blueprints import Blueprint


@dataclass(frozen=True, slots=True)
class RemoteModuleReference:
    """A local module attribute backed by a provider on another Host, by rpc names only."""

    consumer_name: str
    reference_name: str
    provider_name: str
    provider_host_id: str
    rpcs: frozenset[str]
    rpc_name: str


@dataclass(frozen=True, slots=True)
class PythonFragmentPayload:
    """The local Blueprint and the controller's explicit configuration for it.

    ``overrides`` is what the controller's config file and CLI set, by section;
    ``global_overrides`` its global flags. The Host parses them on top of its own
    environment, so each machine's .env configures its own fragment.
    """

    blueprint: Blueprint
    overrides: Mapping[str, Any] | None = None
    global_overrides: Mapping[str, Any] | None = None
    remote_module_references: tuple[RemoteModuleReference, ...] = ()


@dataclass(frozen=True, slots=True)
class HostFragment:
    """Deployment unit sent to one Host; the payload stays pickled until the fragment process."""

    run_id: str
    host_id: str
    revision: str
    payload: bytes

    @classmethod
    def create(
        cls, *, run_id: str, host_id: str, revision: str, payload: PythonFragmentPayload
    ) -> HostFragment:
        payload_bytes = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
        return cls(run_id=run_id, host_id=host_id, revision=revision, payload=payload_bytes)

    def load_payload(self) -> PythonFragmentPayload:
        payload = pickle.loads(self.payload)
        if not isinstance(payload, PythonFragmentPayload):
            raise TypeError("Fragment payload is not a PythonFragmentPayload")
        return payload
