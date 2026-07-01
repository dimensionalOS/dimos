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

"""Protocol version compatibility helpers."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import zip_longest

from dimos_runtime_protocol.models import ProtocolVersion, RuntimeDescription
from dimos_runtime_protocol.version import PROTOCOL_VERSION


@dataclass(frozen=True)
class CompatibilityResult:
    """Compatibility check outcome."""

    compatible: bool
    reason: str = ""


def _parts(version: str) -> tuple[int, ...]:
    values: list[int] = []
    for part in version.split("."):
        if not part.isdigit():
            break
        values.append(int(part))
    return tuple(values)


def _less_than(left: str, right: str) -> bool:
    for l_value, r_value in zip_longest(_parts(left), _parts(right), fillvalue=0):
        if l_value < r_value:
            return True
        if l_value > r_value:
            return False
    return False


def check_compatible(
    runtime: RuntimeDescription | ProtocolVersion,
    *,
    client_version: str = PROTOCOL_VERSION,
) -> CompatibilityResult:
    """Check whether a sidecar protocol version can talk to this client."""

    protocol = runtime.protocol if isinstance(runtime, RuntimeDescription) else runtime
    if _less_than(client_version, protocol.min_compatible):
        return CompatibilityResult(
            compatible=False,
            reason=(
                f"client protocol {client_version} is older than sidecar minimum "
                f"{protocol.min_compatible}"
            ),
        )
    if _parts(client_version)[:1] != _parts(protocol.version)[:1]:
        return CompatibilityResult(
            compatible=False,
            reason=f"protocol major mismatch: client={client_version} sidecar={protocol.version}",
        )
    return CompatibilityResult(compatible=True)
