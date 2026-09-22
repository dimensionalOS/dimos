# Copyright 2025-2026 Dimensional Inc.
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

"""Continuous control command and state payload.

One publisher's sparse set of interface values, carried as parallel
``interface_names`` / ``values`` arrays. Capabilities are described out of band
and lifecycle transitions travel over RPC, so this type stays deliberately thin:
it pins the wire shape and rejects frames that are structurally impossible.

Empty arrays are valid and encodable -- that is the coordinator's heartbeat,
which carries an epoch and a sequence but commands nothing.

Interface-name vocabulary, epoch and sequence rules, and per-interface limits
are the control contract's business, not this type's.
"""

from __future__ import annotations

import math
import time

from dimos_lcm.dimos_control_msgs import ControlValues as LCMControlValues


class ControlValues:
    """Values for a set of named interfaces, stamped by one publisher."""

    msg_name = "dimos_control_msgs.ControlValues"

    source: str
    source_ts: float
    epoch: int
    sequence: int
    interface_names: list[str]
    values: list[float]

    def __init__(
        self,
        source: str,
        source_ts: float | None = None,
        epoch: int = 0,
        sequence: int = 0,
        interface_names: list[str] | None = None,
        values: list[float] | None = None,
    ) -> None:
        """Initialize a ControlValues frame.

        Args:
            source: Publisher identity, not a hardware-specific routing
                identifier. A module publishing two sources uses two values here.
            source_ts: Publisher Unix timestamp in seconds. Defaults to now.
                It is not proof of receiver-side freshness.
            epoch: State generation or command session this frame belongs to.
            sequence: Position within ``epoch``. Only comparable inside one epoch.
            interface_names: Names addressed by this frame, parallel to ``values``.
            values: Value per name, in the same order as ``interface_names``.

        Raises:
            ValueError: If the two arrays differ in length, or any value is
                infinite or NaN.
        """
        names = list(interface_names) if interface_names is not None else []
        vals = [float(v) for v in values] if values is not None else []

        if len(names) != len(vals):
            raise ValueError(
                f"interface_names and values must be the same length; "
                f"got {len(names)} names and {len(vals)} values"
            )
        for name, value in zip(names, vals, strict=True):
            if not math.isfinite(value):
                raise ValueError(f"non-finite value for interface {name!r}: {value!r}")

        self.source = source
        self.source_ts = time.time() if source_ts is None else source_ts
        self.epoch = epoch
        self.sequence = sequence
        self.interface_names = names
        self.values = vals

    def as_dict(self) -> dict[str, float]:
        """Map each interface name to its value.

        Names are not checked for uniqueness here; on a duplicate the last
        value wins. The contract package is what forbids duplicates.
        """
        return dict(zip(self.interface_names, self.values, strict=True))

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMControlValues(
            source=self.source,
            source_ts=self.source_ts,
            epoch=self.epoch,
            sequence=self.sequence,
            interface_names_length=len(self.interface_names),
            values_length=len(self.values),
            interface_names=list(self.interface_names),
            values=list(self.values),
        )
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> ControlValues:
        lcm_msg = LCMControlValues.lcm_decode(data)
        return cls(
            source=lcm_msg.source,
            source_ts=lcm_msg.source_ts,
            epoch=lcm_msg.epoch,
            sequence=lcm_msg.sequence,
            interface_names=list(lcm_msg.interface_names),
            values=[float(v) for v in lcm_msg.values],
        )

    def __repr__(self) -> str:
        return (
            f"ControlValues(source={self.source!r}, source_ts={self.source_ts}, "
            f"epoch={self.epoch}, sequence={self.sequence}, "
            f"interface_names={self.interface_names}, values={self.values})"
        )
