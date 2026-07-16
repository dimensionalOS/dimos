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

"""Numeric metrics that can be transported over LCM and graphed in Rerun."""

from __future__ import annotations

import time
from typing import Any

from dimos_lcm.diagnostic_msgs import (
    DiagnosticArray as LCMMetricsArray,
    DiagnosticStatus,
    KeyValue,
)

from dimos.msgs.std_msgs.Header import Header


class MetricsArray(LCMMetricsArray):  # type: ignore[misc]
    """Numeric time-series metrics backed by the compatible LCM schema."""

    msg_name = "metrics_msgs.MetricsArray"

    @classmethod
    def from_numeric_values(
        cls,
        entity_path: str,
        values: dict[str, float | int],
        *,
        timestamp: float | None = None,
        hardware_id: str = "",
    ) -> MetricsArray:
        """Create one metric group containing numeric key/value pairs."""
        key_values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        status = DiagnosticStatus(
            values_length=len(key_values),
            level=0,
            name=entity_path.strip("/"),
            message="numeric metrics",
            hardware_id=hardware_id,
            values=key_values,
        )
        return cls(
            status_length=1,
            header=Header(timestamp if timestamp is not None else time.time()),
            status=[status],
        )

    @classmethod
    def lcm_decode(cls, data: bytes) -> MetricsArray:
        """Decode through the generated base class and restore this wrapper type."""
        decoded = LCMMetricsArray.lcm_decode(data)
        return cls(
            status_length=decoded.status_length,
            header=decoded.header,
            status=decoded.status,
        )

    def to_rerun(self) -> list[tuple[str, Any]]:
        """Convert numeric metric values to Rerun scalar entity paths."""
        import rerun as rr

        results: list[tuple[str, Any]] = []
        for status in self.status:
            entity_path = status.name.strip("/")
            if not entity_path:
                continue
            for key_value in status.values:
                try:
                    value = float(key_value.value)
                except (TypeError, ValueError):
                    continue
                key = key_value.key.strip("/")
                if key:
                    results.append((f"{entity_path}/{key}", rr.Scalars(value)))
        return results
