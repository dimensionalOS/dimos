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

"""Per-key-expression Zenoh publisher QoS rules.

This module must stay free of zenoh/dimos imports so `dimos.core.global_config`
can import it from lightweight entry-points (see `visualization/rerun/constants.py`).
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel


class ZenohQoS(BaseModel):
    """One publisher-QoS rule: applies to key expressions intersecting `key`."""

    key: str  # zenoh key-expr pattern, e.g. "dimos/rpc/**"
    reliability: Literal["reliable", "best_effort"] | None = None  # None = zenoh default
    congestion_control: Literal["drop", "block"] | None = None  # None = zenoh default


DEFAULT_ZENOH_QOS: tuple[ZenohQoS, ...] = (
    # RPC requests/responses are one-shot: never drop under congestion.
    ZenohQoS(key="dimos/rpc/**", reliability="reliable", congestion_control="block"),
    # Agent/human channels: low-rate, loss unacceptable.
    ZenohQoS(key="dimos/human_input", reliability="reliable", congestion_control="block"),
    ZenohQoS(key="dimos/agent", reliability="reliable", congestion_control="block"),
    ZenohQoS(key="dimos/agent_idle", reliability="reliable", congestion_control="block"),
    # High-rate sensor streams (typed transports embed the message type in the
    # key): drop stale frames under congestion rather than block publishers.
    ZenohQoS(key="**/sensor_msgs.Image", reliability="best_effort", congestion_control="drop"),
    ZenohQoS(
        key="**/sensor_msgs.PointCloud2", reliability="best_effort", congestion_control="drop"
    ),
)
