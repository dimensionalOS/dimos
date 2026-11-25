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

"""Construct explicit traced links without touching the original robot channels."""

from __future__ import annotations

from importlib import import_module
from typing import Any

from dimos.core.stream import Transport, TransportSpec
from dimos.core.transport import (
    CloudflareTransport,
    DDSTransport,
    LCMTransport,
    ROSTransport,
    WebRTCTransport,
    ZenohTransport,
)
from dimos.diagnostics.envelope import traced_channel
from dimos.diagnostics.hosted_carrier import HostedTraceCarrier
from dimos.diagnostics.traced_transport import TracedTransport
from dimos.msgs.std_msgs.String import String


def traced_link(
    backend: str, run_id: str, link_id: str, payload_type: type, **transport_options: Any
) -> TransportSpec:
    """Return a traced transport spec; both endpoints must use the same contract.

    Carrier QoS/provider configuration is passed through, never selected implicitly.
    Only dedicated tracing topics carry envelopes. Legacy endpoints are not bridged.
    """
    classes: dict[str, type[Transport[Any]]] = {
        "zenoh": ZenohTransport,
        "lcm": LCMTransport,
        "ros": ROSTransport,
        "dds": DDSTransport,
        "webrtc": WebRTCTransport,
        "cloudflare": CloudflareTransport,
    }
    if backend not in classes:
        raise ValueError(f"unsupported traced backend {backend}")
    if {"topic", "type", "msg_type"} & transport_options.keys():
        raise ValueError("traced channel and wire type are reserved")
    wire_type = (
        import_module("dimos.diagnostics.dds_frame").TraceFrame if backend == "dds" else String
    )
    type_argument = "msg_type" if backend in ("ros", "webrtc", "cloudflare") else "type"
    if backend in ("webrtc", "cloudflare"):
        if not {"config", "publish_channel", "receive_channel"} <= transport_options.keys():
            raise ValueError(
                "hosted traced links require a ProviderConfig and explicit publish/receive channels"
            )
        carrier = TransportSpec(
            HostedTraceCarrier,
            kwargs={"topic": traced_channel(run_id, link_id), **transport_options},
        )
    else:
        carrier = TransportSpec(
            classes[backend],
            kwargs={
                "topic": traced_channel(run_id, link_id),
                type_argument: wire_type,
                **transport_options,
            },
        )
    return TransportSpec(
        TracedTransport,
        kwargs={
            "carrier": carrier,
            "wire_type": wire_type,
            "payload_type": payload_type,
            "run_id": run_id,
            "link_id": link_id,
        },
    )
