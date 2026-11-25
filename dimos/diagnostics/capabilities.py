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

"""Truthful capability inventory; support is distinct from tested qualification."""

from importlib.util import find_spec
from typing import Any


def capabilities() -> dict[str, Any]:
    return {
        "schema": "dimos.diagnostics.capabilities/v1",
        "transports": {
            name: {
                "local_spans": True,
                "traced_carrier": "string-data",
                "native_identity": name == "Zenoh",
                "native_writer_metadata": "binding_dependent"
                if name in ("ROS", "DDS")
                else "not_exposed",
                "peer_negotiation": True,
                "qualification": "not_run",
            }
            for name in ["Zenoh", "LCM", "ROS", "DDS", "WebRTC", "Cloudflare"]
        },
        "in_process": {"context_propagation": True},
        "native_rust": {
            "instrumentation": "local_sdk_spans_require_rebuild",
            "lineage": "Zenoh, derived module input/output queues and async handlers; custom caches must retain context",
            "modes": ["off", "metrics", "trace"],
        },
        "video": {
            "transport_calls": True,
            "source_publication_to_rtp": "native gateway and browser SDK",
            "texture_and_xr": "Spatial Desktop instrumented software boundaries",
            "camera_exposure": "vendor driver timestamp; physical exposure mapping unavailable",
        },
        "hardware_acceptance": "adapter_required",
        "physical_display": "optical_test_required",
        "ap_topology": "read-only Linux AP SSH collector and normalized controller import; actual access required",
        "socket_probes": "Linux 64-bit receive timestamps when supported; other hosts RTT only",
        "exports": ["json", "perfetto", "otlp-http-json"],
        "installed_optional_dependencies": {
            name: find_spec(name) is not None for name in ("rclpy", "cyclonedds", "aiortc")
        },
    }
