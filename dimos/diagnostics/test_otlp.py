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

"""Offline export must not manufacture a wall-clock anchor for native timelines."""

from dimos.diagnostics.otlp import otlp


def test_otlp_export_preserves_local_duration_and_reports_missing_anchor():
    events = [
        {
            "event": "work." + edge,
            "process": "p",
            "clock_domain": "c",
            "span_id": "1",
            "mono_ns": at,
        }
        for edge, at in [("start", 100), ("end", 150)]
    ]
    assert otlp([], events)["dimosExportHealth"]["omittedSpans"] == 1
    document = otlp(
        [{"process": "p", "clock_domain": "c", "mono_ns": 10, "wall_ns": 1000, "run_id": "run"}],
        events,
    )
    span = document["resourceSpans"][0]["scopeSpans"][0]["spans"][0]
    assert (span["startTimeUnixNano"], span["endTimeUnixNano"]) == ("1090", "1140")
    assert document["dimosExportHealth"]["omittedSpans"] == 0
