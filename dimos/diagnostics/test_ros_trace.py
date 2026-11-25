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

"""ROS executor evidence must not imply command delivery or a shared clock."""

from dimos.diagnostics.analysis import analyze
from dimos.diagnostics.ros_trace import boundaries


def test_overlapping_callbacks_are_paired_by_process_thread_and_callback():
    rows = [
        dict(name="ros2:" + name, mono_ns=t, clock="ctf", pid=pid, tid=1, fields={"callback": 99})
        for name, t, pid in [
            ("callback_start", 0, 1),
            ("callback_start", 10, 2),
            ("callback_end", 20, 1),
            ("callback_end", 40, 2),
        ]
    ]
    events = list(boundaries(rows, "run", "capture"))
    report = analyze([], events)
    assert report["stages"]["ros.callback"]["n"] == 2
    assert report["stages"]["ros.callback"]["max_ms"] == 0.00003
    assert {event["clock_domain"] for event in events} == {"ros-ctf/capture/ctf"}
    assert all(event["context"] is None for event in events)


def test_executor_points_do_not_invent_wait_spans_or_sample_ids():
    rows = [
        dict(
            name="ros2:rclcpp_executor_wait_for_work",
            mono_ns=10,
            clock="ctf",
            pid=1,
            tid=1,
            fields={"timeout": 100},
        )
    ]
    events = list(boundaries(rows, "run", "capture"))
    assert events[0]["event"] == "ros.executor.wait_for_work"
    assert analyze([], events)["stages"] == {}
    assert events[0]["sample_correlation"] == "unavailable"
