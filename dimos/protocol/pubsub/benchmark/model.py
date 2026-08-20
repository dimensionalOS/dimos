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

"""Stable data model for transport benchmark campaigns."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Literal

SCHEMA_VERSION = 2


class Stack(str, Enum):
    LCM = "lcm"
    ZENOH = "zenoh"
    ROS2_ZENOH = "ros2-zenoh"


class Cohort(str, Enum):
    FRESHNESS = "freshness"
    RELIABLE = "reliable"
    MIXED = "mixed"


class Environment(str, Enum):
    LOCAL = "local"
    EMULATED = "emulated"


class NetworkProfile(str, Enum):
    CLEAN = "clean"
    CONSTRAINED = "constrained"
    DEGRADED = "degraded"


@dataclass(frozen=True)
class NetworkConditions:
    rate_mbit_s: int | None = None
    delay_ms: float = 0.0
    jitter_ms: float = 0.0
    loss_pct: float = 0.0
    loss_correlation_pct: float = 0.0


NETWORK_CONDITIONS = {
    NetworkProfile.CLEAN: NetworkConditions(),
    NetworkProfile.CONSTRAINED: NetworkConditions(
        rate_mbit_s=100,
        delay_ms=10.0,
        jitter_ms=2.0,
        loss_pct=0.1,
    ),
    NetworkProfile.DEGRADED: NetworkConditions(
        rate_mbit_s=20,
        delay_ms=30.0,
        jitter_ms=10.0,
        loss_pct=1.0,
        loss_correlation_pct=25.0,
    ),
}


@dataclass(frozen=True)
class TopicWorkload:
    name: str
    payload_bytes: int
    rate_hz: float
    cohort: Cohort
    message_kind: Literal["bytes", "control", "odometry", "image", "pointcloud"] = "bytes"


@dataclass(frozen=True)
class Workload:
    name: str
    topics: tuple[TopicWorkload, ...]
    saturation: bool = False
    saturation_max_messages: int | None = None
    saturation_max_bytes: int | None = None


@dataclass(frozen=True)
class TrialSpec:
    trial_id: str
    stack: Stack
    cohort: Cohort
    environment: Environment
    profile: NetworkProfile
    workload: Workload
    subscribers: int
    repetition: int
    warmup_s: float
    duration_s: float
    drain_s: float
    seed: int

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["stack"] = self.stack.value
        value["cohort"] = self.cohort.value
        value["environment"] = self.environment.value
        value["profile"] = self.profile.value
        value["workload"]["topics"] = list(value["workload"]["topics"])
        for topic in value["workload"]["topics"]:
            topic["cohort"] = topic["cohort"].value
        return value


def trial_spec_from_dict(value: dict[str, Any]) -> TrialSpec:
    workload_value = value["workload"]
    workload = Workload(
        name=workload_value["name"],
        topics=tuple(
            TopicWorkload(
                name=topic["name"],
                payload_bytes=topic["payload_bytes"],
                rate_hz=topic["rate_hz"],
                cohort=Cohort(topic["cohort"]),
                message_kind=topic["message_kind"],
            )
            for topic in workload_value["topics"]
        ),
        saturation=workload_value["saturation"],
        saturation_max_messages=workload_value["saturation_max_messages"],
        saturation_max_bytes=workload_value["saturation_max_bytes"],
    )
    return TrialSpec(
        trial_id=value["trial_id"],
        stack=Stack(value["stack"]),
        cohort=Cohort(value["cohort"]),
        environment=Environment(value["environment"]),
        profile=NetworkProfile(value["profile"]),
        workload=workload,
        subscribers=value["subscribers"],
        repetition=value["repetition"],
        warmup_s=value["warmup_s"],
        duration_s=value["duration_s"],
        drain_s=value["drain_s"],
        seed=value["seed"],
    )


@dataclass(frozen=True)
class MessageSample:
    trial_id: str
    topic: str
    receiver: int
    sequence: int
    payload_bytes: int
    scheduled_ns: int
    publish_start_ns: int
    publish_end_ns: int
    received_ns: int | None = None
    delivery_count: int = 0
    out_of_order: bool = False

    @property
    def publish_call_ns(self) -> int:
        return self.publish_end_ns - self.publish_start_ns

    @property
    def latency_ns(self) -> int | None:
        if self.received_ns is None:
            return None
        return self.received_ns - self.publish_start_ns

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ProcessUsage:
    role: str
    cpu_seconds: float
    peak_rss_bytes: int
    voluntary_context_switches: int = 0
    involuntary_context_switches: int = 0


@dataclass
class TrialRecord:
    spec: TrialSpec
    samples: list[MessageSample] = field(default_factory=list)
    usage: list[ProcessUsage] = field(default_factory=list)
    readiness_s: float = 0.0
    drain_s: float = 0.0
    measurement_s: float | None = None
    error: str | None = None
