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

from __future__ import annotations

import pytest
import zenoh

from dimos.protocol.pubsub.impl.zenohpubsub import resolve_qos
from dimos.protocol.pubsub.impl.zenohqos import DEFAULT_ZENOH_QOS, ZenohQoS


def test_first_match_wins() -> None:
    rules = (
        ZenohQoS(key="dimos/a/**", reliability="best_effort"),
        ZenohQoS(key="dimos/**", reliability="reliable"),
    )
    assert resolve_qos("dimos/a/b", rules) == {"reliability": zenoh.Reliability.BEST_EFFORT}


def test_default_rpc_is_reliable_and_blocks() -> None:
    assert resolve_qos("dimos/rpc/Hello/say/req", DEFAULT_ZENOH_QOS) == {
        "reliability": zenoh.Reliability.RELIABLE,
        "congestion_control": zenoh.CongestionControl.BLOCK,
    }


def test_default_image_type_suffix_drops() -> None:
    assert resolve_qos("dimos/camera/color/sensor_msgs.Image", DEFAULT_ZENOH_QOS) == {
        "reliability": zenoh.Reliability.BEST_EFFORT,
        "congestion_control": zenoh.CongestionControl.DROP,
    }


def test_default_agent_channels_are_exact_keys() -> None:
    qos = resolve_qos("dimos/agent", DEFAULT_ZENOH_QOS)
    assert qos["congestion_control"] == zenoh.CongestionControl.BLOCK
    # A sibling channel does not inherit the agent rule.
    assert resolve_qos("dimos/agents", DEFAULT_ZENOH_QOS) == {}


def test_no_match_returns_empty() -> None:
    assert resolve_qos("dimos/odom/geometry_msgs.PoseStamped", DEFAULT_ZENOH_QOS) == {}


def test_partial_rule_omits_unset_fields() -> None:
    rules = (ZenohQoS(key="dimos/x", reliability="best_effort"),)
    assert resolve_qos("dimos/x", rules) == {"reliability": zenoh.Reliability.BEST_EFFORT}


def test_invalid_rule_key_raises() -> None:
    # Leading slashes are invalid zenoh key expressions; publish() logs this.
    rules = (ZenohQoS(key="/leading/slash"),)
    with pytest.raises(zenoh.ZError):
        resolve_qos("dimos/x", rules)
