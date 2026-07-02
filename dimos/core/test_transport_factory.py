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

from dimos.core.global_config import GlobalConfig
from dimos.core.transport import (
    LCMTransport,
    ZenohTransport,
    pLCMTransport,
    pZenohTransport,
)
from dimos.core.transport_factory import (
    apply_transport_arg,
    make_transport,
    rpc_backend,
    tf_backend,
    transport_topic,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.rpc.pubsubrpc import LCMRPC, ZenohRPC
from dimos.protocol.tf.tf import LCMTF, ZenohTF

LCM = GlobalConfig(transport="lcm")
ZENOH = GlobalConfig(transport="zenoh")


def test_transport_topic_lcm() -> None:
    # LCM channels are leading-slash; either input form normalizes the same.
    assert transport_topic("/human_input", LCM) == "/human_input"
    assert transport_topic("human_input", LCM) == "/human_input"


def test_transport_topic_zenoh() -> None:
    # Zenoh keyexprs can't start with '/'; namespaced under 'dimos/'.
    assert transport_topic("/human_input", ZENOH) == "dimos/human_input"
    assert transport_topic("human_input", ZENOH) == "dimos/human_input"
    assert transport_topic("/coordinator/joint_state", ZENOH) == "dimos/coordinator/joint_state"


def test_make_transport_lcm_typed() -> None:
    t = make_transport("/camera/color", Image, g=LCM)
    assert type(t) is LCMTransport
    assert t.topic.topic == "/camera/color"


def test_make_transport_lcm_pickled() -> None:
    t = make_transport("/human_input", g=LCM)
    assert type(t) is pLCMTransport
    assert t.topic == "/human_input"


def test_make_transport_zenoh_typed() -> None:
    t = make_transport("/camera/color", Image, g=ZENOH)
    assert type(t) is ZenohTransport
    assert t.topic.topic == "dimos/camera/color"


def test_make_transport_zenoh_pickled() -> None:
    t = make_transport("/human_input", g=ZENOH)
    assert type(t) is pZenohTransport
    assert t.topic == "dimos/human_input"


def test_rpc_backend_resolves_per_transport() -> None:
    assert rpc_backend(LCM) is LCMRPC
    assert rpc_backend(ZENOH) is ZenohRPC


def test_tf_backend_resolves_per_transport() -> None:
    assert tf_backend(LCM) is LCMTF
    assert tf_backend(ZENOH) is ZenohTF


def test_zenoh_tf_config_topic_and_pubsub() -> None:
    from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
    from dimos.protocol.tf.tf import ZenohPubsubConfig

    cfg = ZenohPubsubConfig()
    assert cfg.topic.topic == "dimos/tf"
    assert cfg.pubsub is Zenoh


def test_zenoh_rpc_topicgen_has_no_leading_slash() -> None:
    assert ZenohRPC().topicgen("Hello/say", req_or_res=True).topic == "dimos/rpc/Hello/say/res"


def test_apply_transport_arg() -> None:
    g = GlobalConfig(transport="lcm")
    apply_transport_arg(["prog", "--transport", "zenoh"], g=g)
    assert g.transport == "zenoh"
    apply_transport_arg(["prog", "--transport=lcm"], g=g)
    assert g.transport == "lcm"
    apply_transport_arg(["prog", "--other", "x"], g=g)  # no flag -> unchanged
    assert g.transport == "lcm"


@pytest.mark.parametrize(
    "argv",
    [
        ["prog", "--transport", "zeno"],  # typo
        ["prog", "--transport=zeno"],
        ["prog", "--transport"],  # missing value
        ["prog", "--transport", "--web"],  # next flag must not be consumed as value
    ],
)
def test_apply_transport_arg_bad_value_exits(argv: list[str]) -> None:
    g = GlobalConfig(transport="lcm")
    with pytest.raises(SystemExit) as exc:
        apply_transport_arg(argv, g=g)
    assert exc.value.code == 2
    assert g.transport == "lcm"  # config left untouched
