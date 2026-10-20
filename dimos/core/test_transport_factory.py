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

from contextlib import ExitStack
import pickle

from dimos_cdrgen.registry import encode
import pytest

import dimos.core.cdr_transport as cdr_module
from dimos.core.cdr_transport import CdrTransport
from dimos.core.global_config import GlobalConfig
from dimos.core.transport import (
    LCMTransport,
    ZenohTransport,
    pLCMTransport,
    pZenohTransport,
)
from dimos.core.transport_factory import (
    apply_transport_arg,
    default_zenoh_qos,
    make_transport,
    rpc_backend,
    session_config,
    transport_topic,
)
from dimos.messages.sensor_msgs.msg import Image as CdrImage
from dimos.messages.std_msgs.msg import Int32
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, QOS_NEVER_DROP
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service.lcmservice import LCMConfig
from dimos.protocol.service.zenohservice import ZenohConfig

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


def test_default_zenoh_qos_high_rate_sensor_types_drop() -> None:
    assert default_zenoh_qos("/camera/color", Image) == QOS_LATEST_WINS


def test_default_zenoh_qos_agent_channels_never_drop() -> None:
    assert default_zenoh_qos("/human_input") == QOS_NEVER_DROP
    assert default_zenoh_qos("/agent") == QOS_NEVER_DROP
    assert default_zenoh_qos("/agent_idle") == QOS_NEVER_DROP


def test_default_zenoh_qos_seed_channels_never_drop_despite_their_types() -> None:
    assert default_zenoh_qos("/loaded_map", PointCloud2) == QOS_NEVER_DROP
    assert default_zenoh_qos("/seed_map", PointCloud2) == QOS_NEVER_DROP
    assert default_zenoh_qos("/seed_bounds", PoseStamped) == QOS_NEVER_DROP


def test_default_zenoh_qos_everything_else_uses_zenoh_defaults() -> None:
    assert default_zenoh_qos("/cmd_vel", Twist) is None
    assert default_zenoh_qos("/tool_stream") is None


def test_make_transport_zenoh_typed_carries_qos() -> None:
    t = make_transport("/camera/color", Image, g=ZENOH)
    assert t.topic.qos == QOS_LATEST_WINS


def test_make_transport_zenoh_pickled_carries_qos() -> None:
    t = make_transport("/human_input", g=ZENOH)
    assert t._zenoh_topic.qos == QOS_NEVER_DROP


def test_rpc_backend_resolves_per_transport() -> None:
    assert rpc_backend(LCM) is LCMRPC
    assert rpc_backend(ZENOH) is ZenohRPC


def test_session_config_resolves_per_transport() -> None:
    assert type(session_config(ZENOH)) is ZenohConfig
    assert type(session_config(LCM)) is LCMConfig


def test_session_config_unknown_transport_raises() -> None:
    # The field validator rejects unknown backends, so bypass validation.
    g = GlobalConfig.model_construct(transport="carrier_pigeon")
    with pytest.raises(ValueError, match="carrier_pigeon"):
        session_config(g)


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


@pytest.mark.parametrize("config", [LCM, ZENOH])
def test_generated_factory_publishes_cdr_instead_of_pickle(config, mocker):
    boundary = "LCMPubSubBase" if config.transport == "lcm" else "ZenohPubSubBase"
    factory = mocker.patch.object(cdr_module, boundary, autospec=True)
    transport = make_transport("reading", Int32, g=config)
    with ExitStack() as resources:
        resources.callback(transport.stop)
        transport.publish(Int32(42))
        factory.return_value.publish.assert_called_once_with(transport.topic, encode(Int32(42)))
        prefix = "dimos/" if config.transport == "zenoh" else ""
        assert transport.channel == prefix + "reading/cdr/std_msgs/msg/Int32"
        restored = pickle.loads(pickle.dumps(transport))
        assert type(restored) is CdrTransport
        assert restored.channel == transport.channel


def test_generated_sensor_qos_survives_factory_and_pickle():
    sensor = make_transport("camera", CdrImage, g=ZENOH)
    seed = make_transport("loaded_map", CdrImage, g=ZENOH)
    assert sensor.publish_qos == QOS_LATEST_WINS.to_wire()
    assert pickle.loads(pickle.dumps(seed)).publish_qos == QOS_NEVER_DROP.to_wire()
    assert make_transport("camera", CdrImage, g=LCM).publish_qos is None


def test_unregistered_generated_claim_is_rejected_instead_of_pickled():
    class Unregistered:
        __msgtype__ = "std_msgs/msg/Int32"

    with pytest.raises(TypeError, match="registry-owned"):
        make_transport("reading", Unregistered, g=ZENOH)
