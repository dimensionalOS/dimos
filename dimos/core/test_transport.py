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


from dataclasses import dataclass

import pytest

from dimos.core.stream import TransportSpec
from dimos.core.transport import (
    DDS_AVAILABLE,
    DDSTransport,
    JpegLcmTransport,
    LCMTransport,
    pLCMTransport,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.encode.cbor import decode, encode

if DDS_AVAILABLE:
    # cyclonedds is the optional dds extra; see DDS_AVAILABLE.
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.types import uint8

    @dataclass
    class DDSSample(IdlStruct):  # type: ignore[misc]
        value: uint8  # type: ignore[valid-type]


@pytest.mark.parametrize("transport_class", [pLCMTransport, LCMTransport, JpegLcmTransport])
def test_lcm_transport_roundtrip_preserves_connection_settings(transport_class):
    args = () if transport_class is pLCMTransport else (Image,)
    transport = transport_class("/roundtrip", *args, url="udpm://239.255.76.68:7798?ttl=2", ttl=2)
    try:
        restored = decode(encode(transport))
        try:
            assert type(restored) is transport_class
            assert restored.topic == transport.topic
            assert restored.lcm.config == transport.lcm.config
        finally:
            restored.stop()
    finally:
        transport.stop()


def test_with_topic_keeps_every_other_setting() -> None:
    transport = LCMTransport("/a", Image, url="udpm://239.255.76.68:7798?ttl=2", ttl=2)
    try:
        moved = transport.with_topic("/b")
        try:
            assert type(moved) is LCMTransport
            assert moved.topic.topic == "/b"
            assert moved.topic.lcm_type is Image
            assert moved.lcm.config == transport.lcm.config
        finally:
            moved.stop()
    finally:
        transport.stop()


def test_transport_spec_of_a_live_transport_decodes_without_building_it(mocker) -> None:
    transport = LCMTransport("/described", Image, url="udpm://239.255.76.68:7798?ttl=2", ttl=2)
    try:
        spec = TransportSpec.of(transport)
        assert spec.cls is LCMTransport
        assert spec.kwargs["topic"] == "/described"

        lcm_ctor = mocker.patch("dimos.protocol.service.lcmservice.lcm_mod.LCM")
        restored = decode(encode([spec] * 30))  # stream descriptors carry these
        lcm_ctor.assert_not_called()  # no sockets: descriptions are data
        mocker.stop(lcm_ctor)

        built = restored[0].build()
        try:
            assert type(built) is LCMTransport
            assert built.topic == transport.topic
            assert built.lcm.config == transport.lcm.config
        finally:
            built.stop()
    finally:
        transport.stop()


@pytest.mark.skipif(not DDS_AVAILABLE, reason="cyclonedds is not installed")
def test_dds_transport_roundtrip_preserves_qos_and_domain() -> None:
    from cyclonedds.qos import Policy, Qos  # cyclonedds: see DDS_AVAILABLE

    qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(depth=1))
    transport = DDSTransport("/dds_roundtrip", DDSSample, qos=qos, domain_id=3)
    restored = decode(encode(transport))
    assert type(restored) is DDSTransport
    assert restored.topic == transport.topic
    assert restored.dds.qos.asdict() == qos.asdict()
    assert restored.dds.config.domain_id == 3
