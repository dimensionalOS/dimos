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

"""Zenoh's own loopback scouting regression tests, ported from zenoh/tests/scouting.rs.

Upstream: Fix multicast scouting on loopback#2671.
"""

from collections.abc import Callable
import json
import time

import zenoh


def _config(**settings: object) -> zenoh.Config:
    config = zenoh.Config()
    for key, value in settings.items():
        config.insert_json5(key.replace("__", "/"), json.dumps(value))
    return config


def _loopback_multicast(address: str) -> dict[str, object]:
    return {
        "scouting__gossip__enabled": False,
        "scouting__multicast__enabled": True,
        "scouting__multicast__address": address,
        "scouting__multicast__interface": "127.0.0.1",
    }


def test_multicast_scouting_works_on_loopback(
    unused_udp_port_factory: Callable[[], int],
) -> None:
    address = f"224.0.0.224:{unused_udp_port_factory()}"
    responder = zenoh.open(
        _config(
            mode="router",
            listen__endpoints=[],
            scouting__multicast__autoconnect=[],
            **_loopback_multicast(address),
        )
    )
    scout = zenoh.scout(what="router", config=_config(**_loopback_multicast(address)))
    try:
        deadline = time.monotonic() + 5.0
        hello = scout.try_recv()
        while hello is None and time.monotonic() < deadline:
            time.sleep(0.05)
            hello = scout.try_recv()
        assert hello is not None, "timed out waiting for multicast scout Hello"
        assert hello.whatami == zenoh.WhatAmI.ROUTER
        assert str(hello.zid) == str(responder.info.zid())
    finally:
        scout.stop()
        responder.close()


def test_multicast_autoconnect_works_on_loopback(
    unused_udp_port_factory: Callable[[], int],
) -> None:
    address = f"224.0.0.224:{unused_udp_port_factory()}"
    peer1 = zenoh.open(_config(mode="peer", **_loopback_multicast(address)))
    peer2 = zenoh.open(_config(mode="peer", **_loopback_multicast(address)))
    try:
        zid2 = str(peer2.info.zid())
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and zid2 not in map(str, peer1.info.peers_zid()):
            time.sleep(0.05)
        assert zid2 in map(str, peer1.info.peers_zid()), (
            "timed out waiting for loopback scout connection"
        )
    finally:
        peer2.close()
        peer1.close()
