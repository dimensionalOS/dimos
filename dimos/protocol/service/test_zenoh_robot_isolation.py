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

"""Two robots on one network do not see each other's traffic.

Key expressions carry no robot identity, so isolation rests entirely on the topology: one
router per robot, and nothing that routes between them. This stands both robots up on
loopback and checks that a command reaches the robot it was aimed at and no other.

The go2web bridge is modeled with a raw zenoh session rather than a `ZenohService`, because
that is what it is: its own session, always a peer, with multicast scouting always on. The
two bridges therefore find each other, which is the hazard under test.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
import json
import socket
import time
from typing import cast

import pytest
import zenoh

# The host is built from the session bake emits, not from a topology written out here, so
# this fails if the shipped config ever drifts from the one under test.
from dimos.cli.bake.cli import session_settings
from dimos.protocol.service.zenohservice import ZenohService, ZenohSessionPool

# Not `dimos/`, so a stray run of this test cannot reach a real stack's topics.
KEY_CMD_VEL = "dimos_test/isolation/cmd_vel"
KEY_ODOM = "dimos_test/isolation/odometry"

# Zenoh's own scouting port is 7446. Scouting somewhere else keeps these bridges from
# joining whatever else is on the machine, including a live robot session.
SCOUT_ADDRESS = "224.0.0.224:17446"

SETTLE = 2.0
DEADLINE = 10.0


def _free_endpoint() -> str:
    """A loopback endpoint nothing holds. A fixed port collides between runs."""
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return f"tcp/127.0.0.1:{sock.getsockname()[1]}"


@dataclass
class Robot:
    """One robot: its go2web bridge and the baked host that routes for it."""

    bridge: zenoh.Session
    host: ZenohService
    host_endpoint: str


def _bridge_session(listen: str) -> zenoh.Session:
    """A go2web bridge: peer mode, multicast scouting on, no way to configure either."""
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps("peer"))
    config.insert_json5("listen/endpoints", json.dumps([listen]))
    config.insert_json5("scouting/multicast/enabled", json.dumps(True))
    config.insert_json5("scouting/multicast/address", json.dumps(SCOUT_ADDRESS))
    config.insert_json5("scouting/multicast/interface", json.dumps("lo"))
    return zenoh.open(config)


def _baked_host(pool: ZenohSessionPool, listen: str, bridge: str) -> ZenohService:
    """A baked host, configured by the same code that writes the robot's stdin config."""
    wire = session_settings(mode="router", listen=[listen], connect=[bridge])
    # Containment, and the drift check: bake pins multicast off, and were that to change
    # these hosts would scout zenoh's real address and join whatever is live on the machine.
    assert wire["multicast"] is False, "bake now emits multicast scouting for a baked host"
    return ZenohService(
        session_pool=pool,
        mode=wire["mode"],
        connect=wire["connect"],
        listen=wire["listen"],
        multicast=wire["multicast"],
        gossip=wire["gossip"],
        scouting_interface=wire["interface"],
        connect_timeout=cast("int", wire["connect_timeout_ms"]) / 1000,
    )


def _collect(session: zenoh.Session, key: str) -> list[str]:
    received: list[str] = []
    session.declare_subscriber(key, lambda sample: received.append(sample.payload.to_string()))
    return received


def _publish_until(session: zenoh.Session, key: str, payload: str, seen: list[str]) -> None:
    """Republish until the subscriber declarations have propagated, then once more."""
    deadline = time.monotonic() + DEADLINE
    while not seen and time.monotonic() < deadline:
        session.put(key, payload)
        time.sleep(0.05)
    session.put(key, payload)
    time.sleep(0.5)


@pytest.fixture
def pool() -> Iterator[ZenohSessionPool]:
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


@pytest.fixture
def robots(zenoh_defaults: None, pool: ZenohSessionPool) -> Iterator[tuple[Robot, Robot]]:
    """Two robots, each a bridge plus a baked host routing for it."""
    endpoints = [_free_endpoint() for _ in range(2)]
    bridges = [_bridge_session(endpoint) for endpoint in endpoints]
    try:
        built = []
        for bridge, bridge_endpoint in zip(bridges, endpoints, strict=True):
            host_endpoint = _free_endpoint()
            host = _baked_host(pool, host_endpoint, bridge_endpoint)
            host.start()
            built.append(Robot(bridge=bridge, host=host, host_endpoint=host_endpoint))
        time.sleep(SETTLE)
        yield built[0], built[1]
    finally:
        for bridge in bridges:
            bridge.close()


def _laptop(pool: ZenohSessionPool, robot: Robot) -> ZenohService:
    """A `dimos run` scoped to one robot, dialing that robot's baked host router."""
    laptop = ZenohService(
        session_pool=pool, mode="client", connect=[robot.host_endpoint], multicast=False
    )
    laptop.start()
    return laptop


def _peers(session: zenoh.Session) -> set[str]:
    return {str(zid) for zid in session.info.peers_zid()}


def test_the_bridges_do_find_each_other(robots: tuple[Robot, Robot]) -> None:
    """Without this the isolation tests below would pass for the wrong reason."""
    alpha, bravo = robots
    assert _peers(alpha.bridge) == {str(bravo.bridge.info.zid())}
    assert _peers(bravo.bridge) == {str(alpha.bridge.info.zid())}


def test_the_laptop_attaches_to_the_baked_host_router(
    pool: ZenohSessionPool, robots: tuple[Robot, Robot]
) -> None:
    """A client dialing the bridge by mistake also connects, and then sees no maps."""
    alpha, _ = robots
    laptop = _laptop(pool, alpha)
    routers = {str(zid) for zid in laptop.session.info.routers_zid()}
    assert routers == {str(alpha.host.session.info.zid())}


def test_a_command_reaches_only_the_robot_it_was_aimed_at(
    pool: ZenohSessionPool, robots: tuple[Robot, Robot]
) -> None:
    alpha, bravo = robots
    laptop = _laptop(pool, alpha)

    driven_alpha = _collect(alpha.bridge, KEY_CMD_VEL)
    driven_bravo = _collect(bravo.bridge, KEY_CMD_VEL)
    time.sleep(0.5)

    _publish_until(laptop.session, KEY_CMD_VEL, "drive-alpha", driven_alpha)

    assert driven_alpha, "the robot the laptop is scoped to never got the command"
    assert driven_bravo == [], "a command aimed at one robot reached the other"


def test_each_side_sees_only_its_own_robots_odometry(
    pool: ZenohSessionPool, robots: tuple[Robot, Robot]
) -> None:
    alpha, bravo = robots
    laptop = _laptop(pool, alpha)

    on_laptop = _collect(laptop.session, KEY_ODOM)
    on_host_alpha = _collect(alpha.host.session, KEY_ODOM)
    on_host_bravo = _collect(bravo.host.session, KEY_ODOM)
    time.sleep(0.5)

    _publish_until(bravo.bridge, KEY_ODOM, "odom-from-bravo", on_host_bravo)
    _publish_until(alpha.bridge, KEY_ODOM, "odom-from-alpha", on_host_alpha)

    assert set(on_host_alpha) == {"odom-from-alpha"}
    assert set(on_host_bravo) == {"odom-from-bravo"}
    assert set(on_laptop) == {"odom-from-alpha"}
