#!/usr/bin/env python3
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

"""Integration test for the swarm fleet bus across a namespace boundary.

The PX4 swarm depends on two *exposed* streams doing real work at runtime:

* ``drone_state`` — N namespaced drones publish, one shared coordinator listens
  (fan-in, N->1).
* ``swarm_cmd``   — one shared coordinator publishes, N namespaced drones listen
  (fan-out, 1->N).

Unit tests can only prove the blueprint *wiring* is right. This one actually
stands a coordinator up and checks messages traverse the namespace boundary in
both directions, because a silent failure here looks exactly like "the drones
ignored the command".

Stand-in modules are used rather than the real Px4DroneModule/SwarmCoordinator
so the test needs no MAVLink link and no running sim.
"""

from __future__ import annotations

import time
from typing import Any

from dimos_lcm.std_msgs import String
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out

# The two streams the swarm leaves unprefixed so they cross the boundary.
BUS = {"swarm_cmd", "drone_state"}

FLEET_SIZE = 3
SETTLE_SEC = 2.0
DELIVERY_TIMEOUT_SEC = 15.0


class BusLeaf(Module):
    """Stand-in for Px4DroneModule: listens on swarm_cmd, publishes drone_state."""

    swarm_cmd: In[String]
    drone_state: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._got: list[str] = []

    @rpc
    def start(self) -> None:
        super().start()
        if getattr(self.swarm_cmd, "transport", None):
            self.swarm_cmd.subscribe(self._on_cmd)

    def _on_cmd(self, msg: String) -> None:
        self._got.append(msg.data)

    @rpc
    def received(self) -> list[str]:
        """Read back over RPC — the callback runs in this module's worker process."""
        return list(self._got)

    @rpc
    def swarm_cmd_has_transport(self) -> bool:
        return getattr(self.swarm_cmd, "_transport", None) is not None

    @rpc
    def report_state(self, text: str) -> str:
        self.drone_state.publish(String(text))
        return "published"


class BusHub(Module):
    """Stand-in for SwarmCoordinator: publishes swarm_cmd, listens on drone_state."""

    drone_state: In[String]
    swarm_cmd: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._got: list[str] = []

    @rpc
    def start(self) -> None:
        super().start()
        if getattr(self.drone_state, "transport", None):
            self.drone_state.subscribe(self._on_state)

    def _on_state(self, msg: String) -> None:
        self._got.append(msg.data)

    @rpc
    def received(self) -> list[str]:
        return list(self._got)

    @rpc
    def swarm_cmd_has_transport(self) -> bool:
        return getattr(self.swarm_cmd, "_transport", None) is not None

    @rpc
    def broadcast(self, text: str) -> str:
        self.swarm_cmd.publish(String(text))
        return "published"


def _swarm_bus_blueprint():
    return autoconnect(
        BusHub.blueprint(),
        *[
            BusLeaf.blueprint().namespace(f"drone{i + 1}", expose=BUS)
            for i in range(FLEET_SIZE)
        ],
    )


def _wait_for(predicate, timeout: float = DELIVERY_TIMEOUT_SEC) -> bool:
    """Poll until predicate() is truthy. Pub/sub delivery is asynchronous."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.25)
    return False


@pytest.fixture
def bus_coordinator():
    blueprint = _swarm_bus_blueprint()
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={}, overrides={"g": {"viewer": "none"}}
    )
    coordinator = ModuleCoordinator.build(blueprint, parsed)
    # Let every module's start() run and its subscriptions land before publishing.
    time.sleep(SETTLE_SEC)
    try:
        yield coordinator
    finally:
        coordinator.stop()


def test_exposed_out_is_actually_bound_to_a_transport(bus_coordinator):
    """A silently-unbound Out would make every fleet command a no-op."""
    hub = bus_coordinator.get_instance("bushub")
    assert hub.swarm_cmd_has_transport() is True
    for i in range(FLEET_SIZE):
        leaf = bus_coordinator.get_instance(f"drone{i + 1}/busleaf")
        assert leaf.swarm_cmd_has_transport() is True


def test_shared_publisher_fans_out_to_every_namespaced_listener(bus_coordinator):
    """swarm_cmd: 1 -> N across the namespace boundary (takeoff_all, rtl_all, ...)."""
    hub = bus_coordinator.get_instance("bushub")
    hub.broadcast("takeoff-all")

    leaves = [bus_coordinator.get_instance(f"drone{i + 1}/busleaf") for i in range(FLEET_SIZE)]
    assert _wait_for(lambda: all("takeoff-all" in leaf.received() for leaf in leaves)), (
        "swarm_cmd did not reach every drone: "
        + repr([leaf.received() for leaf in leaves])
    )


def test_namespaced_publishers_fan_in_to_the_shared_listener(bus_coordinator):
    """drone_state: N -> 1 across the namespace boundary (fleet_state)."""
    for i in range(FLEET_SIZE):
        bus_coordinator.get_instance(f"drone{i + 1}/busleaf").report_state(f"state-{i + 1}")

    hub = bus_coordinator.get_instance("bushub")
    expected = {f"state-{i + 1}" for i in range(FLEET_SIZE)}
    assert _wait_for(lambda: expected <= set(hub.received())), (
        f"coordinator did not see every drone's state: {hub.received()!r}"
    )
