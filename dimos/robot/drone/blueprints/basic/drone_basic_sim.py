#!/usr/bin/env python3

"""Basic drone sim: visualization and mujoco connection."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.drone.blueprints.primitive.drone_primitive_no_nav import drone_primitive_no_nav
from dimos.robot.drone.mujoco_sim import DroneSimConnection

drone_basic_sim = autoconnect(
    drone_primitive_no_nav,
    DroneSimConnection.blueprint(),
)

__all__ = ["drone_basic_sim"]
