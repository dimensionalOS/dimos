#!/usr/bin/env python3

"""Basic Drone Sim Blueprint with sim connection and visualization"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.drone.blueprints.primitive.drone_primitive_no_nav import (
    drone_primitive_no_nav,
)
from dimos.robot.drone.sim import DroneSimConnection

drone_basic_sim = autoconnect(
    drone_primitive_no_nav,
    DroneSimConnection.blueprint(),
    ReplanningAStarPlanner.blueprint(),
)

__all__ = ["drone_basic_sim"]
