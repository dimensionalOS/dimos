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

from dimos.core.blueprints import autoconnect, Blueprint
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

from dimos.control.coordinator import control_coordinator
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.control.coordinator import ControlCoordinator
from dimos.utils.logging_config import setup_logger
import traceback

logger = setup_logger()

# Coordinator blueprint
coordinator = control_coordinator(
    tick_rate=100.0,
    hardware=[],  # Will be added via auto-setup
    tasks=[],  # PathFollowerTask added via auto-setup
)

# Blueprint with coordinator-based path following
_base_blueprint = autoconnect(
    unitree_go2_basic,
    voxel_mapper(voxel_size=0.1),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
    coordinator,
).global_config(n_dask_workers=6, robot_model="unitree_go2")


def _auto_setup_coordinator_navigation(dimos_instance):
    """Auto-setup coordinator navigation after blueprint build.
    
    This wires the BaseTwistAdapter to GO2Connection automatically.
    """
    try:
        # Get module instances
        planner = dimos_instance.get_instance(ReplanningAStarPlanner)
        coordinator = dimos_instance.get_instance(ControlCoordinator)
        
        if planner is None:
            logger.error("ReplanningAStarPlanner not found")
            return False
        if coordinator is None:
            logger.error("ControlCoordinator not found")
            return False

        # Set coordinator in planner (adds PathFollowerTask)
        if not planner.set_coordinator(coordinator):
            logger.error("Failed to set coordinator in planner")
            return False
        
        # Set up base hardware adapter via planner RPC.
        # We pass None for twist_callback; the BaseTwistAdapterWrapper should
        # handle this by publishing directly to /cmd_vel.
        if not planner.setup_base_hardware_with_callback(coordinator, None):
            logger.error("Failed to set up base hardware via planner")
            return False

        # Verify coordinator is started
        coordinator.start()  # Ensure it's started
        
        logger.info("Coordinator navigation auto-setup completed successfully")
        return True
        
    except Exception as e:
        logger.error(f"Failed to auto-setup coordinator navigation: {e}")
        logger.error(traceback.format_exc())
        return False


class AutoSetupBlueprint:
    """Blueprint wrapper that auto-sets up coordinator navigation."""
    
    def __init__(self, blueprint: Blueprint):
        self._blueprint = blueprint
    
    def build(self, **kwargs):
        """Build blueprint and auto-setup coordinator navigation."""
        dimos = self._blueprint.build(**kwargs)
        _auto_setup_coordinator_navigation(dimos)
        return dimos
    
    def __getattr__(self, name):
        """Delegate all other attributes to wrapped blueprint."""
        return getattr(self._blueprint, name)


# Wrap the blueprint to auto-setup
unitree_go2 = AutoSetupBlueprint(_base_blueprint)

# Keep old name for backward compatibility (but it's the same now)
unitree_go2_with_coordinator = unitree_go2

__all__ = ["unitree_go2", "unitree_go2_with_coordinator"]