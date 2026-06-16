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

"""Combined G1 GR00T sim + manipulation planner — click-to-grasp end to end.

Composes two existing blueprints, adding no new module logic:

* ``g1-groot-wbc`` — the WBC sim. Owns physics, publishes ``/odom`` and the
  ``/entity_state_batch`` entity stream, opens the Babylon viewer, and (via the
  arm trajectory tasks now registered on its coordinator) executes planned arm
  motions on ``traj_left_arm`` / ``traj_right_arm``.
* ``g1-office-planner`` — the ``G1ManipulationModule`` planner. Loads the same
  ``dimos-office`` scene into a MujocoWorld, tracks the pelvis off ``/odom`` and
  obstacles off ``/entity_state_batch``, and plans collision-free arm motions.

The two are bound by topic: the planner consumes what the sim publishes, and its
planned trajectory executes back through the sim's coordinator. The result is a
single runnable process where a world point published on ``/grasp_goal``
resolves (privileged, from the entity stream) to the nearest scene object and
the left arm plans, reaches, and holds at it — the source-blind reach that the
real robot will share, only swapping the privileged pose source for a perceived
one.

    dimos --simulation mujoco run g1-office-grasp

Drive a grasp by publishing a world ``PointStamped`` on ``/grasp_goal`` (or call
the ``grasp_object`` skill by id). ``/point_goal`` still drives ``point_at``.
A live Babylon click→grasp button would publish ``/grasp_goal`` from the viewer;
``/clicked_point`` is intentionally left to the nav stack.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.blueprints import g1_office_planner
from dimos.robot.unitree.g1.blueprints.basic.g1_groot_wbc import g1_groot_wbc

g1_office_grasp = autoconnect(g1_groot_wbc, g1_office_planner)

__all__ = ["g1_office_grasp"]
