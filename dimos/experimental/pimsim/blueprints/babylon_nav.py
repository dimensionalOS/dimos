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

"""pimsim box + FAR nav stack, wired for cross-wall routing.

Composes the babylon box sim with the nav stack (TerrainAnalysis, LocalPlanner,
PathFollower, PGO, SimplePlanner) plus the adapters pimsim needs:
- ``PoseStampedToOdometry``: ``/odom`` -> ``/odometry``
- ``OdomTfBroadcaster``: TF ``map -> body`` for SimplePlanner
- ``MovementManager``: relays ``/clicked_point`` -> the planner goal

Runs on an open cooked floor so a spawned wall is the only obstacle. The
registered ``babylon-nav`` includes the rerun viewer; run it with rerun web::

    dimos run babylon-nav --rerun-web

then drive a goal/walls (PimSimClient) with a HeadlessBrowser connected. The
factories live in ``._factory`` so importing them (e.g. from the cross-wall
test) does not cook a scene at import.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.experimental.pimsim.blueprints._factory import build_babylon_nav

# Wrapped in autoconnect so the all_blueprints generator registers it (and so
# there's no redundant trailing .global_config — build_babylon_nav sets it).
babylon_nav = autoconnect(build_babylon_nav(with_vis=True))

__all__ = ["babylon_nav"]
