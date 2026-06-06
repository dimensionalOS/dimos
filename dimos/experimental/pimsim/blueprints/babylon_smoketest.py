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

"""Minimal browser-physics sim: box-proxy robot + rust lidar, no MuJoCo runtime.

A box-proxy robot whose kinematic base is integrated in the browser from
incoming ``/nav_cmd_vel``, plus the rust ``SceneLidarModule`` raycasting the
cooked scene collision mesh (default ``dimos-office``). Stripped of whole-body
control, camera streaming, and policy sims.

Usage::

    dimos run babylon-smoketest

Then open http://localhost:8091/ (or drive headless via
``dimos.experimental.pimsim.headless.HeadlessBrowser`` +
``dimos.experimental.pimsim.client.PimSimClient``). The scene is resolved at
import; ``build_babylon_sim`` lives in ``._factory`` for reuse without that.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.experimental.pimsim.blueprints._factory import build_babylon_sim

# Wrapped in autoconnect so the all_blueprints generator (which detects
# autoconnect/blueprint-method call chains, not bare factory calls) registers it.
babylon_smoketest = autoconnect(build_babylon_sim())

__all__ = ["babylon_smoketest"]
