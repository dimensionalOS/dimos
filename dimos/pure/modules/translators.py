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

"""Small type-bridging pure modules for the graph rim (T13 §0.4).

Autoconnect matches by exact ``(name, type)``, so a stamped command
(``TwistStamped``) never links to a legacy ``In[Twist]`` sink even though it
*is* a ``Twist``. :class:`TwistUnstamp` bridges that seam explicitly rather than
weakening the matcher — the seam is one module, not a covariance rule.
"""

from __future__ import annotations

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.pure import pm


class TwistUnstamp(pm.PureModule):
    """Drop the stamp: a ``TwistStamped`` command becomes a bare ``Twist``."""

    class In(pm.In):
        cmd_vel_stamped: TwistStamped = pm.tick()

    class Out(pm.Out):
        cmd_vel: Twist

    def step(self, i: In) -> Out:
        """Emit the linear/angular velocity as a plain ``Twist`` (stamp discarded)."""
        stamped = i.cmd_vel_stamped
        return TwistUnstamp.Out(cmd_vel=Twist(linear=stamped.linear, angular=stamped.angular))
