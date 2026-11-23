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

"""Lower the held pen to the paper with you watching: `start`, then `down` (3 mm) until
you see it touch, then `touching` saves that height for the drawing.

    ~/dimos-franka/.venv/bin/python lower_pen.py start | down | touching
"""

import sys

import numpy as np

import handoff as ho
import pen_demo as pd

STEP = float(sys.argv[2]) / 1000 if len(sys.argv) > 2 else 0.003
arms = ho.Arms(dry=False)
try:
    st = pd.load()
    plan, d, paper = st["plan"], st["drawer"], st["paper"]
    cmd = sys.argv[1]
    if cmd == "start":
        T = pd.ho_tcp(arms, d)
        tip = pd.tip_of(T, st["pen"], plan["e"])
        first = np.asarray(pd.smiley(paper)[0][0])
        here = arms.tcp(d)
        # Over the first stroke, the TCP at the height the model said touches.
        up = max(0.0, float(st["z_touch"] - here[2]))
        if up > 0.005:
            arms.straight(d, up, "up to the touch height", pd.AWAY)
        arms.along(
            d, [first[0] - tip[0], first[1] - tip[1], 0.0], "over the drawing's start", pd.NEAR
        )
        pd.settle_z(arms, d, st["z_touch"], "the touch height")
        st["z_probe"] = float(arms.tcp(d)[2])
    elif cmd == "down":
        target = st["z_probe"] - STEP
        pd.settle_z(arms, d, target, f"{STEP * 1000:.0f} mm lower")
        st["z_probe"] = target
    elif cmd == "touching":
        st["z_touch"] = st["z_probe"]
        print(f"  the drawing height is now TCP {st['z_touch'] * 1000:.1f} mm")
    pd.save(st)
    print(
        f"  TCP now {float(arms.tcp(d)[2]) * 1000:.1f} mm; asked {st.get('z_probe', 0) * 1000:.1f} mm "
        f"({(st['z_touch'] - st.get('z_probe', st['z_touch'])) * 1000:.0f} mm below the model's touch height)"
    )
except pd.Failed as e:
    print("STOPPED:", e)
finally:
    arms.close()
