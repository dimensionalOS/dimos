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

"""Input regimes the executor is researched and judged under.

A track fixes what the follower is allowed to know and what pace it is held
to; the two are the same decision, so they live in one record. The CLI, the
episode config, the judge and the deployed adapter all name a TRACK — never a
law — so folding in a research generation is a one-line change here.

Blind is the honest deployment case: the robot's own local map is the
planner's, and a follower that needs it back has to be handed geometry it did
not compute. What survives instead is the planner's precision profile, stamped
into the path's timestamps (``control/profile.py``), which is why blind is a
real track and not a handicap.

Scores compare only WITHIN a track. Hinted is judged against the ~0.75 m/s the
plant reaches at the command ceiling because live clearance permits outrunning
the stamp encoding; blind may only decode the stamps (governor band 0.2-0.5),
so it is judged against the mid-band and its pace pillar is weighted down to
match how little of the score it can move.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Track:
    name: str
    controller: str  # REGISTRY name of the law this track currently runs
    annotate_clearance: bool  # is the follower handed the clearance array
    # Not read on this branch: the referee that scores pace lives on
    # ivan/feat/trajectory_ctrl. They are half of what a track IS (see above),
    # so they are written down here with the other half, not derived twice.
    cruise: float  # m/s to hold through curves
    pace_weight: float  # how much pace counts when this track is scored


TRACKS: dict[str, Track] = {
    # The python law is the default so nothing needs the crate built; its
    # rust twin ("seed-rs" / "blind-rs") is parity-locked and is what deploys.
    "hinted": Track("hinted", "hinted", True, 0.75, 5.0),
    "blind": Track("blind", "blind", False, 0.35, 0.5),
}

DEFAULT = "hinted"
