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

"""Input regimes the follower runs under.

A track fixes what the follower is allowed to know; the CLI, the blueprint and
the deployed adapter all name a TRACK — never a law — so folding in a research
generation is a one-line change here.

Blind is the honest deployment case: the robot's own local map is the
planner's, and a follower that needs it back has to be handed geometry it did
not compute. What survives instead is the planner's precision profile, stamped
into the path's timestamps (``control/profile.py``), which is why blind is a
real track and not a handicap.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from dimos.navigation.motion.control.laws import blind, hinted


@dataclass(frozen=True)
class Track:
    name: str
    controller: Callable[..., Any]  # factory of the law this track currently runs
    annotate_clearance: bool  # is the follower handed the clearance array


TRACKS: dict[str, Track] = {
    # The python law is the default so nothing needs the crate built; its rust
    # twin (`:make_rust`) is parity-locked, and the native host runs the rust
    # directly (adapter/rust/src/follower.rs::Track).
    "hinted": Track("hinted", hinted.make, True),
    "blind": Track("blind", blind.make, False),
}

DEFAULT = "hinted"
