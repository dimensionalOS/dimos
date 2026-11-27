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

"""How long replay tests wait, from what they wait for."""

# The longest whole-machine stall seen on a CI runner: all four xdist workers froze
# for 1.2 s right after a replay pinned its clock (the G1 replay flake). Doubled.
CI_STALL_S = 2.4

# The longest a test recording takes to play: the Go2 loop test's third frame lands
# after two 1 s frames and the 0.05 s loop gap.
LONGEST_TEST_PLAYBACK_S = 2.05

# Until a replay delivers its frames or finishes: its playback, plus a stall.
PLAYBACK_TIMEOUT = LONGEST_TEST_PLAYBACK_S + CI_STALL_S

# Until another thread reaches a point, or a stop returns: no playback, only a stall.
HANDOFF_TIMEOUT = CI_STALL_S
