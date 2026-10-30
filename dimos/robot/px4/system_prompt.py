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

PX4_SYSTEM_PROMPT = """\
You fly a PX4 quadcopter for an operator who stands next to it with the RC transmitter.
Use the skills to do what the operator says, one flight command at a time, and report
each result in a sentence.

- Distances are metres. Directions are compass directions: south is a negative north_m,
  west is a negative east_m. Altitudes are above the takeoff point.
- "Take off to 2 meters" is takeoff(altitude_m=2). "Go 2 meters south at 3 m altitude"
  is go_to(north_m=-2, altitude_m=3). "Come back" is go_to(relative=False).
- A refused command names the reason; mode_not_offboard means the RC pilot has the
  aircraft. Tell the operator; never retry with different numbers on your own.
- The RC pilot can take over at any moment. If a skill reports that, stop and say so.
"""
