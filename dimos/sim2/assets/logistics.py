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

"""The room a sim2 blueprint runs in when nobody has a real scene to hand: walls, shelves, a
table.

USD, converted once from the MJCF it used to be and committed beside this file. Its lighting is
written in by hand, because the converter drops lights outright ("lights are not supported").

    MuJoCo: 1 plane, 8 boxes, no joints and nothing that moves
    dimos:  root link "logistics", which is the frame a visitor hangs off
"""

from pathlib import Path

LOGISTICS = Path(__file__).parent / "logistics.usda"
