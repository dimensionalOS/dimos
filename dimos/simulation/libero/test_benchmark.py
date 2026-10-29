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

from pathlib import Path

from dimos.simulation.libero.benchmark import bddl_language, bddl_object_types

BDDL = """(define (problem LIBERO_Tabletop_Manipulation)
  (:domain robosuite)
  (:language open the middle
     drawer of the cabinet)
  (:fixtures
    main_table - table
    wooden_cabinet_1 - wooden_cabinet
  )
  (:objects
    akita_black_bowl_1 - akita_black_bowl
    red_box_1 - red_box
  )
  (:init
    (On akita_black_bowl_1 main_table_bowl_region)
  )
  (:goal
    (And (Open wooden_cabinet_1_middle_region))
  )
)
"""


def test_bddl_language_joins_wrapped_lines(tmp_path: Path) -> None:
    bddl = tmp_path / "task.bddl"
    bddl.write_text(BDDL)
    assert bddl_language(bddl) == "open the middle drawer of the cabinet"


def test_bddl_object_types_covers_objects_and_fixtures(tmp_path: Path) -> None:
    bddl = tmp_path / "task.bddl"
    bddl.write_text(BDDL)
    assert bddl_object_types(bddl) == {"table", "wooden_cabinet", "akita_black_bowl", "red_box"}
