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

"""Assemble existing MJCF assets without creating a running physics world."""

from collections.abc import Mapping
from copy import deepcopy
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from robosuite.models.robots.robot_model import RobotModel
from robosuite.models.tasks.task import Task
from robosuite.utils.mjcf_utils import array_to_string

from dimos.sim2.arena import AuthoredArena


def assemble_mjcf(
    arena: AuthoredArena,
    models: Mapping[str, RobotModel],
    *,
    hidden_geom_groups: tuple[int, ...] = (),
) -> Task:
    """Compose prepared models while preserving authored scene metadata and appearance."""
    task = Task(arena, list(models.values()))
    # Task merges physical model sections but drops authored custom metadata.
    authored_custom = arena.root.find("custom")
    if authored_custom is not None:
        custom = task.root.find("custom")
        if custom is None:
            task.root.append(deepcopy(authored_custom))
        else:
            custom.extend(deepcopy(list(authored_custom)))
    option = task.root.find("option")
    if option is None:
        option = ET.SubElement(task.root, "option")
    authored_option = arena.root.find("option")
    if authored_option is not None:
        option.attrib.update(authored_option.attrib)
        option.extend(deepcopy(list(authored_option)))
    visual = task.root.find("visual")
    if visual is not None:
        task.root.remove(visual)
    authored_visual = arena.root.find("visual")
    task.root.append(
        deepcopy(authored_visual) if authored_visual is not None else ET.Element("visual")
    )
    for geom in task.worldbody.iter("geom"):
        if int(geom.get("group", "0")) in hidden_geom_groups:
            color = np.fromstring(geom.get("rgba", ".5 .5 .5 1"), sep=" ")
            color[3] = 0.0
            geom.set("rgba", array_to_string(color))
    return task


def lower_genesis_contacts(task: Task, model: Any) -> None:
    """Preserve MJCF explicit-pair eligibility in Genesis's bitmask importer.

    Genesis 1.4 imports collision masks but ignores the MJCF pair table. Give
    each explicit pair an unused directed bit; this enables exactly that pair
    without turning visual geometry into broad collision geometry.
    Pair-specific friction/solver overrides remain unsupported by Genesis.
    """
    occupied = int(np.bitwise_or.reduce(model.geom_contype | model.geom_conaffinity))
    bit = occupied.bit_length()
    for first, second in zip(model.pair_geom1, model.pair_geom2, strict=True):
        if bit >= 31:
            raise NotImplementedError("explicit contacts exceed Genesis's collision-mask capacity")
        first_geom = task.worldbody.find(f".//geom[@name='{model.geom(first).name}']")
        second_geom = task.worldbody.find(f".//geom[@name='{model.geom(second).name}']")
        if first_geom is None or second_geom is None:
            raise ValueError("explicit contact pair must reference named geometry")
        first_geom.set(
            "contype",
            str(int(first_geom.get("contype", str(model.geom_contype[first]))) | (1 << bit)),
        )
        second_geom.set(
            "conaffinity",
            str(
                int(second_geom.get("conaffinity", str(model.geom_conaffinity[second])))
                | (1 << bit)
            ),
        )
        bit += 1
