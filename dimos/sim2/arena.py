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

"""Import existing MJCF arenas without losing their authored MuJoCo defaults."""

from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import mujoco
from robosuite.models.arenas.arena import Arena

Defaults = dict[str, dict[str, dict[str, str]]]


class AuthoredArena(Arena):
    """An upstream Arena preserving native units, includes, assets and defaults.

    robosuite 1.5.1 drops the anonymous default class and does not inherit
    childclass. Resolve native syntax first, then expand those defaults through
    its existing XML-import hooks. There is no alternate scene model/schema.
    """

    def __init__(self, path: Path) -> None:
        source = path.resolve()
        spec = mujoco.MjSpec.from_file(str(source))
        for entries, directory in (
            (spec.meshes, spec.meshdir),
            (spec.textures, spec.texturedir),
            (spec.hfields, ""),
        ):
            for asset in entries:
                if asset.file:
                    asset.file = str((source.parent / directory / asset.file).resolve())
        spec.meshdir = ""
        spec.texturedir = ""
        for key in list(spec.keys):
            spec.delete(key)
        with TemporaryDirectory(prefix="dimos-arena-") as directory:
            expanded = Path(directory) / "arena.xml"
            expanded.write_text(spec.to_xml())
            super().__init__(str(expanded))
        # Arena recolors collision geoms and removes their materials after its
        # postprocessing hook. Keep the authored appearance on the same objects.
        for geom, appearance in self._authored_appearance.items():
            for attribute, value in appearance.items():
                if value is None:
                    geom.attrib.pop(attribute, None)
                else:
                    geom.set(attribute, value)

    def _get_default_classes(self, default: ET.Element) -> Defaults:
        result: Defaults = {}

        def visit(node: ET.Element, inherited: dict[str, dict[str, str]]) -> None:
            values = deepcopy(inherited)
            for child in node:
                if child.tag != "default":
                    values.setdefault(child.tag, {}).update(child.attrib)
            result[node.get("class", "main")] = values
            for child in node.findall("default"):
                visit(child, values)

        visit(default, {})
        return result

    def _replace_defaults_inline(self, default_dic: Defaults) -> None:
        # Arena later appends UI target bodies. Keep element identities so even
        # an authored body with the same name is preserved by our cleanup hook.
        self._authored_bodies = set(self.worldbody.findall("body"))

        def visit(node: ET.Element, inherited: str) -> None:
            if node.tag == "default":
                return
            active = node.attrib.pop("class", inherited)
            children = node.attrib.pop("childclass", inherited)
            node.attrib = {**default_dic[active].get(node.tag, {}), **node.attrib}
            for child in node:
                visit(child, children)

        visit(self.root, "main")
        self._authored_appearance = {
            geom: {attribute: geom.get(attribute) for attribute in ("rgba", "material")}
            for geom in self.worldbody.iter("geom")
        }

    def _postprocess_arena(self) -> None:
        # These are robosuite mjviewer control gizmos, not authored scene objects.
        # MotorScene controls robots through motor commands and does not use them.
        for body in self.worldbody.findall("body"):
            if body not in self._authored_bodies and body.get("name") in (
                "left_eef_target",
                "right_eef_target",
            ):
                self.worldbody.remove(body)
