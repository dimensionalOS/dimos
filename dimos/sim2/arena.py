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

from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import mujoco
from robosuite.models.arenas.arena import Arena

from dimos.sim2.model import Defaults, default_classes, inline_defaults


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
        return default_classes(default)

    def _replace_defaults_inline(self, default_dic: Defaults) -> None:
        # Arena later appends UI target bodies. Track the original elements.
        self._authored_bodies = set(self.worldbody.findall("body"))
        inline_defaults(self.root, default_dic)
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
