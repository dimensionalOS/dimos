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

"""A described model as a USD file a physics engine can open.

A description carries a ``UsdModel``, which may be a layer that exists nowhere: a robot's
committed layer with its sensor mounts added is new text, and so is a scene composed out of
several subtrees. MuJoCo opens a *file*, and a layer's references are relative to wherever it
was written, so handing it one means writing the text out with its asset paths resolved first.

Written under a content hash and reused, the way ``mjcf_with_frames`` caches its MJCFs. That
matters for more than speed: the world is compiled in more than one process -- Simulation steps
it while a sensor's view draws it -- and each would otherwise write its own copy of the same
layer and name it something different, which is exactly what a renderer asking for a model by
name cannot cope with.
"""

import hashlib
import os
from pathlib import Path
import tempfile

from dimos.robot.assets.model import UsdModel

_CACHE = Path(tempfile.gettempdir()) / "dimos-sim2-usd"


def usd_layer(model: UsdModel) -> Path:
    """``model``'s layer as a file, with every asset path it names resolved; reused while it does not change.

    The same layer always lands at the same path, so two processes compiling one world agree on
    what to call the robot they are drawing.
    """
    from dimos.robot.assets.usd import resolve_usd_assets

    source = model.source_path
    # A layer read from a file names its meshes relative to that file; one converted or composed
    # in memory came from nowhere and names them absolutely or not at all.
    layer = resolve_usd_assets(model.usd, str(source.parent) if source is not None else "")
    out = _CACHE / f"{_name_of(model)}-{hashlib.sha256(layer.encode()).hexdigest()[:16]}.usda"
    if not out.exists():
        out.parent.mkdir(parents=True, exist_ok=True)
        # Written beside and moved into place, so a second process never opens a half-written
        # layer and reports it as a syntax error. Named per process, so two of them writing the
        # same layer at once do not write into one another's partial file.
        partial = out.with_suffix(f".{os.getpid()}.partial")
        partial.write_text(layer)
        partial.replace(out)
    return out


def _name_of(model: UsdModel) -> str:
    """Something recognisable in front of the hash: a path a person has to read says whose it is."""
    source = model.source_path
    return source.stem if source is not None else "layer"


def scene_layer(scene: UsdModel | Path | str | None) -> Path:
    """The file an engine opens for the world a config names.

    A world given as USD is materialised like a robot's: written out with its asset paths
    resolved, because a composed scene exists nowhere until it is. A world given as a file is
    already one, and is handed over as it stands.

    That second case is how the toy worlds the tests build, and the shipped `logistics.xml`,
    keep working: they are MJCF, which MuJoCo reads perfectly well and usd-core cannot parse at
    all. Refusing a world that loads would buy nothing. The robot descriptions are the ones held
    to USD and nothing else, and the real blueprints hand the scene over as USD too.
    """
    if isinstance(scene, UsdModel):
        return usd_layer(scene)
    return Path(scene) if scene is not None else Path(__file__).with_name("logistics.xml")
