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
several subtrees. An engine opens a *file*, and a layer's references are relative to wherever it
was written, so handing it one means writing the text out with its asset paths resolved first.

Written under a content hash and reused. That matters for more than speed: the world is compiled
in more than one process -- Simulation steps it while a sensor's view draws it -- and each would
otherwise write its own copy of the same layer and name it something different, which is exactly
what a renderer asking for a model by name cannot cope with.
"""

from collections.abc import Callable
import hashlib
import os
from pathlib import Path
import tempfile

from dimos.robot.assets.model import UsdModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_CACHE = Path(tempfile.gettempdir()) / "dimos-sim2-usd"
# Where a fetched asset lands, keyed by the hash of its content: shared by every process on this
# machine, and safe to keep, because a name that is a hash can only ever mean the same bytes.
_ASSETS = Path(tempfile.gettempdir()) / "dimos-usd-assets"


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
    return layer_file(layer, called=_name_of(model))


def layer_file(layer: str, called: str = "layer") -> Path:
    """A layer's text as a file, under a hash of itself; written once and reused.

    The same layer always lands at the same path, so two processes that compiled one world agree
    on what to call the thing they are each holding.
    """
    out = _CACHE / f"{called}-{hashlib.sha256(layer.encode()).hexdigest()[:16]}.usda"
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

    That second case is deliberate: a file on disk needs nothing done to it, and what formats an
    engine can open is the engine's business, not this function's. Composing is the only thing
    that needs the layer parsed, and a world nobody composed does not need composing. The *robot*
    descriptions are the ones held to USD and nothing else.
    """
    if isinstance(scene, UsdModel):
        return usd_layer(scene)
    return Path(scene) if scene is not None else Path(__file__).with_name("logistics.xml")


def scene_as_files(usd: str, fetch: Callable[[str], bytes]) -> Path:
    """``usd`` written out as a file, with every asset it names fetched to disk beside it.

    A scene arrives as one string naming its meshes rather than pointing at them -- which is what
    lets it reach a consumer that shares no filesystem with whoever composed it. An engine opens
    files, so somebody has to turn those names back into files, and this is that somebody for
    anything that is going to open the scene rather than draw it lazily.

    Every way a prim can name a file is rewritten, not just the obvious one. A mesh is a
    **reference arc** to a sidecar, its original is named in `assetInfo["identifier"]`, and a
    texture is a plain asset-valued attribute. Walking only the attributes found three assets in
    a room of three and a half thousand and the engine then refused the first desk: "at least 4
    vertices required", which is what an unresolved mesh looks like by the time anyone sees it.

    Fetched once per machine, not once per process, and keyed by **content**: a layer that says
    what an asset's hash is gets that asset skipped if those bytes are already on disk, and one
    that does not gets it fetched and then keyed by the hash of what came back. The same bytes
    are never stored twice and an unchanged mesh is never asked for again.

    ``fetch`` is how a name becomes bytes: the asker knows who published the scene, and this
    does not need to.
    """
    from dimos.robot.assets.usd import _layer_from, _pxr, asset_attributes

    pxr = _pxr()
    layer = _layer_from(pxr, usd, "a scene")
    _ASSETS.mkdir(parents=True, exist_ok=True)
    here: dict[str, str] = {}
    stack, rewritten = [layer.pseudoRoot], 0

    def on_disk(name: str, declared: str | None) -> str:
        if name not in here:
            here[name] = str(_cached(name, declared, fetch))
        return here[name]

    while stack:
        spec = stack.pop()
        info = dict(spec.assetInfo) if spec.assetInfo else {}
        declared = str(info["version"]) if "version" in info else None
        items = spec.referenceList.explicitItems or spec.referenceList.prependedItems
        if items:
            spec.referenceList.explicitItems = [
                pxr.Sdf.Reference(on_disk(ref.assetPath, declared), ref.primPath)
                if ref.assetPath
                else ref
                for ref in items
            ]
            rewritten += len(items)
        identifier = info.get("identifier")
        if identifier is not None and str(identifier.path):
            info["identifier"] = pxr.Sdf.AssetPath(on_disk(str(identifier.path), declared))
            spec.assetInfo = info
        for attribute, value in asset_attributes(spec):
            attribute.default = pxr.Sdf.AssetPath(on_disk(str(value.path), None))
            rewritten += 1
        stack.extend(spec.nameChildren)

    logger.info(
        f"sim2: {rewritten} asset reference(s) over {len(here)} file(s) of this scene are on "
        f"disk under {_ASSETS}"
    )
    return layer_file(layer.ExportToString(), called="scene")


def _cached(name: str, digest: str | None, fetch: Callable[[str], bytes]) -> Path:
    """The file holding this asset, fetching it only if its content is not already here."""
    suffix = Path(name).suffix
    if digest:
        promised = _ASSETS / f"{digest}{suffix}"
        if promised.exists():
            return promised
    data = fetch(name)
    from dimos.robot.assets.usd import asset_hash

    out = _ASSETS / f"{asset_hash(data)}{suffix}"
    if not out.exists():
        partial = out.with_suffix(f"{suffix}.{os.getpid()}.partial")
        partial.write_bytes(data)
        partial.replace(out)
    return out
