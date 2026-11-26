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


from collections.abc import Callable
import hashlib
from pathlib import Path
import tempfile

from dimos.constants import CACHE_DIR
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.lfs import pull_usd_assets
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Under this user's cache, not the shared temp dir: these files are loaded as scene content.
_CACHE = CACHE_DIR / "sim2-usd"


def usd_layer(model: UsdModel, transform: Callable[[str], str] | None = None) -> Path:
    """Write the model out as a layer file, optionally rewritten for whoever is about to read it."""
    from dimos.robot.assets.usd_utils.assets import resolve_usd_assets

    root = model.asset_root
    layer = resolve_usd_assets(model.usd, str(root) if root is not None else "")
    # Before the transform opens a stage of it, and as MuJoCo reads a Git LFS pointer as an empty mesh.
    pull_usd_assets([layer])
    if transform is not None:
        layer = transform(layer)
    return layer_file(layer, called=_name_of(model))


def layer_file(layer: str, called: str = "layer") -> Path:
    out = _CACHE / f"{called}-{hashlib.sha256(layer.encode()).hexdigest()[:16]}.usda"
    if not out.exists():
        _write_whole(out, layer.encode())
    return out


def _write_whole(out: Path, data: bytes) -> None:
    """Write ``out`` so a reader sees all of it or none; each writer, thread or process, has its own partial."""
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        dir=out.parent, prefix=f"{out.name}.", suffix=".partial", delete=False
    ) as partial:
        partial.write(data)
    Path(partial.name).replace(out)


def _name_of(model: UsdModel) -> str:
    source = model.source_path
    return source.stem if source is not None else "layer"
