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

"""Getting at USD at all: the lazily-loaded pxr handle, its type aliases, and the smallest
helpers that every other module here needs."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from types import ModuleType
from typing import TYPE_CHECKING, Any, TypeAlias
import uuid

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray


Vector = tuple[float, float, float]


_NO_USD = (
    "Reading or writing USD needs usd-core (pxr), which is in the 'sim' and 'scene' extras: "
    "uv sync --extra sim. On linux-aarch64 a wheel exists only from usd-core 26.11 and only for "
    "Python 3.11 and up, so a 3.10 arm robot cannot serve USD at all."
)
_PXR: Pxr | None = None
_PXR_LOCK = Lock()


def load_pxr() -> Pxr:
    global _PXR
    if _PXR is not None:
        return _PXR
    with _PXR_LOCK:
        if _PXR is None:
            _let_mujoco_register_its_schemas_first()
            try:
                from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, Vt
            except ImportError as exc:  # pragma: no cover - only where the extra is absent
                raise ImportError(_NO_USD) from exc
            _PXR = Pxr(
                Gf=Gf, Sdf=Sdf, Tf=Tf, Usd=Usd, UsdGeom=UsdGeom, UsdPhysics=UsdPhysics, Vt=Vt
            )
    return _PXR


def _let_mujoco_register_its_schemas_first() -> None:
    try:
        import mujoco._usd  # noqa: F401
    except ImportError:
        return


Layer: TypeAlias = Any
Matrix: TypeAlias = Any
Prim: TypeAlias = Any
Quat: TypeAlias = Any
Spec: TypeAlias = Any
Stage: TypeAlias = Any


@dataclass(frozen=True)
class Pxr:
    Gf: ModuleType
    Sdf: ModuleType
    Tf: ModuleType
    Usd: ModuleType
    UsdGeom: ModuleType
    UsdPhysics: ModuleType
    Vt: ModuleType


def frame_name(prim: Prim) -> str:
    return str(prim.GetDisplayName() or prim.GetName())


def warm_usd() -> None:
    pxr = load_pxr()
    layer = pxr.Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString('#usda 1.0\n(\n    defaultPrim = "warm"\n)\n\ndef Xform "warm"\n{\n}\n')
    stage = pxr.Usd.Stage.Open(layer)
    list(stage.Traverse())


def layer_from(pxr: Pxr, usd: str, what: str, root: Path | str | None) -> Layer:
    if root is None:
        layer = pxr.Sdf.Layer.CreateAnonymous(".usda")
    else:
        name = f".dimos-in-memory-{uuid.uuid4().hex}.usda"
        layer = pxr.Sdf.Layer.New(
            pxr.Sdf.FileFormat.FindByExtension(".usda"), str(Path(root) / name)
        )
    if not layer.ImportFromString(usd):
        raise ValueError(f"the USD for {what!r} is not a layer this build of USD can read")
    return layer


def usd_matrix(pxr: Pxr, transform: NDArray[np.float64]) -> Matrix:
    return pxr.Gf.Matrix4d(*np.asarray(transform, dtype=np.float64).T.reshape(-1).tolist())


def prim_identifier(pxr: Pxr, name: str) -> str:
    return str(pxr.Tf.MakeValidIdentifier(name))
