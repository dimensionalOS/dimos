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

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from dimos.robot.assets.usd_utils.assets import (
    publish_usd_assets,
    referenced_asset_paths,
    resolve_usd_assets,
)
from dimos.robot.assets.usd_utils.layers import overlay_usds, usd_under_namespace

pytest.importorskip("pxr")

_ROOT = "/robots/arm"

_EVERY_KIND_OF_ASSET = """#usda 1.0
(
    defaultPrim = "arm"
    subLayers = [
        @lights.usda@ (offset = 7)
    ]
)

def Xform "arm" (
    append references = @appended.usda@
    prepend payload = @payload.usda@
)
{
    asset[] textures = [@albedo.png@, @normal.png@]
}
"""

_EVERY_KIND_OF_FILE = {
    "lights.usda",
    "appended.usda",
    "payload.usda",
    "albedo.png",
    "normal.png",
}


def _layer(usd: str) -> Any:
    from pxr import Sdf

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    return layer


def _stage(usd: str) -> Any:
    from pxr import Usd

    return Usd.Stage.Open(_layer(usd))


def test_every_kind_of_asset_is_listed_as_a_file_the_layer_needs() -> None:
    assert referenced_asset_paths(_EVERY_KIND_OF_ASSET) == _EVERY_KIND_OF_FILE


def test_every_kind_of_asset_is_resolved_against_the_root() -> None:
    resolved = referenced_asset_paths(resolve_usd_assets(_EVERY_KIND_OF_ASSET, _ROOT))

    assert resolved == {str(Path(_ROOT) / name) for name in _EVERY_KIND_OF_FILE}


def test_every_kind_of_asset_is_published_under_a_bus_name() -> None:
    usd, public = publish_usd_assets(_EVERY_KIND_OF_ASSET)

    assert set(public.values()) == _EVERY_KIND_OF_FILE
    assert referenced_asset_paths(usd) == set(public)
    assert not [name for name in public if name.startswith("/") or ".." in name.split("/")]


def test_a_sublayer_keeps_its_offset_when_its_path_moves() -> None:
    layer = _layer(resolve_usd_assets(_EVERY_KIND_OF_ASSET, _ROOT))

    assert list(layer.subLayerPaths) == [f"{_ROOT}/lights.usda"]
    assert layer.subLayerOffsets[0].offset == 7


def test_a_deleted_reference_is_not_a_file_the_layer_needs_but_still_moves() -> None:
    usd = '#usda 1.0\n\ndef Xform "arm" (\n    delete references = @gone.usda@\n)\n{\n}\n'

    assert referenced_asset_paths(usd) == set()
    layer = _layer(resolve_usd_assets(usd, _ROOT))
    deleted = layer.GetPrimAtPath("/arm")
    assert [arc.assetPath for arc in deleted.referenceList.deletedItems] == [f"{_ROOT}/gone.usda"]


_TIMED_ARCS = """#usda 1.0

def Xform "arm" (
    prepend references = @wave.usda@</clip> (offset = 3; scale = 2; customData = {string take = "b"})
    prepend payload = @heavy.usda@ (offset = 5)
)
{
}
"""


@pytest.mark.parametrize(
    "move", [lambda usd: resolve_usd_assets(usd, _ROOT), lambda usd: publish_usd_assets(usd)[0]]
)
def test_moving_an_arc_keeps_its_timing_and_custom_data(move: Any) -> None:
    layer = _layer(move(_TIMED_ARCS))
    prim = layer.GetPrimAtPath("/arm")

    (reference,) = prim.referenceList.prependedItems
    assert reference.assetPath != "wave.usda", "the reference did not move at all"
    assert (reference.layerOffset.offset, reference.layerOffset.scale) == (3, 2)
    assert dict(reference.customData) == {"take": "b"}
    assert str(reference.primPath) == "/clip"
    (payload,) = prim.payloadList.prependedItems
    assert payload.assetPath != "heavy.usda", "the payload did not move at all"
    assert payload.layerOffset.offset == 5


_SCENE = """#usda 1.0
(
    defaultPrim = "room"
)

def Xform "room"
{
}

def Material "Materials"
{
    def Material "paint"
    {
        custom string dimos_owner = "room"
    }
}
"""

_PAINTED_ARM = """#usda 1.0
(
    defaultPrim = "arm"
)

def Xform "arm"
{
    def Mesh "shell" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {
        rel material:binding = </Materials/paint>
        rel material:binding:physics = </Materials/rubber>
    }

    def Xform "looks" (
        prepend apiSchemas = ["MaterialBindingAPI", "CollectionAPI:trim"]
    )
    {
        prepend rel collection:trim:includes = </Parts/trim>
        rel material:binding:collection:trim = [<.collection:trim>, </Materials/chrome>]
    }
}

def Scope "Materials"
{
    def Material "paint"
    {
        custom string dimos_owner = "arm"
    }

    def Material "rubber"
    {
    }

    def Material "chrome"
    {
    }
}

def Xform "Parts"
{
    def Xform "trim"
    {
    }
}
"""


def _mounted_in_the_room(usd: str) -> Any:
    """The scene with ``usd`` mounted on its room, as a part provides it: its prims under that frame."""
    return _stage(overlay_usds(_SCENE, [(usd_under_namespace(usd, "room"), "room")]))


def _targets(stage: Any, prim_path: str, relationship: str) -> list[Any]:
    return list(stage.GetPrimAtPath(prim_path).GetRelationship(relationship).GetTargets())


def test_a_mounted_mesh_keeps_the_material_bound_beside_its_default_prim() -> None:
    stage = _mounted_in_the_room(_PAINTED_ARM)

    (paint,) = _targets(stage, "/room/arm/shell", "material:binding")
    assert str(paint) != "/Materials/paint", "the arm is wearing the room's paint"
    assert stage.GetPrimAtPath(paint).GetAttribute("dimos_owner").Get() == "arm"
    assert stage.GetPrimAtPath("/Materials/paint").GetAttribute("dimos_owner").Get() == "room"


def test_a_mounted_physics_binding_follows_it_too() -> None:
    stage = _mounted_in_the_room(_PAINTED_ARM)

    (rubber,) = _targets(stage, "/room/arm/shell", "material:binding:physics")
    assert stage.GetPrimAtPath(rubber), f"the physics material at {rubber} was left behind"


def test_a_mounted_collection_binding_keeps_its_material_and_members() -> None:
    stage = _mounted_in_the_room(_PAINTED_ARM)

    collection, chrome = _targets(stage, "/room/arm/looks", "material:binding:collection:trim")
    assert str(collection) == "/room/arm/looks.collection:trim"
    assert stage.GetPrimAtPath(chrome), f"the chrome material at {chrome} was left behind"
    (trim,) = _targets(stage, "/room/arm/looks", "collection:trim:includes")
    assert stage.GetPrimAtPath(trim), f"the collection member at {trim} was left behind"


def test_two_bindings_to_one_material_beside_the_default_prim_stay_one() -> None:
    twice = _PAINTED_ARM.replace("</Materials/rubber>", "</Materials/paint>")
    stage = _mounted_in_the_room(twice)

    assert _targets(stage, "/room/arm/shell", "material:binding") == _targets(
        stage, "/room/arm/shell", "material:binding:physics"
    )


def test_a_moved_material_does_not_swallow_one_its_name_now_shadows() -> None:
    # /Looks/paint and /paint both move under the namespace, and neither may land on the other.
    arm = """#usda 1.0
(
    defaultPrim = "arm"
)

def Xform "arm"
{
    def Mesh "shell" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {
        rel material:binding = </Looks/paint>
        rel material:binding:physics = </paint>
    }
}

def Scope "Looks"
{
    def Material "paint"
    {
        custom string dimos_owner = "looks"
    }
}

def Material "paint"
{
    custom string dimos_owner = "root"
}
"""
    stage = _mounted_in_the_room(arm)

    (looks,) = _targets(stage, "/room/arm/shell", "material:binding")
    (root,) = _targets(stage, "/room/arm/shell", "material:binding:physics")
    assert looks != root
    assert stage.GetPrimAtPath(looks).GetAttribute("dimos_owner").Get() == "looks"
    assert stage.GetPrimAtPath(root).GetAttribute("dimos_owner").Get() == "root"


def test_resolving_assets_already_in_place_opens_no_list_editor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from pxr import Sdf

    resolved = resolve_usd_assets(_EVERY_KIND_OF_ASSET, _ROOT)

    def opened(spec: Any) -> None:
        pytest.fail(f"opened a list editor on {spec.path}, which a scene does per arc")

    for name in ("referenceList", "payloadList"):
        monkeypatch.setattr(Sdf.PrimSpec, name, property(opened))
    assert resolve_usd_assets(resolved, _ROOT) == resolved


_BOUND_ARM = """#usda 1.0
(
    defaultPrim = "arm"
    upAxis = "Z"
)

def Xform "arm"
{
    def Xform "base" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "MaterialBindingAPI"]
    )
    {
        rel material:binding = </Looks/paint>
        float level.connect = </arm/tip.level>
    }

    def Xform "tip" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        prepend references = </arm/base>
    )
    {
        float level
    }

    def PhysicsFixedJoint "weld"
    {
        rel physics:body0 = </arm/base>
        rel physics:body1 = </arm/tip>
    }
}

def Scope "Looks"
{
    def Material "paint"
    {
    }
}
"""


@pytest.mark.parametrize("namespace", ["left", "fleet/robot0", "arm"])
def test_a_layer_nested_under_a_namespace_names_each_frame_as_on_tf(namespace: str) -> None:
    from dimos.robot.assets.usd_utils.frames import usd_frame_names

    nested = usd_under_namespace(_BOUND_ARM, namespace)
    stage = _stage(nested)

    under = {f"{namespace}/{frame}" for frame in usd_frame_names(_BOUND_ARM)}
    ancestors = {"/".join(namespace.split("/")[: n + 1]) for n in range(namespace.count("/") + 1)}
    assert usd_frame_names(nested) == under | ancestors
    assert str(stage.GetDefaultPrim().GetPath()) == f"/{namespace.partition('/')[0]}"
    assert stage.GetMetadata("upAxis") == "Z"
    base = stage.GetPrimAtPath(f"/{namespace}/arm/base")
    assert [str(t) for t in base.GetRelationship("material:binding").GetTargets()] == [
        f"/{namespace}/Looks/paint"
    ]
    assert [str(c) for c in base.GetAttribute("level").GetConnections()] == [
        f"/{namespace}/arm/tip.level"
    ]
    weld = stage.GetPrimAtPath(f"/{namespace}/arm/weld")
    assert [str(t) for t in weld.GetRelationship("physics:body1").GetTargets()] == [
        f"/{namespace}/arm/tip"
    ]
    tip = stage.GetPrimAtPath(f"/{namespace}/arm/tip")
    assert tip.HasAPI(_rigid_body_api()), "the internal reference still composes"
    assert not stage.GetPrimAtPath("/arm/base") or namespace == "arm"


def _rigid_body_api() -> Any:
    from pxr import UsdPhysics

    return UsdPhysics.RigidBodyAPI


def test_no_namespace_leaves_the_layer_alone() -> None:
    assert usd_under_namespace(_BOUND_ARM, "") == _BOUND_ARM


def test_a_namespace_that_cannot_be_a_prim_path_is_refused() -> None:
    with pytest.raises(ValueError, match="'2nd' starts with '2'"):
        usd_under_namespace(_BOUND_ARM, "fleet/2nd")
