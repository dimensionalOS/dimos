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
from dimos.robot.assets.usd_utils.layers import usd_moved_under, usd_under_namespace

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


@pytest.mark.parametrize("under", ["room/spot", "left"])
def test_a_namespaced_layer_moved_under_a_prim_keeps_every_path_it_states(under: str) -> None:
    moved = usd_moved_under(usd_under_namespace(_BOUND_ARM, "fleet/robot0"), "fleet/robot0", under)
    stage = _stage(moved)

    assert not stage.GetPrimAtPath("/fleet")
    assert str(stage.GetDefaultPrim().GetPath()) == f"/{under.partition('/')[0]}"
    weld = stage.GetPrimAtPath(f"/{under}/arm/weld")
    assert [str(t) for t in weld.GetRelationship("physics:body1").GetTargets()] == [
        f"/{under}/arm/tip"
    ]
    assert stage.GetPrimAtPath(f"/{under}/arm/tip").HasAPI(_rigid_body_api())


def test_a_part_s_frames_are_found_from_their_tf_ids_wherever_it_was_put() -> None:
    from dimos.robot.assets.usd_utils.frames import usd_with_frame_names, usd_world_frame_prims

    named = usd_with_frame_names(_BOUND_ARM, "fleet/robot0", {"arm/tip": "fleet/robot0/tip"})
    served = usd_under_namespace(named, "fleet/robot0")
    moved = usd_moved_under(served, "fleet/robot0", "room/spot")

    for usd, at in ((served, "fleet/robot0"), (moved, "room/spot")):
        prims = usd_world_frame_prims(usd)
        assert prims["fleet/robot0/tip"] == f"{at}/arm/tip"
        assert prims["fleet/robot0/arm/base"] == f"{at}/arm/base"
        assert "fleet/robot0/arm/tip" not in prims
