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

"""Unreal editor-only preparation in the separate SF555 JoyLo project.

Reads DIMOS_SF555_JOYLO_ASSETS. Replaces four merged chunks with two furniture
chunks and saves the two bottle assets for pose-driven runtime rendering. Their
actors are removed from the background map. The plastic body uses an opaque
approximation so RGB and ideal depth see the same graspable surface; cap/label
materials retain the source appearance. The original office project is untouched.
"""

import json
import math
import os
from pathlib import Path

import unreal  # type: ignore[import-not-found]


def main() -> None:
    root = Path(os.environ["DIMOS_SF555_JOYLO_ASSETS"])
    project = Path(unreal.Paths.project_dir()).resolve()
    if project != (root / "DimosRenderer").resolve():
        raise ValueError("Refusing to edit any project except the prepared JoyLo copy")
    split = json.loads((root / "visual-split.json").read_text())
    level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    assets = unreal.get_editor_subsystem(unreal.EditorAssetSubsystem)
    assert level.load_level("/Game/SF555/Maps/Office")
    old = {a.get_actor_label(): a for a in actors.get_all_level_actors()}
    replacements = {}
    for name in split["remove_actors"]:
        if name not in old:
            raise ValueError(f"Missing expected office actor: {name}")
        for slot in old[name].static_mesh_component.static_mesh.get_editor_property(
            "static_materials"
        ):
            if slot.material_interface:
                replacements[str(slot.material_slot_name)] = slot.material_interface
                replacements[slot.material_interface.get_name()] = slot.material_interface
    actors.destroy_actors([old[name] for name in split["remove_actors"]])
    manager = unreal.InterchangeManager.get_interchange_manager_scripted()
    source = unreal.InterchangeManager.create_source_data(str(root / "pickables.glb"))
    assert manager.import_scene(
        "/Game/Joylo/Imported",
        source,
        unreal.ImportAssetParameters(is_automated=True, replace_existing=True),
    )
    manager.wait_until_all_tasks_done(False)
    expected = {r["name"]: r for r in split["exported"]}
    imported = {
        a.get_actor_label(): a
        for a in actors.get_all_level_actors()
        if a.get_actor_label() in expected
    }
    if set(imported) != set(expected):
        raise ValueError(f"Missing imported actors: {set(expected) - set(imported)}")
    tools = unreal.AssetToolsHelpers.get_asset_tools()
    opaque_path = "/Game/Joylo/MI_BottlePlastic"
    opaque = (
        assets.load_asset(opaque_path)
        if assets.does_asset_exist(opaque_path)
        else tools.create_asset(
            "MI_BottlePlastic",
            "/Game/Joylo",
            unreal.MaterialInstanceConstant,
            unreal.MaterialInstanceConstantFactoryNew(),
        )
    )
    lib = unreal.MaterialEditingLibrary
    lib.set_material_instance_parent(opaque, assets.load_asset("/Game/Materials/M_Dimos"))
    lib.set_material_instance_vector_parameter_value(
        opaque, "BaseColor", unreal.LinearColor(0.62, 0.80, 0.90, 1)
    )
    lib.set_material_instance_scalar_parameter_value(opaque, "Roughness", 0.25)
    native = {}
    report = []
    for name, actor in imported.items():
        comp = actor.static_mesh_component
        mesh = comp.static_mesh
        position = actor.get_actor_location()
        expected_position = expected[name]["position"]
        error = math.dist(
            [position.x, -position.y, position.z], [100 * v for v in expected_position]
        )
        if error > 0.02:
            raise ValueError(f"Imported pose differs from physical source: {name}: {error} cm")
        dynamic = name.startswith("Joylo_bottle_")
        materials = []
        for index, slot in enumerate(mesh.get_editor_property("static_materials")):
            mat = replacements.get(
                str(slot.material_slot_name),
                replacements.get(slot.material_interface.get_name(), slot.material_interface),
            )
            if dynamic and mat.get_blend_mode() not in (
                unreal.BlendMode.BLEND_OPAQUE,
                unreal.BlendMode.BLEND_MASKED,
            ):
                mat = opaque
            mesh.set_material(index, mat)
            materials.append(mat.get_path_name())
        if dynamic:
            rotation = actor.get_actor_rotation()
            scale = actor.get_actor_scale3d()
            if (
                max(abs(rotation.pitch), abs(rotation.yaw), abs(rotation.roll)) > 1e-3
                or max(abs(scale.x - 1), abs(scale.y - 1), abs(scale.z - 1)) > 1e-4
            ):
                raise ValueError(
                    f"Native bottle must be identity-oriented and unit-scale: {name}: {rotation}, {scale}"
                )
            native["props/" + name.removeprefix("Joylo_")] = mesh.get_path_name()
            actors.destroy_actor(actor)
        report.append(
            {
                "name": name,
                "mesh": mesh.get_path_name(),
                "materials": materials,
                "position_error_cm": error,
                "dynamic": dynamic,
            }
        )
    assets.save_directory("/Game/Joylo", only_if_is_dirty=True, recursive=True)
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    assert unreal.EditorLoadingAndSavingUtils.save_map(world, "/Game/SF555/Maps/OfficeJoylo")
    (root / "native-visuals.json").write_text(json.dumps(native, indent=2))
    (root / "unreal-pickables-report.json").write_text(json.dumps(report, indent=2))
    print("JOYLO_PICKABLES_READY " + json.dumps(native), flush=True)


main()
