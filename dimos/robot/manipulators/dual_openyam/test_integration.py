# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from io import BytesIO

from yourdfpy import URDF  # type: ignore[import-untyped]

from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.robot.manipulators.dual_openyam.config import dual_openyam_model_config


def test_authoritative_arm_only_model_prepares_for_viser() -> None:
    config = dual_openyam_model_config()
    prepared = prepare_urdf_for_drake(
        config.model.load(),
        convert_meshes=config.auto_convert_meshes,
    )
    model = URDF.load(
        BytesIO(prepared.xml.encode()),
        mesh_dir=str(prepared.source_path.parent),
        build_scene_graph=True,
        build_collision_scene_graph=True,
        load_meshes=True,
        load_collision_meshes=True,
    )
    assert {
        "left_grasp_frame",
        "right_grasp_frame",
        "left_tip_left",
        "left_tip_right",
        "right_tip_left",
        "right_tip_right",
    } <= set(model.link_map)
