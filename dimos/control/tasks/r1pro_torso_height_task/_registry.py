# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

TASK_FACTORIES = {"r1pro_torso_height": "dimos.robot.galaxea.r1pro.torso_height_task:create_task"}
TASK_CONSUMES = {"r1pro_torso_height": {"lift_rate": ("on_lift_rate", "direct")}}
