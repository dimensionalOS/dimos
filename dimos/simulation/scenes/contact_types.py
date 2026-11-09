# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Names used by procedural scene geometry and contact classification."""

from typing import Literal

Part = Literal["trunk", "lidar", "leg", "foot"]
Kind = Literal["floor", "wall", "ceiling", "clutter"]
