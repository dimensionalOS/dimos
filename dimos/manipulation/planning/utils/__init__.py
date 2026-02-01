# Copyright 2025 Dimensional Inc.
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

"""
Manipulation Planning Utilities

Standalone utility functions for kinematics and path operations.
These are extracted from the old ABC base classes to enable composition over inheritance.

## Modules

- kinematics_utils: Jacobian operations, singularity detection, pose error computation
- path_utils: Path interpolation, simplification, length computation
"""

from dimos.manipulation.planning.utils.kinematics_utils import (
    check_singularity,
    compute_error_twist,
    compute_pose_error,
    damped_pseudoinverse,
    get_manipulability,
)
from dimos.manipulation.planning.utils.path_utils import (
    compute_path_length,
    interpolate_path,
    interpolate_segment,
)

__all__ = [
    # Kinematics utilities
    "check_singularity",
    "compute_error_twist",
    # Path utilities
    "compute_path_length",
    "compute_pose_error",
    "damped_pseudoinverse",
    "get_manipulability",
    "interpolate_path",
    "interpolate_segment",
]
