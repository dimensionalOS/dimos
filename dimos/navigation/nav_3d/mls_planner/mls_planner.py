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

"""Python interface to the Rust multi-level surface path planner."""

from __future__ import annotations

try:
    from dimos_mls_planner import MLSPlanner
except ImportError as e:
    raise ImportError(
        "dimos_mls_planner is not built. Run: "
        "uv run maturin develop --uv -m dimos/navigation/nav_3d/mls_planner/rust/Cargo.toml"
    ) from e

__all__ = ["MLSPlanner"]
