# Copyright 2025-2026 Dimensional Inc.
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

"""Shared CoACD setup for scene-cooking convex decomposition."""

from __future__ import annotations


def silence_coacd_logging() -> None:
    """Quiet CoACD's per-invocation C-library log spam.

    ``set_log_level`` is a cheap call, so this is safe to call before
    every ``coacd.run_coacd`` invocation instead of caching "already
    silenced" state on a function attribute.
    """
    import coacd  # type: ignore[import-not-found, import-untyped]

    coacd.set_log_level("error")
