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

"""Enumerated types for the PimSim spec.

``EntityKind`` and ``ShapeHint`` are re-exported from ``entity.py`` (the wire
contract owns them); ``AuthorityMode`` is introduced here to name the
``OWNS`` / ``MIRROR`` distinction that today is an ad-hoc ``"browser"`` /
``"external"`` string on the Babylon viewer.
"""

from __future__ import annotations

from enum import Enum

# Canonical literals for the entity wire format — defined where the format is.
from dimos.experimental.pimsim.entity import EntityKind, ShapeHint


class AuthorityMode(Enum):
    """Whether a physics authority simulates the scene or only mirrors it."""

    OWNS = "owns"
    """This instance integrates physics and is the source of truth for the
    entity stream (Babylon with ``entity_authority="browser"``; MuJoCo)."""

    MIRROR = "mirror"
    """This instance renders another authority's stream as kinematic bodies —
    a viewer, not a simulator (Babylon with ``entity_authority="external"``)."""


__all__ = ["AuthorityMode", "EntityKind", "ShapeHint"]
