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

"""Manifest domain model: the Python mirror of web/shared/manifest.ts.

Pinned by the golden vectors in web/shared/fixtures/manifests.json (tested
from both pytest and deno test). The transport (protocol.py) checks only
field shapes; this module owns the domain rules: bounded unique ids, positive
rates, and panel/layout references that resolve. Panels and layout are
minimal until T7 (the layout is a flat panel-id order, not a tree).
"""

import json
import sys
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError

Delivery = Literal["latest", "reliable"]

# Bound for channel/panel ids, encodings, and panel kinds.
MAX_MANIFEST_ID_LEN = 64


class ManifestError(ValueError):
    """`code` is the machine-readable reason, pinned by the golden vectors."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(f"{code}: {message}")


class _ManifestModel(BaseModel):
    # strict + allow_inf_nan=False, mirroring _WireModel in protocol.py
    # (which imports ChannelSpec from here, so this base cannot be shared).
    model_config = ConfigDict(strict=True, allow_inf_nan=False)


class ChannelSpec(_ManifestModel):
    """One robot->viewer stream (see ChannelSpec in manifest.ts).

    Field names are the wire names, hence the camelCase.
    """

    ch: str
    encoding: str
    delivery: Delivery
    maxHz: int | float


class PanelSpec(_ManifestModel):
    """Minimal until T7: kind is the panel-component registry key; channels
    lists the channel ids the panel consumes."""

    id: str
    kind: str
    channels: list[str]


class Manifest(_ManifestModel):
    channels: list[ChannelSpec]
    panels: list[PanelSpec] = Field(default_factory=list)
    # Panel ids in display order (a layout tree replaces this in T7).
    layout: list[str] = Field(default_factory=list)


_MANIFEST_TA: TypeAdapter[Manifest] = TypeAdapter(Manifest)


def _bounded_id(s: str) -> bool:
    return 1 <= len(s) <= MAX_MANIFEST_ID_LEN


def parse_manifest(data: Any) -> Manifest:
    """Validated manifest from parsed JSON (or any untrusted value); raises
    ManifestError. Absent panels/layout normalize to empty; unknown keys are
    dropped (forward compatibility with newer bridges). Mirrors
    parseManifest() in manifest.ts: shape first, then domain rules in
    document order, so both sides report the same code (pinned by fixtures).
    """
    try:
        # Through JSON, not validate_python: strict python-mode validation
        # wants model instances for nested fields, but callers hold
        # parsed-JSON dicts.
        manifest = _MANIFEST_TA.validate_json(json.dumps(data))
    except ValidationError as e:
        raise ManifestError("invalid_shape", str(e)) from e

    ch_ids: set[str] = set()
    for spec in manifest.channels:
        if not _bounded_id(spec.ch):
            raise ManifestError(
                "invalid_channel_id", f"channel id must be 1..{MAX_MANIFEST_ID_LEN} chars"
            )
        if spec.ch in ch_ids:
            raise ManifestError("duplicate_channel_id", f"duplicate channel {spec.ch}")
        ch_ids.add(spec.ch)
        if not _bounded_id(spec.encoding):
            raise ManifestError(
                "invalid_encoding", f"encoding must be 1..{MAX_MANIFEST_ID_LEN} chars"
            )
        # Bounded to float64: a JSON integer beyond that range is exact in
        # Python but Infinity through JS JSON.parse, and manifest.ts rejects
        # it here with this code. Non-finite floats are already shape-rejected
        # (and isfinite() would raise OverflowError on huge ints).
        if not 0 < spec.maxHz <= sys.float_info.max:
            raise ManifestError("invalid_max_hz", f"maxHz for {spec.ch} must be a positive number")

    panel_ids: set[str] = set()
    for panel in manifest.panels:
        if not (_bounded_id(panel.id) and _bounded_id(panel.kind)):
            raise ManifestError(
                "invalid_panel", f"panel id/kind must be 1..{MAX_MANIFEST_ID_LEN} chars"
            )
        if panel.id in panel_ids:
            raise ManifestError("duplicate_panel_id", f"duplicate panel {panel.id}")
        panel_ids.add(panel.id)
        for ch in panel.channels:
            if ch not in ch_ids:
                raise ManifestError(
                    "unknown_panel_channel", f"panel {panel.id} wants undeclared channel {ch}"
                )

    for panel_id in manifest.layout:
        if panel_id not in panel_ids:
            raise ManifestError("unknown_layout_panel", f"layout names unknown panel {panel_id}")

    return manifest
