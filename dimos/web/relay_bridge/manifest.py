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
from both pytest and deno test). The transport (protocol.py) carries the
manifest as one opaque dict; this module is the single owner of its
structure and domain rules: version gate, bounded unique ids, positive
rates, panel/layout/pages references that resolve, and kind-specific panel
rules (video, map2d, teleop, chat, navmap, control).

Manifest v1 is frozen. Additive changes (new panel kinds, new params) ride
the existing shape: unknown keys and kinds pass through validation.
Breaking changes bump `version`, and the Cockpit refuses politely
(unsupported_version).
"""

import json
import sys
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError

Delivery = Literal["latest", "reliable"]
# Flow direction seen from the viewer: rx = robot->viewer; tx (teleop, chat)
# arrives with later tickets.
Dir = Literal["rx", "tx"]

# The only manifest version this build understands.
MANIFEST_VERSION = 1

# Bound for channel/panel ids, encodings, and panel kinds.
MAX_MANIFEST_ID_LEN = 64

# Channel ids with this prefix belong to protocol control (e.g. @control)
# and can never be declared by a manifest.
RESERVED_CHANNEL_PREFIX = "@"


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
    """One robot<->viewer stream (see ChannelSpec in manifest.ts).

    Field names and order are the wire names/order, hence the camelCase.
    `params` carries encoder settings (e.g. jpeg quality); absent fields
    normalize to their defaults.
    """

    ch: str
    dir: Dir = "rx"
    encoding: str
    delivery: Delivery
    maxHz: int | float
    params: dict[str, Any] = Field(default_factory=dict)


class PanelSpec(_ManifestModel):
    """kind is the panel-component registry key; channels lists the channel
    ids the panel consumes in kind-defined slot order (e.g. map2d: costmap,
    then the optional pose overlay). title "" means untitled (the frame
    falls back to the panel id)."""

    id: str
    kind: str
    title: str = ""
    channels: list[str]
    params: dict[str, Any] = Field(default_factory=dict)


class Manifest(_ManifestModel):
    version: int | float
    channels: list[ChannelSpec]
    panels: list[PanelSpec] = Field(default_factory=list)
    # Recursive layout tree: a leaf is a panel-id string; row/col nodes are
    # {"row"/"col": [children], "shares"?: [floats]}. Modeled as Any and
    # validated by the hand-rolled walker below so structural failures map
    # to invalid_layout AFTER the panel/kind rules (document order, like
    # manifest.ts) and normalization can drop unknown node keys while
    # preserving `shares` presence. None = no layout authored.
    layout: Any = None
    # Panel ids rendered as full-page tabs. Kept out of the model input in
    # parse_manifest so its shape errors fire in the pages phase (after
    # layout), mirroring manifest.ts.
    pages: list[str] = Field(default_factory=list)


_MANIFEST_TA: TypeAdapter[Manifest] = TypeAdapter(Manifest)

# Panel kinds with a fixed channel slot table (the cockpit panels authored by
# dimos/web/cockpit.py Chat/NavMap/Control): the panel must bind exactly
# these slots, in this order, each with the listed dir + encoding. Delivery
# is deliberately not checked (it is the bridge's per-channel choice) and the
# role->channel mapping the panel also carries in `params` is informational.
# Mirrored by SLOT_KINDS in manifest.ts (same codes, same slot order).
_SLOT_KINDS: dict[str, tuple[str, tuple[tuple[Dir, str], ...]]] = {
    # chat, idle flag, mode, then the text input.
    "chat": (
        "invalid_chat_panel",
        (
            ("rx", "chat.json.v1"),
            ("rx", "flag.json.v1"),
            ("rx", "mode.json.v1"),
            ("tx", "text.json.v1"),
        ),
    ),
    # costmap, pose, path, places, nav state, then goal + command outputs.
    "navmap": (
        "invalid_navmap_panel",
        (
            ("rx", "costmap.zlib.v1"),
            ("rx", "pose.json.v1"),
            ("rx", "path.json.v1"),
            ("rx", "places.json.v1"),
            ("rx", "navstate.json.v1"),
            ("tx", "pose_goal.json.v1"),
            ("tx", "command.json.v1"),
        ),
    ),
    # mode, policy state, nav state, then the command output.
    "control": (
        "invalid_control_panel",
        (
            ("rx", "mode.json.v1"),
            ("rx", "policy.json.v1"),
            ("rx", "navstate.json.v1"),
            ("tx", "command.json.v1"),
        ),
    ),
}


def _bounded_id(s: str) -> bool:
    return 1 <= len(s) <= MAX_MANIFEST_ID_LEN


def _validate_layout_node(node: Any, panel_ids: set[str], seen: set[str]) -> Any:
    """Depth-first layout validation + rebuild (see validateLayout in
    manifest.ts). A node's own structure (row/col exclusivity, children,
    shares) is checked before its children; string leaves check unknown
    before duplicate. `seen` is shared with the pages loop so one panel can
    be placed only once across the whole cockpit. Unknown node keys are
    dropped by the rebuild (global unknown-key policy)."""
    if isinstance(node, str):
        if node not in panel_ids:
            raise ManifestError("unknown_layout_panel", f"layout names unknown panel {node}")
        if node in seen:
            raise ManifestError("duplicate_layout_panel", f"panel {node} placed more than once")
        seen.add(node)
        return node
    if not isinstance(node, dict):
        raise ManifestError("invalid_layout", "layout node must be a panel id or a row/col object")
    is_row = "row" in node
    if is_row == ("col" in node):
        raise ManifestError("invalid_layout", "layout node needs exactly one of row/col")
    children = node["row"] if is_row else node["col"]
    if not isinstance(children, list) or not children:
        raise ManifestError("invalid_layout", "row/col children must be a non-empty array")
    shares = None
    if "shares" in node:
        shares = node["shares"]
        # bool is excluded explicitly (Python bool is an int); the float64
        # bound mirrors JS Number.isFinite, which rejects huge JSON ints
        # that parse to Infinity there.
        if (
            not isinstance(shares, list)
            or len(shares) != len(children)
            or any(
                isinstance(s, bool)
                or not isinstance(s, (int, float))
                or not 0 < s <= sys.float_info.max
                for s in shares
            )
        ):
            raise ManifestError("invalid_layout", "shares must be positive numbers, one per child")
    rebuilt = [_validate_layout_node(child, panel_ids, seen) for child in children]
    out: dict[str, Any] = {"row" if is_row else "col": rebuilt}
    if shares is not None:
        out["shares"] = shares
    return out


def parse_manifest(data: Any) -> Manifest:
    """Validated manifest from parsed JSON (or any untrusted value); raises
    ManifestError. Absent dir/params/title/layout/pages normalize to
    "rx"/{}/""/None/[]; unknown keys are dropped (forward compatibility
    with newer bridges). Mirrors parseManifest() in manifest.ts: shape,
    then domain rules in document order, so both sides report the same code
    (pinned by fixtures). The version gate runs before the channel/panel
    shape checks so a structurally-alien future manifest still reports
    unsupported_version, which the Cockpit turns into a polite notice.
    """
    # Through JSON, so the hand checks and the strict model see the same
    # JSON-shaped values a peer would put on the wire (strict python-mode
    # validation wants model instances for nested fields, but callers hold
    # parsed-JSON dicts).
    try:
        raw = json.loads(json.dumps(data))
    except (TypeError, ValueError) as e:
        raise ManifestError("invalid_shape", str(e)) from e
    if not isinstance(raw, dict):
        raise ManifestError("invalid_shape", "manifest must be an object")
    version = raw.get("version")
    if isinstance(version, bool) or not isinstance(version, (int, float)):
        raise ManifestError("invalid_shape", "manifest version must be a number")
    if version != MANIFEST_VERSION:
        raise ManifestError("unsupported_version", f"manifest version {version} is not supported")
    try:
        manifest = _MANIFEST_TA.validate_json(
            json.dumps({k: v for k, v in raw.items() if k != "pages"})
        )
    except ValidationError as e:
        raise ManifestError("invalid_shape", str(e)) from e

    ch_ids: dict[str, ChannelSpec] = {}
    for spec in manifest.channels:
        if not _bounded_id(spec.ch):
            raise ManifestError(
                "invalid_channel_id", f"channel id must be 1..{MAX_MANIFEST_ID_LEN} chars"
            )
        if spec.ch.startswith(RESERVED_CHANNEL_PREFIX):
            raise ManifestError(
                "reserved_channel_id",
                f"channel ids beginning with {RESERVED_CHANNEL_PREFIX} are reserved "
                "for protocol control",
            )
        if spec.ch in ch_ids:
            raise ManifestError("duplicate_channel_id", f"duplicate channel {spec.ch}")
        ch_ids[spec.ch] = spec
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
        # Kind-specific rules (incl. dir=rx for viewer-rendered channels);
        # unknown kinds stay unvalidated (forward compatibility with newer
        # bridges).
        if panel.kind == "video":
            # One feed, or two: the second is drawn inset over the first
            # (picture-in-picture). Both must be real video.
            if not 1 <= len(panel.channels) <= 2:
                raise ManifestError(
                    "invalid_video_panel",
                    f"video panel {panel.id} must bind one channel, or two for an inset",
                )
            for ch in panel.channels:
                bound = ch_ids[ch]
                if bound.encoding != "jpeg.v1" or bound.delivery != "latest" or bound.dir != "rx":
                    raise ManifestError(
                        "invalid_video_panel",
                        f"video panel {panel.id} needs a jpeg.v1 latest rx channel",
                    )
        if panel.kind == "map2d":
            # channels[0] is the costmap; channels[1] (optional) the pose overlay.
            if len(panel.channels) not in (1, 2):
                raise ManifestError(
                    "invalid_map2d_panel",
                    f"map2d panel {panel.id} must bind one or two channels",
                )
            costmap = ch_ids[panel.channels[0]]
            if (
                costmap.encoding != "costmap.zlib.v1"
                or costmap.delivery != "latest"
                or costmap.dir != "rx"
            ):
                raise ManifestError(
                    "invalid_map2d_panel",
                    f"map2d panel {panel.id} needs a costmap.zlib.v1 latest rx channel first",
                )
            if len(panel.channels) == 2:
                pose = ch_ids[panel.channels[1]]
                if pose.encoding != "pose.json.v1" or pose.dir != "rx":
                    raise ManifestError(
                        "invalid_map2d_panel",
                        f"map2d panel {panel.id} pose channel must be a pose.json.v1 rx channel",
                    )
        if panel.kind == "teleop":
            if len(panel.channels) != 1:
                raise ManifestError(
                    "invalid_teleop_panel",
                    f"teleop panel {panel.id} must bind exactly one channel",
                )
            cmd = ch_ids[panel.channels[0]]
            if cmd.encoding != "twist.json.v1" or cmd.delivery != "latest" or cmd.dir != "tx":
                raise ManifestError(
                    "invalid_teleop_panel",
                    f"teleop panel {panel.id} needs a twist.json.v1 latest tx channel",
                )
        if panel.kind in _SLOT_KINDS:
            code, slots = _SLOT_KINDS[panel.kind]
            if len(panel.channels) != len(slots):
                raise ManifestError(
                    code, f"{panel.kind} panel {panel.id} must bind exactly {len(slots)} channels"
                )
            for i, (dir_, encoding) in enumerate(slots):
                bound = ch_ids[panel.channels[i]]
                if bound.encoding != encoding or bound.dir != dir_:
                    raise ManifestError(
                        code,
                        f"{panel.kind} panel {panel.id} channel {i} must be a {encoding} "
                        f"{dir_} channel",
                    )

    seen: set[str] = set()
    if manifest.layout is not None:
        manifest.layout = _validate_layout_node(manifest.layout, panel_ids, seen)

    raw_pages = raw.get("pages", [])
    if not isinstance(raw_pages, list) or any(not isinstance(p, str) for p in raw_pages):
        raise ManifestError("invalid_shape", "bad pages shape")
    for page_id in raw_pages:
        if page_id not in panel_ids:
            raise ManifestError("unknown_page_panel", f"pages names unknown panel {page_id}")
        if page_id in seen:
            raise ManifestError("duplicate_layout_panel", f"panel {page_id} placed more than once")
        seen.add(page_id)
    manifest.pages = raw_pages

    return manifest
