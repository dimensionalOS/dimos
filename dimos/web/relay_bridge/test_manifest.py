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

"""Golden-fixture tests keeping manifest.py behavior identical to manifest.ts."""

import json

import pytest

from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.manifest import ManifestError, parse_manifest

with open(find_web_dir() / "shared" / "fixtures" / "manifests.json") as f:
    VECTORS = json.load(f)["vectors"]

VALID = [v for v in VECTORS if "manifest" in v]
INVALID = [v for v in VECTORS if "error" in v]


@pytest.mark.parametrize("vector", VALID, ids=[v["name"] for v in VALID])
def test_valid_manifest_normalizes_per_golden(vector):
    assert parse_manifest(vector["data"]).model_dump() == vector["manifest"]


@pytest.mark.parametrize("vector", INVALID, ids=[v["name"] for v in INVALID])
def test_invalid_manifest_rejected_with_pinned_code(vector):
    with pytest.raises(ManifestError) as exc_info:
        parse_manifest(vector["data"])
    assert exc_info.value.code == vector["error"]


def test_every_vector_is_classified():
    assert VALID and INVALID
    assert len(VALID) + len(INVALID) == len(VECTORS)


def test_huge_int_max_hz_rejected_like_ts():
    # Not a golden vector: gen.ts cannot emit a number beyond float64
    # (JSON.stringify(Infinity) is null). Python parses such a JSON integer
    # exactly while JS JSON.parse yields Infinity; both sides must reject it
    # with the same code (manifest_test.ts pins the JS half).
    data = {
        "version": 1,
        "channels": [
            {"ch": "odom", "encoding": "pose.json.v1", "delivery": "reliable", "maxHz": 10**400}
        ],
    }
    with pytest.raises(ManifestError) as exc_info:
        parse_manifest(data)
    assert exc_info.value.code == "invalid_max_hz"


def test_huge_int_share_rejected_like_ts():
    # Same non-golden reasoning, for the layout walker's share bound
    # (manifest_test.ts pins the JS half with Infinity).
    data = {
        "version": 1,
        "channels": [
            {"ch": "odom", "encoding": "pose.json.v1", "delivery": "reliable", "maxHz": 20.5}
        ],
        "panels": [{"id": "pose", "kind": "readout", "channels": ["odom"]}],
        "layout": {"row": ["pose"], "shares": [10**400]},
    }
    with pytest.raises(ManifestError) as exc_info:
        parse_manifest(data)
    assert exc_info.value.code == "invalid_layout"


def test_integral_float_version_accepted_like_ts():
    # Not a golden vector: 1.0 renders as "1" through JS JSON.stringify, so
    # only the Python side can pin that a float 1.0 passes the version gate
    # the way JS `1.0 === 1` does.
    manifest = parse_manifest({"version": 1.0, "channels": []})
    assert manifest.version == 1


# Slot-table kinds (chat / navmap / control): the shapes dimos/web/cockpit.py
# authors for the microduck cockpit. Golden vectors pin the cross-language
# codes; these spell the rules out per kind so a slot-table edit here cannot
# silently drift from what cockpit.py emits.

SLOT_SHAPES = {
    "chat": (
        "invalid_chat_panel",
        [
            ("agent", "rx", "chat.json.v1", "reliable"),
            ("agent_idle", "rx", "flag.json.v1", "reliable"),
            ("mode", "rx", "mode.json.v1", "reliable"),
            ("human_input", "tx", "text.json.v1", "reliable"),
        ],
    ),
    "navmap": (
        "invalid_navmap_panel",
        [
            ("global_costmap", "rx", "costmap.zlib.v1", "latest"),
            ("odom", "rx", "pose.json.v1", "reliable"),
            ("path", "rx", "path.json.v1", "latest"),
            ("places", "rx", "places.json.v1", "reliable"),
            ("nav_state", "rx", "navstate.json.v1", "latest"),
            ("goal_request", "tx", "pose_goal.json.v1", "reliable"),
            ("ui_command", "tx", "command.json.v1", "reliable"),
        ],
    ),
    "control": (
        "invalid_control_panel",
        [
            ("mode", "rx", "mode.json.v1", "reliable"),
            ("policy_state", "rx", "policy.json.v1", "latest"),
            ("nav_state", "rx", "navstate.json.v1", "latest"),
            ("ui_command", "tx", "command.json.v1", "reliable"),
        ],
    ),
}


def slot_manifest(kind: str, **overrides) -> dict:
    """A one-panel manifest of `kind` whose channels follow SLOT_SHAPES;
    `overrides` patch one channel by index: {2: {"dir": "tx"}}."""
    _, shape = SLOT_SHAPES[kind]
    channels = [
        {"ch": ch, "dir": direction, "encoding": encoding, "delivery": delivery, "maxHz": 10.0}
        for ch, direction, encoding, delivery in shape
    ]
    for index, patch in overrides.items():
        channels[int(index)].update(patch)
    return {
        "version": 1,
        "channels": channels,
        "panels": [{"id": "p0", "kind": kind, "channels": [c["ch"] for c in channels]}],
    }


@pytest.mark.parametrize("kind", sorted(SLOT_SHAPES))
def test_slot_kind_accepts_its_shape(kind):
    manifest = parse_manifest(slot_manifest(kind))
    (panel,) = manifest.panels
    assert panel.kind == kind
    assert len(panel.channels) == len(SLOT_SHAPES[kind][1])


@pytest.mark.parametrize("kind", sorted(SLOT_SHAPES))
def test_slot_kind_ignores_delivery(kind):
    # Delivery is the bridge's per-channel choice, not part of the slot rule.
    n = len(SLOT_SHAPES[kind][1])
    flipped = {
        i: {"delivery": "latest" if SLOT_SHAPES[kind][1][i][3] == "reliable" else "reliable"}
        for i in range(n)
    }
    parse_manifest(slot_manifest(kind, **{str(k): v for k, v in flipped.items()}))


@pytest.mark.parametrize("kind", sorted(SLOT_SHAPES))
def test_slot_kind_rejects_wrong_channel_count(kind):
    code, shape = SLOT_SHAPES[kind]
    data = slot_manifest(kind)
    panel = data["panels"][0]
    for channels in ([], panel["channels"][:-1], [*panel["channels"], panel["channels"][0]]):
        panel["channels"] = channels
        with pytest.raises(ManifestError) as exc_info:
            parse_manifest(data)
        assert exc_info.value.code == code, channels


@pytest.mark.parametrize("kind", sorted(SLOT_SHAPES))
def test_slot_kind_rejects_wrong_encoding_in_every_slot(kind):
    code, shape = SLOT_SHAPES[kind]
    for i in range(len(shape)):
        with pytest.raises(ManifestError) as exc_info:
            parse_manifest(slot_manifest(kind, **{str(i): {"encoding": "jpeg.v1"}}))
        assert exc_info.value.code == code, i


@pytest.mark.parametrize("kind", sorted(SLOT_SHAPES))
def test_slot_kind_rejects_wrong_dir_in_every_slot(kind):
    code, shape = SLOT_SHAPES[kind]
    for i, (_, direction, _, _) in enumerate(shape):
        flipped = "tx" if direction == "rx" else "rx"
        with pytest.raises(ManifestError) as exc_info:
            parse_manifest(slot_manifest(kind, **{str(i): {"dir": flipped}}))
        assert exc_info.value.code == code, i


@pytest.mark.parametrize("kind", sorted(SLOT_SHAPES))
def test_slot_kind_rejects_swapped_slots(kind):
    code, _ = SLOT_SHAPES[kind]
    data = slot_manifest(kind)
    channels = data["panels"][0]["channels"]
    channels[0], channels[1] = channels[1], channels[0]
    with pytest.raises(ManifestError) as exc_info:
        parse_manifest(data)
    assert exc_info.value.code == code


def test_slot_kind_unknown_channel_reported_before_slot_rule():
    data = slot_manifest("control")
    data["panels"][0]["channels"][0] = "missing"
    with pytest.raises(ManifestError) as exc_info:
        parse_manifest(data)
    assert exc_info.value.code == "unknown_panel_channel"


def test_slot_kinds_share_channels_across_panels():
    # The microduck cockpit binds mode/nav_state/ui_command from both the
    # control bar and the nav map; each panel is validated independently.
    control = slot_manifest("control")
    navmap = slot_manifest("navmap")
    by_ch = {c["ch"]: c for c in [*control["channels"], *navmap["channels"]]}
    data = {
        "version": 1,
        "channels": list(by_ch.values()),
        "panels": [
            {**control["panels"][0], "id": "p0"},
            {**navmap["panels"][0], "id": "p1"},
        ],
        "layout": {"col": ["p0", "p1"]},
    }
    manifest = parse_manifest(data)
    assert [p.kind for p in manifest.panels] == ["control", "navmap"]


@pytest.mark.parametrize("kind", sorted(SLOT_SHAPES))
def test_golden_vectors_cover_slot_kind(kind):
    # The cross-language contract for these kinds lives in the fixtures;
    # both an accepted and a rejected vector must exist per kind.
    code, _ = SLOT_SHAPES[kind]
    assert any(any(p.get("kind") == kind for p in v["data"].get("panels", [])) for v in VALID), kind
    assert any(v["error"] == code for v in INVALID), code
