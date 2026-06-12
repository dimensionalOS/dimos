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

from dimos.visualization.rerun.reflex_selector_app.selector_app.selector_app import (
    enrich_row,
    filter_rows,
    fmt_ago,
    fmt_bw,
    fmt_hz,
    grouped_display_rows,
    is_heavy,
    render_badge,
    row_counts,
    selection_dirty,
    staged_topics_after_toggle,
    status_text,
    topic_group,
)


def test_filter_rows_applies_search_and_selector_filters() -> None:
    rows = [
        {
            "channel": "/camera/color",
            "name": "camera",
            "type_name": "Image",
            "renderability": "renderable",
            "live": True,
            "selected": False,
            "logging": False,
        },
        {
            "channel": "/debug/text",
            "name": "debug",
            "type_name": "str",
            "renderability": "unsupported",
            "live": False,
            "selected": True,
            "logging": False,
        },
        {
            "channel": "/pose",
            "name": "pose",
            "type_name": "PoseStamped",
            "renderability": "renderable",
            "live": True,
            "selected": False,
            "logging": True,
        },
    ]

    assert [row["channel"] for row in filter_rows(rows, search_query="pose")] == ["/pose"]
    assert [
        row["channel"]
        for row in filter_rows(rows, renderable_only=True, live_only=True, selected_only=True)
    ] == ["/pose"]


def test_row_counts_and_status_text_describe_catalog_state() -> None:
    rows = [
        {
            "selected": True,
            "logging": False,
            "live": True,
            "renderability": "renderable",
            "type_name": "A",
        },
        {
            "selected": False,
            "logging": True,
            "live": False,
            "renderability": "unsupported",
            "type_name": "B",
        },
    ]

    staged_count, logging_count, live_count, renderable_count = row_counts(rows)

    assert (staged_count, logging_count, live_count, renderable_count) == (1, 1, 1, 1)
    assert status_text(rows, live_count=live_count, renderable_count=renderable_count) == (
        "LCM 2 channels · 1 live · 1 renderable"
    )
    assert "No decodable LCM data" in status_text([], live_count=0, renderable_count=0)
    assert "no observed topic has a Rerun converter" in status_text(
        [{"type_name": "A"}], live_count=1, renderable_count=0
    )


def test_staged_topics_after_toggle_preserves_other_staged_topics() -> None:
    rows = [
        {"channel": "/a", "selected": True},
        {"channel": "/b", "selected": True},
        {"channel": "/c", "selected": False},
    ]

    assert staged_topics_after_toggle(rows, "/b", False) == ["/a"]
    assert staged_topics_after_toggle(rows, "/c", True) == ["/a", "/b", "/c"]


def test_filter_rows_heavy_only_keeps_heavy_rows() -> None:
    rows = [
        {"channel": "/camera", "heavy": True},
        {"channel": "/pose", "heavy": False},
    ]

    assert [row["channel"] for row in filter_rows(rows, heavy_only=True)] == ["/camera"]


def test_topic_group_assigns_design_groups() -> None:
    assert topic_group("/color_image", "sensor_msgs.Image") == "Perception"
    assert topic_group("/goal_pose", "geometry_msgs.PoseStamped") == "Navigation"
    assert topic_group("/cmd_vel", "geometry_msgs.Twist") == "Control"
    assert topic_group("/odom", "nav_msgs.Odometry") == "Robot state"
    assert topic_group("/agent_log", "str") == "Text / logs"
    assert topic_group("GO2_LOW_STATE", None) == "Other"


def test_is_heavy_uses_bandwidth_and_bulky_types() -> None:
    assert is_heavy({"bandwidth_bps": 60_000.0, "type_name": "str"})
    assert is_heavy({"bandwidth_bps": 100.0, "type_name": "sensor_msgs.Image"})
    assert not is_heavy({"bandwidth_bps": 100.0, "type_name": "geometry_msgs.PoseStamped"})


def test_render_badge_maps_renderability_to_badge() -> None:
    assert render_badge("renderable", "native to_rerun()") == ("native", "renderable")
    assert render_badge("renderable", "visual converter") == ("converter", "converter")
    assert render_badge("unsupported", "no Rerun converter") == ("unsupported", "unsupported")
    assert render_badge("unknown", "unknown message type") == ("unsupported", "unsupported")


def test_formatters_match_design_copy() -> None:
    assert fmt_hz(0) == "—"
    assert fmt_hz(0.4) == "0.4 Hz"
    assert fmt_hz(14.8) == "15 Hz"
    assert fmt_bw(0) == "—"
    assert fmt_bw(420.0) == "420 B/s"
    assert fmt_bw(36_000.0) == "36 kB/s"
    assert fmt_bw(2_200_000.0) == "2.2 MB/s"
    assert fmt_ago(None) == "never"
    assert fmt_ago(1.0) == "now"
    assert fmt_ago(42.0) == "42s ago"
    assert fmt_ago(180.0) == "3m ago"


def test_enrich_row_precomputes_display_fields() -> None:
    raw = {
        "channel": "/color_image#sensor_msgs.Image",
        "name": "/color_image",
        "type_name": "sensor_msgs.Image",
        "renderability": "renderable",
        "render_reason": "native to_rerun()",
        "live": True,
        "selected": True,
        "logging": True,
        "last_seen_monotonic": 99.0,
        "message_count": 10,
        "rate_hz": 14.8,
        "bandwidth_bps": 2_200_000.0,
        "average_message_size_bytes": 148_000.0,
        "last_error": None,
    }

    row = enrich_row(raw, now_monotonic=100.0)

    assert row["group"] == "Perception"
    assert row["selectable"] is True
    assert row["heavy"] is True
    assert row["row_class"] == "vc-row is-staged"
    assert row["check_class"] == "vc-check is-checked is-applied"
    assert row["badge_class"] == "vc-badge b-native"
    assert row["bw_class"] == "num mono is-heavy"
    assert row["rate_text"] == "15 Hz"
    assert row["bw_text"] == "2.2 MB/s"
    assert row["state_label"] == "live"
    assert row["state_title"] == "last seen now"


def test_enrich_row_disables_non_renderable_topics() -> None:
    raw = {
        "channel": "GO2_LOW_STATE",
        "name": "GO2_LOW_STATE",
        "type_name": None,
        "renderability": "unsupported",
        "render_reason": "no Rerun converter",
        "live": False,
        "selected": False,
        "logging": False,
        "last_seen_monotonic": 0.0,
        "message_count": 1,
        "rate_hz": 0.0,
        "bandwidth_bps": 0.0,
        "average_message_size_bytes": 0.0,
        "last_error": None,
    }

    row = enrich_row(raw, now_monotonic=1.0)

    assert row["group"] == "Other"
    assert row["selectable"] is False
    assert row["row_class"] == "vc-row is-disabled"
    assert row["badge_label"] == "unsupported"
    assert row["row_title"] == "no Rerun converter"
    assert row["rate_text"] == "—"


def test_grouped_display_rows_orders_groups_with_headers() -> None:
    rows = [
        {"kind": "topic", "channel": "/log", "group": "Text / logs"},
        {"kind": "topic", "channel": "/image", "group": "Perception"},
        {"kind": "topic", "channel": "/depth", "group": "Perception"},
    ]

    flattened = grouped_display_rows(rows)

    assert [item.get("group") for item in flattened if item["kind"] == "group"] == [
        "Perception",
        "Text / logs",
    ]
    assert flattened[0] == {"kind": "group", "group": "Perception", "count": 2}
    assert [item["channel"] for item in flattened if item["kind"] == "topic"] == [
        "/image",
        "/depth",
        "/log",
    ]


def test_selection_dirty_compares_staged_and_applied_sets() -> None:
    clean = [
        {"channel": "/a", "selected": True, "logging": True},
        {"channel": "/b", "selected": False, "logging": False},
    ]
    staged_extra = [
        {"channel": "/a", "selected": True, "logging": True},
        {"channel": "/b", "selected": True, "logging": False},
    ]
    cleared = [{"channel": "/a", "selected": False, "logging": True}]

    assert not selection_dirty(clean)
    assert selection_dirty(staged_extra)
    assert selection_dirty(cleared)
