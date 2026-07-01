#!/usr/bin/env python3
# Unit tests for the optional QoS config-map layer in qos_sched.py — the topic-glob → lane rules that
# classify custom per-blueprint topics the name/type heuristic can't. Covers rule precedence, glob +
# "#type" matching, the safe default, and a missing rules file.
#
# qos_sched.py is stdlib-only, so we import it directly off its own directory (not a published package).
# Run: uv run pytest dimos/web/dimoscope/servers/test_qos.py -q
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from qos_sched import (
    LANE_BULK,
    LANE_COMMAND,
    LANE_DEFAULT,
    LANE_SENSOR,
    QosRule,
    default_priority,
    load_qos_rules,
    set_qos_rules,
)


def teardown_function():
    set_qos_rules([])  # never leak rules between tests (module-global _RULES)


def test_no_rules_falls_back_to_heuristic():
    set_qos_rules([])
    assert default_priority("/cmd_vel") == LANE_COMMAND
    assert default_priority("/lidar") == LANE_BULK
    assert default_priority("/pose") == LANE_SENSOR
    assert default_priority("/some/random/topic") == LANE_DEFAULT  # unknown → safe middle


def test_rule_beats_heuristic():
    # a topic the heuristic would call BULK (matches /lidar) is forced to SENSOR by an explicit rule
    set_qos_rules([QosRule("/lidar", "sensor")])
    assert default_priority("/lidar") == LANE_SENSOR


def test_glob_matches_subtree():
    set_qos_rules([QosRule("/blueprintX/*", "command")])
    assert default_priority("/blueprintX/status") == LANE_COMMAND
    assert (
        default_priority("/blueprintY/status") == LANE_DEFAULT
    )  # not matched → heuristic → default


def test_first_match_wins():
    set_qos_rules([QosRule("/a/*", "sensor"), QosRule("/a/hot", "command")])
    assert default_priority("/a/hot") == LANE_SENSOR  # earlier rule wins even though both match


def test_type_pattern_matches_on_topic_hash_type():
    # a '#' in the pattern matches "<topic>#<type>" — classify any custom-named topic by its message type
    set_qos_rules([QosRule("*#my_msgs.RobotState", "sensor")])
    assert default_priority("/blueprintX/status", "my_msgs.RobotState") == LANE_SENSOR
    assert default_priority("/blueprintX/status", "my_msgs.Other") == LANE_DEFAULT


def test_unknown_lane_is_dropped():
    set_qos_rules([QosRule("/x", "bogus")])
    assert default_priority("/x") == LANE_DEFAULT  # bad-lane rule ignored → heuristic → default


def test_load_missing_file_is_noop():
    assert load_qos_rules("/no/such/qos.rules.json") == 0


def test_load_json_file(tmp_path):
    p = tmp_path / "qos.rules.json"
    p.write_text(
        '[{"topic": "/cam/*", "lane": "bulk"}, {"topic": "*#geometry_msgs.Twist", "lane": "command"}]'
    )
    assert load_qos_rules(p) == 2
    assert default_priority("/cam/front") == LANE_BULK
    assert default_priority("/anything", "geometry_msgs.Twist") == LANE_COMMAND


def test_load_skips_comment_entries(tmp_path):
    # a leading {"_comment": ...} entry (as in qos.rules.example.json) is tolerated, not fatal
    p = tmp_path / "qos.rules.json"
    p.write_text('[{"_comment": "docs"}, {"topic": "/x", "lane": "sensor"}]')
    assert load_qos_rules(p) == 1
    assert default_priority("/x") == LANE_SENSOR


def test_load_malformed_json_is_noop(tmp_path):
    p = tmp_path / "qos.rules.json"
    p.write_text("{ not json")
    assert load_qos_rules(p) == 0
