"""Tests for the pure subject-tracking module: metrics + state machine."""

import json
import math

from pawtrack.track_state import (
    MonitorParams,
    MonitorState,
    TrackMetrics,
    TrackSnapshot,
    VisualObservation,
    bbox_in_image,
    clamp_bbox,
    ground_contact_pixel,
    is_bbox_shape,
    valid_bbox,
    visual_metrics,
)


def _metrics(center=(20.0, 20.0), width=20.0, area=400.0):
    cx, cy = center
    half = width / 2.0
    return TrackMetrics(
        bbox=(cx - half, cy - half, cx + half, cy + half),
        center_px=(cx, cy), width_px=width, height_px=width,
        area_px=area, area_ratio=area / (640 * 480),
        image_error_x=0.0, image_error_y=0.0, confidence=0.9,
        mask_area_px=area * 0.8, mask_area_ratio=0.0,
    )


def test_visual_metrics_reports_size_and_center_error():
    metrics = visual_metrics(
        VisualObservation(
            bbox=(40.0, 20.0, 80.0, 60.0),
            image_width=200,
            image_height=100,
            confidence=0.8,
            mask_area_px=1200.0,
        )
    )

    assert metrics.center_px == (60.0, 40.0)
    assert metrics.width_px == 40.0
    assert metrics.height_px == 40.0
    assert metrics.area_px == 1600.0
    assert metrics.area_ratio == 0.08
    assert metrics.image_error_x == -0.4
    assert metrics.image_error_y == -0.2
    assert metrics.mask_area_ratio == 0.06


def test_visual_metrics_rejects_invalid_bbox():
    observation = VisualObservation(
        bbox=(10.0, 10.0, 10.0, 20.0),
        image_width=100,
        image_height=100,
    )

    try:
        visual_metrics(observation)
    except ValueError as err:
        assert "bbox" in str(err)
    else:
        raise AssertionError("invalid bbox should raise")


def test_ground_contact_pixel_is_bbox_bottom_center():
    # The subject meets the floor at the bottom-center of its box, not the
    # center -- a seated person's box center is torso/seat height.
    assert ground_contact_pixel((100.0, 40.0, 140.0, 200.0)) == (120.0, 200.0)


def test_is_bbox_shape_accepts_four_finite_numbers():
    assert is_bbox_shape([10.0, 20.0, 30.0, 40.0])
    assert is_bbox_shape((10, 20, 30, 40))


def test_is_bbox_shape_rejects_malformed():
    assert not is_bbox_shape(None)
    assert not is_bbox_shape([1, 2])  # too short -- would crash on index 2/3
    assert not is_bbox_shape([1, 2, 3, 4, 5])  # too long
    assert not is_bbox_shape(["a", "b", "c", "d"])  # non-numeric
    assert not is_bbox_shape([1.0, 2.0, math.inf, 4.0])  # not finite


def test_bbox_in_image_rejects_degenerate_and_off_frame():
    assert bbox_in_image((10.0, 10.0, 50.0, 50.0), 100, 100)
    assert not bbox_in_image((50.0, 10.0, 50.0, 50.0), 100, 100)  # zero width
    assert not bbox_in_image((-50.0, -50.0, -10.0, -10.0), 100, 100)  # off
    assert bbox_in_image((-20.0, -20.0, 30.0, 30.0), 100, 100)  # partial


def test_valid_bbox_combines_shape_and_overlap():
    assert valid_bbox([10.0, 10.0, 50.0, 50.0], 100, 100)
    assert not valid_bbox([1, 2], 100, 100)  # bad shape, no crash
    assert not valid_bbox([200.0, 200.0, 300.0, 300.0], 100, 100)  # off-frame


def test_clamp_bbox_keeps_in_bounds_and_clamps_overflow():
    assert clamp_bbox((10.0, 10.0, 50.0, 50.0), 100, 100) == (
        10.0, 10.0, 50.0, 50.0)
    # A box spilling past two edges clamps to the visible portion, staying
    # non-degenerate (x2 > x1, y2 > y1) so EdgeTAM init gets a usable box.
    clamped = clamp_bbox((-20.0, -20.0, 130.0, 130.0), 100, 100)
    assert clamped == (0.0, 0.0, 100.0, 100.0)
    edge = clamp_bbox((90.0, 90.0, 130.0, 130.0), 100, 100)
    assert edge == (90.0, 90.0, 100.0, 100.0)
    assert edge[2] > edge[0] and edge[3] > edge[1]


def test_begin_is_acquiring():
    snap = MonitorState().begin("a person sitting on a chair")
    assert snap.status == "acquiring"
    assert snap.description == "a person sitting on a chair"


def test_tracks_then_lost_retains_last_fix():
    state = MonitorState()
    state.begin("person")
    snap, action = state.observe(
        "person", _metrics(center=(20.0, 20.0)), 100.0)
    assert snap.status == "tracking" and action == "track"
    lost, action = state.observe("person", None, 100.5)
    assert lost.status == "lost" and action == "coast"
    assert lost.center_px == (20.0, 20.0) and lost.seen_at == 100.0


def test_lost_with_no_prior_track_has_no_bbox():
    state = MonitorState()
    state.begin("person")
    lost, _ = state.observe("person", None, 0.0)
    assert lost.status == "lost" and lost.bbox is None


def test_drift_gate_rejects_a_huge_center_jump():
    state = MonitorState()
    state.begin("person")
    state.observe("person", _metrics(center=(100.0, 100.0), width=20.0), 0.0)
    snap, action = state.observe(
        "person", _metrics(center=(400.0, 400.0), width=20.0), 0.1)
    # ~424px jump >> 2.5 * 20 -> rejected as drift, treated as a miss.
    assert snap.status == "lost" and action == "coast"
    assert snap.center_px == (100.0, 100.0)  # kept the last good fix


def test_drift_gate_rejects_area_explosion():
    state = MonitorState()
    state.begin("person")
    state.observe("person", _metrics(center=(100.0, 100.0), area=400.0), 0.0)
    snap, _ = state.observe(
        "person", _metrics(center=(100.0, 100.0), area=4000.0), 0.1)
    assert snap.status == "lost"  # 10x area jump rejected


def test_drift_gate_allows_normal_motion():
    state = MonitorState()
    state.begin("person")
    state.observe("person", _metrics(center=(100.0, 100.0), width=20.0), 0.0)
    snap, action = state.observe(
        "person", _metrics(center=(120.0, 100.0), width=20.0), 0.1)
    assert snap.status == "tracking" and action == "track"
    assert snap.center_px == (120.0, 100.0)


def test_marks_stale_after_timeout():
    state = MonitorState(
        MonitorParams(stale_timeout_s=1.0, max_lost_frames=999))
    state.begin("person")
    state.observe("person", _metrics(), 0.0)
    fresh, _ = state.observe("person", None, 0.5)
    assert fresh.status == "lost"  # still within the timeout
    stale, _ = state.observe("person", None, 2.0)
    assert stale.status == "stale"  # aged out
    assert stale.center_px == (20.0, 20.0)  # still carries the last fix


def test_reacquired_skips_drift_gate():
    state = MonitorState()
    state.begin("person")
    state.observe("person", _metrics(center=(50.0, 50.0)), 0.0)
    state.reacquired()  # VLM/motion found it, possibly far away
    snap, action = state.observe(
        "person", _metrics(center=(500.0, 400.0)), 0.2)
    assert snap.status == "tracking" and action == "track"
    assert snap.center_px == (500.0, 400.0)  # far jump accepted post-reacquire


def test_reacquire_cadence_then_give_up():
    state = MonitorState(MonitorParams(
        max_lost_frames=3, reacquire_interval_frames=2,
        max_reacquire_attempts=2, stale_timeout_s=1e9,
    ))
    state.observe("person", _metrics(), 0.0)  # establish baseline
    actions = [state.observe("person", None, 0.0)[1] for _ in range(7)]
    assert actions == [
        "coast", "coast", "reacquire", "coast", "reacquire", "coast", "give_up"
    ]


def test_reacquired_resets_lost_counter():
    state = MonitorState(MonitorParams(
        max_lost_frames=3, reacquire_interval_frames=2, stale_timeout_s=1e9))
    state.observe("person", _metrics(), 0.0)
    for _ in range(3):
        state.observe("person", None, 0.0)  # 3rd is reacquire-due
    state.reacquired()
    assert state.observe("person", None, 0.0)[1] == "coast"


def test_stopped_and_errored():
    state = MonitorState()
    state.observe("person", _metrics(), 5.0)
    err = state.errored("person", "Monitor loop error; see logs.")
    assert err.status == "error" and err.center_px == (20.0, 20.0)
    assert err.message == "Monitor loop error; see logs."
    stopped = state.stopped()
    assert stopped.status == "stopped" and stopped.bbox is None


def test_json_schema_drops_seen_at_adds_age():
    snap = TrackSnapshot(
        status="tracking", description="person", message="Tracking person.",
        seen_at=10.0, bbox=(1.0, 2.0, 3.0, 4.0),
    )
    data = json.loads(snap.to_json(now=12.5))
    assert "seen_at" not in data
    assert data["last_seen_age_s"] == 2.5
    assert data["status"] == "tracking"
    assert data["bbox"] == [1.0, 2.0, 3.0, 4.0]
    idle = TrackSnapshot(status="idle", description=None, message="-")
    assert json.loads(idle.to_json())["last_seen_age_s"] is None
