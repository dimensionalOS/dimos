"""Orchestration tests for the greeter container.

Drive ``_tick`` synchronously with fakes for the models, the patrol spec, the
GO2 connection, and TF -- no GPU, robot, or threads -- and assert the full
wander -> approach -> greet -> cooldown -> wander cycle, the patrol stop/resume,
the Hello/BalanceStand greeting, and the don't-revisit-too-soon rule.
"""

import contextlib
import json
import math

import numpy as np
import pytest

from pawtrack import greeter_container
from pawtrack.greeter_container import GreeterSkillContainer
from pawtrack.greeter_state import (
    GreeterMachine,
    GreeterObservation,
    GreeterParams,
)
from pawtrack.ground_raycast import CameraIntrinsics
from pawtrack.visited_registry import Candidate

# Bottom-center (470, 240); with the look-down transform + intrinsics below this
# back-projects to ~1.5 m away -- a safe standoff (>= min_safe, <= standoff+tol).
_SUBJECT_BBOX = (450.0, 200.0, 490.0, 240.0)


class _Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _FakeImage:
    width = 640
    height = 480
    ts = 0.0

    def to_opencv(self):
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)


class _FakePatrol:
    def __init__(self, patrolling=False):
        self.patrolling = patrolling
        self.calls = []

    def start_patrol(self):
        self.patrolling = True
        self.calls.append("start")
        return "started"

    def stop_patrol(self):
        self.patrolling = False
        self.calls.append("stop")
        return "stopped"

    def is_patrolling(self):
        return self.patrolling


class _FakeConnection:
    def __init__(self):
        self.requests = []

    def publish_request(self, topic, data):
        self.requests.append((topic, data))
        return {}


class _Det:
    def __init__(self, bbox):
        self.bbox = bbox

    def bbox_2d_volume(self):
        x1, y1, x2, y2 = self.bbox
        return (x2 - x1) * (y2 - y1)


class _Dets:
    def __init__(self, detections):
        self.detections = detections

    def __len__(self):
        return len(self.detections)


class _FakeTracker:
    def __init__(self, track_bbox=None):
        self._track_bbox = track_bbox
        self.inited = []

    def init_track(self, image, box, obj_id=1):
        self.inited.append(tuple(box))
        return [1]

    def process_image(self, _image):
        if self._track_bbox is None:
            return _Dets([])
        return _Dets([_Det(self._track_bbox)])

    def stop(self):
        pass


class _FakeVl:
    def __init__(self, response, facing="back"):
        self.response = response
        self.facing = facing  # answer to the front/back facing prompt
        self.last_prompt = None

    def query(self, _image, prompt):
        self.last_prompt = prompt
        if "facing" in prompt.lower():  # the post-wave facing-detection query
            return self.facing
        return self.response

    def stop(self):
        pass


class _FakeTransform:
    def to_matrix(self):
        # Camera 1 m up looking straight down (origin at world x=y=0).
        return np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
        ])


class _FakeTf:
    def __init__(self, present=True):
        self._present = present

    def get(self, _frame, _child, **_kwargs):
        return _FakeTransform() if self._present else None


def _container(vl_response="[]", track_bbox=None, facing="back"):
    c = GreeterSkillContainer()
    c.cmd_vel = _Recorder()
    c.debug_image = _Recorder()
    c.greeter_phase = _Recorder()
    c.subject_world_pose = _Recorder()
    c._patrolling_module_spec = _FakePatrol()
    c._connection = _FakeConnection()
    c._intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=320.0, cy=240.0)
    c._tf = _FakeTf()
    c._latest_image = _FakeImage()
    c._vl_model = _FakeVl(vl_response, facing)
    c._tracker = _FakeTracker(track_bbox)
    c.config.recover_settle_s = 0.0  # no real RecoveryStand->BalanceStand wait
    # Short timers so the timed phases advance within the test.
    c._machine = GreeterMachine(GreeterParams(
        standoff_m=1.5, standoff_tolerance_m=0.3,
        greet_duration_s=0.5, cooldown_duration_s=0.5,
    ))
    return c


class _FakeVec:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _FakeOdom:
    """Minimal PoseStamped stand-in: position (x, y) + yaw (rad)."""

    def __init__(self, x, y, yaw=0.0):
        self.position = _FakeVec(x, y)
        self.yaw = yaw


class _FakeOrigin:
    def __init__(self, x, y):
        self.position = _FakeVec(x, y)


class _FakeCostmap:
    """Minimal OccupancyGrid stand-in: grid[y][x] costs, resolution, origin."""

    def __init__(self, grid, resolution, origin_xy):
        self.grid = grid
        self.resolution = resolution
        self.origin = _FakeOrigin(*origin_xy)


def _sport_ids(connection):
    return [data["api_id"] for _topic, data in connection.requests]


def test_wander_starts_patrol_and_stays_without_a_target():
    c = _container(vl_response="[]")  # nothing detected
    c._tick(2.0)  # past the first scan interval
    assert c._patrolling_module_spec.is_patrolling()
    assert c._machine.state == "wander"
    assert c.cmd_vel.msgs == []  # wander leaves driving to the patrol


def test_wander_rescans_when_all_visible_people_are_already_greeted():
    # The trap: the dog sees a person but already greeted them, so it won't
    # re-engage -- and the patrol is goal-less and blocked by that person. It
    # must rotate to look elsewhere instead of freezing.
    c = _container(vl_response='[{"bbox": [450, 200, 490, 240]}]')
    seen = c._scan(c._latest_image)  # where does the detected person land?
    assert seen  # a person is visible
    c._registry.mark_visited(seen[0].position)  # ... and already greeted
    c._patrolling_module_spec.patrolling = True  # patrol was (fruitlessly) roaming

    c.cmd_vel.msgs.clear()
    c._tick(2.0)  # wander: sees only the visited person

    assert c._machine.state == "wander"
    assert c._rescanning
    last = c.cmd_vel.msgs[-1]
    assert last.angular.z != 0.0 and last.linear.x == 0.0  # rotating in place
    assert "stop" in c._patrolling_module_spec.calls  # took over from the patrol
    assert not c._patrolling_module_spec.is_patrolling()


def test_wander_does_not_rescan_when_no_one_is_visible():
    c = _container(vl_response="[]")  # empty room in view
    c.cmd_vel.msgs.clear()
    c._tick(2.0)
    assert c._machine.state == "wander"
    assert c.cmd_vel.msgs == []  # no rescan; leave roaming to the patrol
    assert c._patrolling_module_spec.is_patrolling()
    assert not c._rescanning


def test_wander_rescan_is_bounded_and_hands_back_to_patrol():
    c = _container(vl_response='[{"bbox": [450, 200, 490, 240]}]')
    seen = c._scan(c._latest_image)
    c._registry.mark_visited(seen[0].position)
    c._rescanning = True
    c._rescan_started_s = 0.0  # already rotating since t=0

    c._tick(c.config.wander_rescan_max_s + 1.0)  # past the rescan budget

    assert not c._rescanning  # gave the rotation a fair shot
    assert c._patrolling_module_spec.is_patrolling()  # handed back to the patrol


def test_full_cycle_and_do_not_revisit():
    c = _container(
        vl_response='[{"bbox": [450, 200, 490, 240]}]',
        track_bbox=_SUBJECT_BBOX,
    )

    c._tick(2.0)  # wander: detect + select -> approach
    assert c._machine.state == "approach"
    assert "stop" in c._patrolling_module_spec.calls  # patrol paused to engage
    assert c._tracker.inited  # tracker locked onto the chosen subject

    c._tick(2.1)  # approach: at a safe standoff (~1.5 m) -> greet, any facing
    assert c._machine.state == "greet"
    assert _sport_ids(c._connection)[-1] == 1016  # Hello fired once
    assert c.cmd_vel.msgs[-1].is_zero()  # halted before the wave, not still moving

    c._tick(2.8)  # greet timer elapsed -> cooldown
    assert c._machine.state == "cooldown"
    assert 1006 in _sport_ids(c._connection)  # RecoveryStand after the gesture
    assert c._registry.is_visited((1.5, 0.0), now=2.8)  # subject recorded

    c._tick(3.4)  # cooldown timer elapsed -> wander
    assert c._machine.state == "wander"
    assert 1002 in _sport_ids(c._connection)  # BalanceStand before patrol resumes
    # The full recovery sequence ran in order: Hello -> RecoveryStand -> Balance.
    assert _sport_ids(c._connection) == [1016, 1006, 1002]

    # The just-greeted person is still within the forget window: detected again
    # but not re-selected.
    stop_count_before = c._patrolling_module_spec.calls.count("stop")
    c._tick(5.0)
    assert c._machine.state == "wander"
    assert c._patrolling_module_spec.calls.count("stop") == stop_count_before


def test_subject_is_re_greeted_after_the_forget_window():
    # The relaxed same-person rule: once revisit_forget_s passes, the same seated
    # person becomes greet-eligible again instead of being skipped for the run.
    c = _container(
        vl_response='[{"bbox": [450, 200, 490, 240]}]', track_bbox=_SUBJECT_BBOX)
    seen = c._scan(c._latest_image)
    c._registry.mark_visited(seen[0].position, now=0.0)  # greeted at t=0

    c._tick(30.0)  # within revisit_forget_s (60 s default) -> still skipped
    assert c._machine.state == "wander"

    c._tick(70.0)  # past the forget window -> re-engage the same person
    assert c._machine.state == "approach"


def test_greets_with_a_heart_when_the_subject_faces_the_camera():
    c = _container(
        vl_response='[{"bbox": [450, 200, 490, 240]}]',
        track_bbox=_SUBJECT_BBOX, facing="front")
    c.config.heart_duration_s = 0.0  # no real wait in the test
    c._tick(2.0)  # wander -> approach
    c._tick(2.1)  # approach -> greet (Hello)
    c._tick(2.8)  # greet -> cooldown: facing=front -> FingerHeart

    ids = _sport_ids(c._connection)
    heart = greeter_container._FINGER_HEART_API_ID
    assert heart in ids  # rewarded the facing subject
    # ... after the wave and before recovery: Hello -> FingerHeart -> RecoveryStand
    assert ids.index(1016) < ids.index(heart) < ids.index(1006)


def test_no_heart_when_the_subject_faces_away():
    c = _container(
        vl_response='[{"bbox": [450, 200, 490, 240]}]',
        track_bbox=_SUBJECT_BBOX, facing="back")
    c.config.heart_duration_s = 0.0
    c._tick(2.0)
    c._tick(2.1)
    c._tick(2.8)  # facing away -> no heart, just move on
    assert greeter_container._FINGER_HEART_API_ID not in _sport_ids(c._connection)


def test_facing_failure_still_recovers_and_marks_visited():
    # If the post-wave VLM facing call blows up, the dog must still recover and
    # mark the subject visited -- the cooldown effects run once on the edge, so a
    # raised exception would otherwise freeze it after Hello and let it re-greet
    # the same subject.
    c = _container(
        vl_response='[{"bbox": [450, 200, 490, 240]}]', track_bbox=_SUBJECT_BBOX)
    c.config.heart_duration_s = 0.0

    def query(_image, prompt):
        if "facing" in prompt.lower():
            raise RuntimeError("VLM timeout")
        return '[{"bbox": [450, 200, 490, 240]}]'

    c._vl_model.query = query

    c._tick(2.0)  # wander -> approach
    c._tick(2.1)  # approach -> greet
    c._tick(2.8)  # greet -> cooldown: facing query raises, must not abort cleanup

    assert c._machine.state == "cooldown"
    assert 1006 in _sport_ids(c._connection)  # RecoveryStand still sent
    assert c._registry.is_visited((1.5, 0.0), now=2.8)  # subject still marked


def test_approach_drives_by_ground_distance_not_a_width_model():
    c = _container()
    far = c._approach_twist(_SUBJECT_BBOX, 640, distance_m=3.0)
    assert far.linear.x > 0.0  # far -> close the distance
    too_close = c._approach_twist(_SUBJECT_BBOX, 640, distance_m=0.8)
    assert too_close.linear.x < 0.0  # too close -> back off to a safe gap
    blind = c._approach_twist(_SUBJECT_BBOX, 640, distance_m=None)
    assert blind.linear.x == 0.0  # no ground fix -> face only, do not lunge


def test_lost_subject_deadreckons_to_last_known_not_bail():
    # On track loss, the dog drives (odom) toward the subject's last-known floor
    # position instead of bailing -- it stays engaged while still en route.
    c = _container(
        vl_response='[{"bbox": [450, 200, 490, 240]}]', track_bbox=_SUBJECT_BBOX)
    c._tick(2.0)  # -> approach; last_position = (1.5, 0)
    assert c._machine.state == "approach"

    c._tracker = _FakeTracker(track_bbox=None)  # subject vanishes
    c._latest_odom = _FakeOdom(5.0, 0.0, yaw=math.pi)  # far, facing the target
    c.cmd_vel.msgs.clear()
    c._tick(2.5)

    assert c._machine.state == "approach"  # did not abandon on loss
    assert not c.cmd_vel.msgs[-1].is_zero()  # driving toward last-known, not frozen


def test_lost_subject_gives_up_after_arriving_and_not_re_detecting():
    c = _container(
        vl_response='[{"bbox": [450, 200, 490, 240]}]', track_bbox=_SUBJECT_BBOX)
    c._tick(2.0)  # -> approach; last_position = (1.5, 0)

    c._tracker = _FakeTracker(track_bbox=None)  # vanishes
    c._latest_odom = _FakeOdom(3.0, 0.0)  # within standoff of (1.5, 0): arrived
    c._vl_model.response = "[]"  # ... and it is no longer detectable there
    c._tick(4.0)  # past scan interval -> re-detect runs, finds nothing -> give up

    assert c._machine.state == "wander"
    assert c._last_snapshot.reason == "lost"


def test_lost_subject_re_acquires_at_last_known_and_greets():
    c = _container(
        vl_response='[{"bbox": [450, 200, 490, 240]}]', track_bbox=_SUBJECT_BBOX)
    c._tick(2.0)  # -> approach; last_position = (1.5, 0)

    c._tracker = _FakeTracker(track_bbox=None)  # vanishes mid-approach
    c._latest_odom = _FakeOdom(3.0, 0.0)  # arrived at the last-known spot
    inited_before = len(c._tracker.inited)
    c._tick(4.0)  # re-detect finds the chair there -> re-acquire + wave

    assert len(c._tracker.inited) > inited_before  # tracker re-seeded
    assert c._machine.state == "greet"  # re-acquired at standoff -> greet
    assert _sport_ids(c._connection)[-1] == 1016  # Hello fired


def test_stuck_engagement_returns_to_patrol_and_skips_subject():
    # A subject we can never reach (always beyond standoff) must not hang the
    # loop: the engage timeout bails to wander and skips that subject.
    far_bbox = (500.0, 200.0, 560.0, 240.0)  # bottom-center -> ~2.1 m away
    c = _container(
        vl_response='[{"bbox": [500, 200, 560, 240]}]', track_bbox=far_bbox)
    c._machine = GreeterMachine(GreeterParams(
        standoff_m=1.5, standoff_tolerance_m=0.3,
        engage_timeout_s=1.0, greet_duration_s=0.5, cooldown_duration_s=0.5,
    ))

    c._tick(2.0)  # wander -> approach (far subject selected)
    assert c._machine.state == "approach"
    c._tick(2.5)  # too far to reach standoff, still within the timeout
    assert c._machine.state == "approach"
    c._tick(3.1)  # engagement stalled past engage_timeout_s -> wander
    assert c._machine.state == "wander"
    assert c._registry.is_visited((2.1, 0.0))  # the unreachable subject is skipped
    assert c.cmd_vel.msgs[-1].is_zero()  # no stale drive command into wander


def test_start_and_stop_greeting_skill():
    c = _container(vl_response="[]")
    started = c.start_greeting()
    assert "start" in started.lower()
    stopped = c.stop_greeting()
    assert "stop" in stopped.lower()
    assert c.cmd_vel.msgs and c.cmd_vel.msgs[-1].is_zero()
    assert "stopped" in c.greeter_phase.msgs[-1].lower()  # stop phase published


def test_start_greeting_accepts_a_target_override():
    # The target is parameterizable so the same loop can greet "a chair" in sim
    # (no seated person there) and "a person sitting on a chair" on the robot.
    c = _container(vl_response="[]")
    c.start_greeting(target="a chair")
    assert c._description == "a chair"  # overrides the configured default
    c.stop_greeting()
    c.start_greeting()  # omitted -> back to the configured subject
    assert c._description == c.config.description
    c.stop_greeting()


def test_scan_queries_the_active_target():
    c = _container(vl_response="[]")
    c._description = "a chair"
    c._scan(_FakeImage())
    assert "a chair" in c._vl_model.last_prompt  # VLM asked for the active target


def test_module_stop_halts_the_robot():
    c = _container()
    c._patrolling_module_spec.patrolling = True
    # stop() halts before tearing down; the full DimOS teardown that follows
    # needs more wiring than the fakes provide, so suppress just that part.
    with contextlib.suppress(Exception):
        c.stop()  # DimOS shutdown path, not the stop_greeting skill
    assert c.cmd_vel.msgs[-1].is_zero()
    assert not c._patrolling_module_spec.is_patrolling()


def test_halt_stops_patrol_even_if_velocity_publish_fails():
    c = _container()
    c._patrolling_module_spec.patrolling = True

    class _Raising:
        def publish(self, _msg):
            raise RuntimeError("publish boom")

    c.cmd_vel = _Raising()
    c._halt()  # must not raise, and must still stop the patrol
    assert not c._patrolling_module_spec.is_patrolling()


def test_reset_engagement_clears_transient_state_but_keeps_registry():
    c = _container()
    c._machine.step(  # drive into a mid-engagement phase
        GreeterObservation(target_acquired=True, subject_visible=True), 0.0)
    c._chosen = Candidate(position=(1.0, 0.0), bbox=(0.0, 0.0, 10.0, 10.0))
    c._last_position = (1.0, 0.0)
    c._registry.mark_visited((9.0, 9.0))  # greeted in an earlier run

    c._reset_engagement()

    assert c._machine.state == "wander"
    assert c._chosen is None and c._last_position is None
    assert c._last_snapshot is None
    assert c._registry.is_visited((9.0, 9.0))  # kept across restarts


def test_status_trace_and_world_pose_explain_the_engagement():
    far_bbox = (500.0, 200.0, 560.0, 240.0)  # ~2.1 m -> stays approaching
    c = _container(
        vl_response='[{"bbox": [500, 200, 560, 240]}]', track_bbox=far_bbox)
    c._tick(2.0)  # wander -> approach (select a subject ~2.1 m away)
    c._tick(2.1)  # approach: track + locate (still too far to greet)

    trace = json.loads(c.greeter_status())
    assert trace["state"] == "approach"
    assert trace["subject_visible"] is True
    assert trace["distance_m"] is not None
    for key in ("patrolling", "subject_xy", "greeted", "reason"):
        assert key in trace
    assert "is_front" not in trace  # facing is gone from the trace
    assert c.subject_world_pose.msgs  # 3D marker published while engaging


def test_wave_hello_halts_then_waves_then_recovers():
    c = _container()
    c.config.greet_duration_s = 0.0  # no waits in the test
    c.config.recover_settle_s = 0.0
    c._patrolling_module_spec.patrolling = True  # dog was wandering when asked

    result = c.wave_hello()

    assert "wave" in result.lower()
    assert not c._patrolling_module_spec.is_patrolling()  # patrol stopped to wave
    assert c.cmd_vel.msgs[-1].is_zero()  # stood still to wave
    # Hello, then recover: RecoveryStand -> BalanceStand so it can move again.
    assert _sport_ids(c._connection) == [1016, 1006, 1002]


def test_wave_hello_stops_a_running_loop_before_waving():
    # If the autonomous loop is running, wave_hello must stop it (and join the
    # tick thread) first, so no tick restarts the patrol or drives the dog mid-wave.
    c = _container(vl_response="[]")
    c.config.greet_duration_s = 0.0
    c.config.recover_settle_s = 0.0
    c.start_greeting()
    assert c._thread is not None and c._thread.is_alive()

    c.wave_hello()

    assert c._thread is None  # the loop was stopped before the gesture
    assert _sport_ids(c._connection) == [1016, 1006, 1002]


def test_scan_clamps_off_frame_boxes_before_locating():
    # A VLM box that spills off the right edge is valid (it overlaps the frame)
    # but must be clamped before raycasting, or its off-image bottom-center
    # locates to the wrong floor point.
    c = _container(vl_response='[{"bbox": [600, 200, 700, 240]}]')

    candidates = c._scan(_FakeImage())

    assert len(candidates) == 1
    assert candidates[0].bbox[2] == 640.0  # clamped to the frame width
    # Clamped bottom-center (620, 240) -> floor x = (620-320)/100 = 3.0.
    assert candidates[0].position == pytest.approx((3.0, 0.0))


def test_scan_drops_a_candidate_on_a_mapped_wall():
    # The reflection case: the VLM box raycasts onto the floor at (1.5, 0.0),
    # but the costmap marks that spot a wall (a person seen reflected in glass).
    c = _container(vl_response='[{"bbox": [450, 200, 490, 240]}]')
    assert len(c._scan(_FakeImage())) == 1  # no costmap yet -> kept

    res = 0.1
    wall = [[0] * 11 for _ in range(11)]
    wall[5][5] = 100  # lethal cell at the candidate's floor position
    c._latest_costmap = _FakeCostmap(wall, res, (1.5 - 5 * res, 0.0 - 5 * res))
    assert c._scan(_FakeImage()) == []  # dropped: on a mapped wall

    free = [[0] * 11 for _ in range(11)]
    c._latest_costmap = _FakeCostmap(free, res, (1.5 - 5 * res, 0.0 - 5 * res))
    assert len(c._scan(_FakeImage())) == 1  # free space at the same spot -> kept


def test_obstacle_filter_can_be_disabled():
    c = _container(vl_response='[{"bbox": [450, 200, 490, 240]}]')
    wall = [[100] * 11 for _ in range(11)]
    c._latest_costmap = _FakeCostmap(wall, 0.1, (1.0, -0.5))
    c.config.reject_on_mapped_obstacle = False
    assert len(c._scan(_FakeImage())) == 1  # filter off -> kept despite the wall


def test_scan_drops_a_candidate_beyond_the_distance_bound():
    # A box near the image horizon ranges to a far floor point; such a phantom
    # is dropped before the dog commits to it and walks at empty space.
    c = _container(vl_response='[{"bbox": [450, 200, 490, 240]}]')
    c._locate = lambda box, image: ((50.0, 0.0), 50.0)  # 50 m ranging artifact
    assert c._scan(_FakeImage()) == []  # 50 m > the 15 m bound -> dropped
    c.config.max_subject_distance_m = 100.0
    assert len(c._scan(_FakeImage())) == 1  # within a looser bound -> kept


def test_roams_forward_when_odom_stalls_in_wander():
    # Patrol is goal-less and the dog is not moving: detect the stall from odom
    # and take over with a forward roam into new space.
    c = _container(vl_response="[]")
    c._latest_odom = _FakeOdom(0.0, 0.0)
    c.config.stall_timeout_s = 1.0
    c.config.roam_turn_s = 0.0  # skip the reorient phase -> straight to forward
    c._patrolling_module_spec.patrolling = True
    c._wander_tick(c._latest_image, 0.0)  # establish the anchor (not yet stalled)
    assert not c._roaming

    c.cmd_vel.msgs.clear()
    c._wander_tick(c._latest_image, 2.0)  # no movement for 2 s > 1 s -> roam

    assert c._roaming
    last = c.cmd_vel.msgs[-1]
    assert last.linear.x > 0.0  # driving forward into new space
    assert not c._patrolling_module_spec.is_patrolling()  # took over the patrol


def test_roam_reorients_in_place_before_driving_forward():
    c = _container(vl_response="[]")
    c._latest_odom = _FakeOdom(0.0, 0.0)
    c.config.stall_timeout_s = 1.0
    c.config.roam_turn_s = 2.0
    c._wander_tick(c._latest_image, 0.0)
    c.cmd_vel.msgs.clear()
    c._wander_tick(c._latest_image, 2.0)  # enters roam; within the turn phase
    turn = c.cmd_vel.msgs[-1]
    assert turn.angular.z != 0.0 and turn.linear.x == 0.0  # reorient first


def test_roam_hands_back_to_patrol_after_escaping():
    c = _container(vl_response="[]")
    c._latest_odom = _FakeOdom(0.0, 0.0)
    c.config.stall_timeout_s = 1.0
    c.config.roam_turn_s = 0.0
    c.config.roam_escape_distance_m = 0.5
    c._wander_tick(c._latest_image, 0.0)
    c._wander_tick(c._latest_image, 2.0)  # stalled -> roaming
    assert c._roaming

    c._latest_odom = _FakeOdom(1.0, 0.0)  # escaped 1 m > 0.5 m
    c._wander_tick(c._latest_image, 2.5)

    assert not c._roaming
    assert c._patrolling_module_spec.is_patrolling()  # patrol resumed


def test_no_roam_while_making_progress():
    c = _container(vl_response="[]")
    c._latest_odom = _FakeOdom(0.0, 0.0)
    c.config.stall_timeout_s = 1.0
    c._wander_tick(c._latest_image, 0.0)
    c._latest_odom = _FakeOdom(0.0, 0.5)  # moved 0.5 m (> 0.3 threshold)
    c._wander_tick(c._latest_image, 2.0)
    assert not c._roaming  # movement resets the stall timer


def test_no_roam_without_odom():
    c = _container(vl_response="[]")
    c._latest_odom = None  # no odom -> never drive blind
    c.config.stall_timeout_s = 1.0
    c._wander_tick(c._latest_image, 0.0)
    c.cmd_vel.msgs.clear()
    c._wander_tick(c._latest_image, 5.0)
    assert not c._roaming
    assert c._patrolling_module_spec.is_patrolling()  # patrol owns motion


def test_wander_rescans_when_every_detection_is_filtered_out():
    # The freeze case: the dog faces a wall/reflection -- it detects a "person"
    # but the costmap filter drops it, leaving no candidate. It must rotate to a
    # new heading rather than sit frozen (the patrol is usually goal-less here).
    c = _container(vl_response='[{"bbox": [450, 200, 490, 240]}]')
    wall = [[100] * 11 for _ in range(11)]  # candidate raycasts to (1.5, 0.0)
    c._latest_costmap = _FakeCostmap(wall, 0.1, (1.0, -0.5))
    c._patrolling_module_spec.patrolling = True
    c.cmd_vel.msgs.clear()

    c._tick(2.0)  # past the first scan interval

    assert c._last_scan_dropped > 0  # the detection was filtered out
    assert c._rescanning  # rotating to look elsewhere, not frozen
    last = c.cmd_vel.msgs[-1]
    assert last.angular.z != 0.0 and last.linear.x == 0.0  # in-place yaw
    assert not c._patrolling_module_spec.is_patrolling()  # took over the patrol


def test_halt_zeros_velocity_and_stops_patrol():
    c = _container()
    c._patrolling_module_spec.patrolling = True
    c._halt()
    assert c.cmd_vel.msgs[-1].is_zero()
    assert not c._patrolling_module_spec.is_patrolling()


def test_loop_fails_safe_and_stops_after_repeated_errors():
    c = _container(vl_response="[]")
    c._patrolling_module_spec.patrolling = True
    c.config.max_consecutive_errors = 2
    c.config.loop_hz = 50.0

    def boom(_now):
        raise RuntimeError("tick boom")

    c._tick = boom  # every tick fails
    c.start_greeting()
    c._thread.join(timeout=2.0)

    assert not c._thread.is_alive()  # gave up rather than spinning
    assert c.cmd_vel.msgs[-1].is_zero()  # halted
    assert not c._patrolling_module_spec.is_patrolling()  # patrol stopped
    assert c._last_snapshot is not None and c._last_snapshot.reason == "error"


def test_stop_greeting_also_stops_the_patrol():
    # In wander the patrol owns motion, so stopping the loop alone leaves the
    # dog driving -- stop_greeting must stop the patrol too.
    c = _container(vl_response="[]")
    c._patrolling_module_spec.patrolling = True  # robot mid-patrol
    c.stop_greeting()
    assert not c._patrolling_module_spec.is_patrolling()
    assert "stop" in c._patrolling_module_spec.calls
    assert c.cmd_vel.msgs[-1].is_zero()


class _FakeCenter:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class _FakeDetection3D:
    def __init__(self, center):
        self.center = center


class _FakePointCloud:
    frame_id = "world"

    def __init__(self, n_points=10):
        self._points = np.zeros((n_points, 3))

    def as_numpy(self):
        return self._points, None


def test_locate_prefers_lidar_pointcloud_over_raycast(monkeypatch):
    # With a non-empty lidar cloud present, _locate ranges from the projected
    # points (here faked to centre on (1.5, 0)) not the flat-floor raycast.
    c = _container()
    c._latest_pointcloud = _FakePointCloud(10)  # from_2d itself is faked below
    monkeypatch.setattr(
        greeter_container.Detection3DPC, "from_2d",
        lambda *a, **k: _FakeDetection3D(_FakeCenter(1.5, 0.0)),
    )
    # _FakeTf puts the camera at world x=y=0, so the distance is just |centre|.
    assert c._locate(_SUBJECT_BBOX, _FakeImage()) == ((1.5, 0.0), 1.5)


def test_locate_falls_back_to_raycast_without_lidar():
    c = _container()
    assert c._latest_pointcloud is None
    assert c._locate(_SUBJECT_BBOX, _FakeImage()) == c._locate_raycast(
        _SUBJECT_BBOX, _FakeImage())


def test_locate_falls_back_to_raycast_when_lidar_finds_no_points(monkeypatch):
    c = _container()
    c._latest_pointcloud = _FakePointCloud(10)
    monkeypatch.setattr(
        greeter_container.Detection3DPC, "from_2d", lambda *a, **k: None)
    assert c._locate(_SUBJECT_BBOX, _FakeImage()) == c._locate_raycast(
        _SUBJECT_BBOX, _FakeImage())


def test_locate_falls_back_to_raycast_when_lidar_cloud_is_empty():
    c = _container()
    c._latest_pointcloud = _FakePointCloud(0)  # cloud arrives but has no points
    assert c._locate(_SUBJECT_BBOX, _FakeImage()) == c._locate_raycast(
        _SUBJECT_BBOX, _FakeImage())


class _FlakyConnection:
    def __init__(self, fail_times, error=None):
        self.fail_times = fail_times
        self.error = error or Exception("Data channel is not open")
        self.calls = 0

    def publish_request(self, _topic, _data):
        self.calls += 1
        if self.calls <= self.fail_times:
            raise self.error
        return {}


def test_send_sport_retries_a_transient_closed_channel_then_succeeds(monkeypatch):
    monkeypatch.setattr(greeter_container.time, "sleep", lambda _s: None)
    c = _container()
    c._connection = _FlakyConnection(fail_times=2)  # opens on the 3rd attempt
    assert c._send_sport(greeter_container._HELLO_API_ID) is True
    assert c._connection.calls == 3


def test_send_sport_gives_up_without_crashing_when_channel_stays_closed(
    monkeypatch,
):
    monkeypatch.setattr(greeter_container.time, "sleep", lambda _s: None)
    c = _container()
    c._connection = _FlakyConnection(fail_times=99)  # channel never reopens
    assert c._send_sport(greeter_container._HELLO_API_ID) is False
    assert c._connection.calls == c.config.sport_retry_attempts


def test_send_sport_reraises_unexpected_errors(monkeypatch):
    # A non-channel error is a real bug -> surface it to the loop fail-safe.
    monkeypatch.setattr(greeter_container.time, "sleep", lambda _s: None)
    c = _container()
    c._connection = _FlakyConnection(fail_times=99, error=ValueError("boom"))
    with pytest.raises(ValueError):
        c._send_sport(greeter_container._HELLO_API_ID)


def test_send_sport_without_a_connection_is_a_no_op():
    c = _container()
    c._connection = None
    assert c._send_sport(greeter_container._HELLO_API_ID) is False


def test_search_twist_yaws_toward_last_seen_side_not_frozen():
    c = _container()
    c._last_error_x = 0.8  # subject was last seen to the right
    right = c._search_twist()
    c._last_error_x = -0.8  # ... to the left
    left = c._search_twist()
    assert right.linear.x == 0.0 and right.linear.y == 0.0  # never drives blind
    assert right.angular.z != 0.0  # yaws to re-find rather than freezing
    assert right.angular.z == -left.angular.z  # opposite side -> opposite yaw
    c._last_error_x = 0.0
    assert c._search_twist().angular.z == 0.0  # nothing to re-centre toward


def test_engage_tick_searches_instead_of_freezing_when_track_lost():
    # Tracker returns nothing (track_bbox=None) mid-approach: the dog should yaw
    # to re-find the subject, not publish a dead stop.
    c = _container(track_bbox=None)
    c._last_error_x = 0.8
    obs = c._engage_tick(_FakeImage())
    assert obs.subject_visible is False
    assert not c.cmd_vel.msgs[-1].is_zero()  # searching, not frozen
