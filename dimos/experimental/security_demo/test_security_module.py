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

from __future__ import annotations

import threading

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D


@pytest.mark.self_hosted
def test_find_best_person_detects_person(security_module, yolo_detector, person_image):
    security_module._detector = yolo_detector

    result = security_module._find_best_person(person_image)

    assert result is not None
    assert result.name == "person"
    assert result.bbox_2d_volume() > 0


@pytest.mark.self_hosted
def test_find_best_person_returns_none_for_empty_scene(security_module, yolo_detector, empty_image):
    security_module._detector = yolo_detector

    result = security_module._find_best_person(empty_image)

    assert result is None


def test_patrol_step_transitions_to_following_on_detection(
    security_module, person_image, make_detection, mocker
):
    module = security_module
    module._state = "PATROLLING"
    module._has_active_goal = True
    module._latest_image = person_image
    module._latest_pose = PoseStamped(
        pose=Pose(
            position=Point(x=1, y=2, z=0.0), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )

    det = make_detection()
    module._detector.process_image.return_value = ImageDetections2D(
        image=person_image, detections=[det]
    )
    # patch to avoid cv2 dep issues
    mocker.patch.object(type(det), "draw_on", autospec=True)

    module._patrol_step()

    assert module._state == "FOLLOWING"
    last_state = module.security_state.publish.call_args[0][0]
    assert last_state.data == "FOLLOWING"
    module._speak_skill.speak.assert_called_with("Intruder detected", blocking=False)
    module.goal_request.publish.assert_called()  # goal cancellation
    module.detection.publish.assert_called()
    assert module._has_active_goal is False
    assert module._tracker is not None
    module._tracker.init_track.assert_called_once()


def test_patrol_step_requests_goal_when_no_active_goal(security_module):
    module = security_module
    module._state = "PATROLLING"
    module._has_active_goal = False
    module._latest_image = None  # causes early return after goal logic

    goal = PoseStamped(
        pose=Pose(
            position=Point(x=5, y=5, z=0.0), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    module._router.next_goal.return_value = goal

    module._patrol_step()

    module.goal_request.publish.assert_called_once_with(goal)
    assert module._has_active_goal is True


def test_follow_step_publishes_twist_when_tracking(
    security_module, person_image, make_detection, mocker
):
    module = security_module
    module._state = "FOLLOWING"
    module._latest_image = person_image

    bbox = (100.0, 50.0, 300.0, 400.0)
    det = make_detection(bbox=bbox)
    module._tracker.process_image.return_value = ImageDetections2D(
        image=person_image, detections=[det]
    )

    twist = Twist(linear=Vector3(x=0.3, y=0.0, z=0.0), angular=Vector3(z=0.1, x=0.0, y=0.0))
    module._visual_servo.compute_twist.return_value = twist

    mocker.patch("dimos.experimental.security_demo.security_module.time.sleep")

    module._follow_step()

    module._visual_servo.compute_twist.assert_called_once_with(bbox, person_image.width)
    module.cmd_vel.publish.assert_called_once_with(twist)
    assert module._state == "FOLLOWING"


def test_follow_step_transitions_to_patrolling_on_person_lost(security_module, person_image):
    module = security_module
    module._state = "FOLLOWING"
    module._latest_image = person_image

    module._tracker.process_image.return_value = ImageDetections2D(
        image=person_image, detections=[]
    )

    module._follow_step()

    # Zero twist should be published to stop the robot
    published_twist = module.cmd_vel.publish.call_args[0][0]
    assert published_twist == Twist(
        linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
    )

    module._speak_skill.speak.assert_called_with(
        "Lost sight of intruder, resuming patrol", blocking=False
    )
    module._router.reset.assert_called_once()
    assert module._state == "PATROLLING"
    assert module._has_active_goal is False


def test_main_loop_stops_cleanly(security_module):
    module = security_module

    call_count = 0

    def patrol_side_effect():
        nonlocal call_count
        call_count += 1
        if call_count >= 1:
            module._stop_event.set()

    module._patrol_step = patrol_side_effect

    thread = threading.Thread(target=module._main_loop, daemon=True)
    thread.start()
    thread.join(timeout=5.0)

    assert not thread.is_alive(), "main_loop thread did not stop"
    assert module._state == "IDLE"

    # Verify zero twist published on shutdown
    last_twist = module.cmd_vel.publish.call_args[0][0]
    assert last_twist == Twist(
        linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
    )

    # Verify state transitions: PATROLLING then IDLE
    state_values = [call.args[0].data for call in module.security_state.publish.call_args_list]
    assert "PATROLLING" in state_values
    assert "IDLE" in state_values
