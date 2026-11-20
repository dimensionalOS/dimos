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

import json
import math

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.calibration.hand_eye import HandEyeSample, MarkerObservationError
from dimos.calibration.hand_eye_run import run_hand_eye
from dimos.calibration.hand_eye_session import load_session
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

HOME = (0.0, 1.0, -1.0, 1.1, 0.0, 0.0)
A1Z_RETURN = (0.0, 0.05, -0.05, 0.0, 0.0, 0.0)


@pytest.fixture
def tip_from_mount():
    return Transform(frame_id="tip", child_frame_id="mount", translation=Vector3(-0.075, 0, 0))


@pytest.fixture
def home_tool_pose():
    return PoseStamped(
        frame_id="world",
        position=(0.45, -0.15, 0.35),
        orientation=Rotation.from_euler("xyz", [0.3, 0.55, -0.4]).as_quat(),
    )


@pytest.fixture
def automatic_run(tmp_path, marker_image, observations, mocker, home_tool_pose, tip_from_mount):
    """Simulate a planner moving its TCP with a camera mounted on a different link."""
    template, info, _ = marker_image
    _, camera_to_mount, base_from_target = observations
    commands = []
    images = []

    def plan_joint_move(joints):
        commands.append(tuple(joints))
        return home_tool_pose

    def capture():
        tcp = commands[-1] if isinstance(commands[-1], PoseStamped) else home_tool_pose
        image = template.copy()
        image.ts += len(images) + 1
        image.data[0, 0] = len(images)
        images.append(image)
        measured = Transform.from_matrix(
            Transform.from_pose("tip", tcp).to_matrix() @ tip_from_mount.to_matrix(),
            ts=image.ts,
            frame_id="base",
            child_frame_id="mount",
        )
        return image, measured

    def detect(image, info, measured, marker_size, marker_id):
        return HandEyeSample(
            measured, np.linalg.inv(measured @ camera_to_mount) @ base_from_target, 0.1
        )

    mocker.patch("dimos.calibration.hand_eye_session.detect_sample", side_effect=detect)

    def run(
        review,
        *,
        directory=tmp_path,
        home=HOME,
        joint_count=6,
        move_joints=plan_joint_move,
        **settings,
    ):
        return run_hand_eye(
            directory,
            info,
            home=home,
            joint_names=[f"joint{i}" for i in range(joint_count)],
            base_frame="base",
            ee_frame="mount",
            tip_frame="tip",
            move_joints=move_joints,
            move_pose=commands.append,
            capture=capture,
            review=review,
            **settings,
        )

    return run, commands, images


@pytest.mark.parametrize(
    ("home", "training", "validation"),
    [(HOME, 8, 4), ((*HOME, 0.25), 9, 5)],
)
def test_moves_two_local_tool_rings_and_validates_reviewed_images(
    tmp_path, automatic_run, observations, mocker, home_tool_pose, home, training, validation
):
    run, commands, _ = automatic_run
    review = mocker.Mock(return_value="accept")
    _, camera_to_mount, _ = observations

    result = run(
        review,
        home=home,
        joint_count=len(home),
        count=training,
        validation_count=validation,
    )

    assert result["validated"]
    assert result["parent_frame"] == "mount"
    np.testing.assert_allclose(result["camera_to_ee"], camera_to_mount, atol=1e-8)
    assert json.loads((tmp_path / "hand_eye.json").read_text()) == result
    assert commands[0] == home
    assert commands[-1] == (0.0,) * len(home)
    rotation_at_home = Rotation.from_quat(home_tool_pose.orientation.to_list())
    for role, ring in (("train", commands[1:training]), ("validation", commands[training:-1])):
        radius = math.radians(11 if role == "train" else 7)
        count = training - 1 if role == "train" else validation
        assert len(ring) == count
        for index, target in enumerate(ring):
            assert target.frame_id == "world"
            np.testing.assert_array_equal(
                target.position.to_list(), home_tool_pose.position.to_list()
            )
            local = rotation_at_home.inv() * Rotation.from_quat(target.orientation.to_list())
            phase = math.tau * (index + (0.5 if role == "validation" else 0)) / count
            np.testing.assert_allclose(
                local.as_rotvec(),
                radius * np.array([math.cos(phase), math.sin(phase), 0]),
                atol=1e-12,
            )
    session = load_session(tmp_path)
    samples = session["samples"]
    assert [s["role"] for s in samples] == ["train"] * training + ["validation"] * validation
    for sample, call in zip(samples, review.call_args_list, strict=True):
        reviewed, _ = call.args
        assert sample["image_timestamp"] == reviewed.ts
        np.testing.assert_array_equal(
            cv2.imread(str(tmp_path / sample["image"]), cv2.IMREAD_UNCHANGED), reviewed.to_opencv()
        )


def test_retake_quit_and_resume_reuses_saved_home_and_approved_views(
    tmp_path, automatic_run, home_tool_pose, mocker
):
    run, commands, images = automatic_run
    choices = mocker.Mock(side_effect=["retry", "accept", "quit"])

    assert run(choices) is None

    home, next_goal = commands
    assert home == HOME
    approved = load_session(tmp_path)["samples"]
    assert [s["image_timestamp"] for s in approved] == [images[1].ts]
    assert [p.name for p in tmp_path.glob("*.png")] == [approved[0]["image"]]
    with pytest.raises(ValueError, match="settings changed"):
        run(choices, count=9)
    assert len(commands) == 2

    # Changing the reading the planner would return cannot shift the saved rings.
    home_tool_pose.position.x += 0.0003
    original_x = load_session(tmp_path)["home_tool_pose"]["position"][0]
    resumed_review = mocker.Mock(return_value="accept")
    result = run(resumed_review, return_joints=A1Z_RETURN)

    assert commands[2].position.x == original_x
    np.testing.assert_array_equal(
        commands[2].orientation.to_list(), next_goal.orientation.to_list()
    )
    assert result["validated"]
    assert resumed_review.call_count == 11
    assert commands[-1] == A1Z_RETURN
    samples = load_session(tmp_path)["samples"]
    assert samples[:1] == approved
    assert [s["image_timestamp"] for s in samples] == [
        images[1].ts,
        *(image.ts for image in images[3:]),
    ]
    commands.clear()
    assert run(resumed_review)["validated"]
    assert commands == []


@pytest.mark.parametrize(
    ("axis_name", "tool_axis", "camera_pitch_deg", "distance"),
    [("x", [1, 0, 0], 90, 0.23), ("-z", [0, 0, -1], 180, 0.33)],
)
def test_resumed_orbit_moves_around_the_home_target_and_keeps_it_centered(
    automatic_run,
    observations,
    home_tool_pose,
    mocker,
    axis_name,
    tool_axis,
    camera_pitch_deg,
    distance,
):
    run, commands, _ = automatic_run
    _, camera_to_mount, base_from_target = observations
    # Mount the camera at the TCP and center the target at a known distance.
    camera_to_mount[:3, 3] = [0.075, 0, 0]
    camera_to_mount[:3, :3] = Rotation.from_euler("y", camera_pitch_deg, degrees=True).as_matrix()
    home_rotation = Rotation.from_quat(home_tool_pose.orientation.to_list())
    home_position = np.array(home_tool_pose.position.to_list())
    home_ray = home_rotation.apply(tool_axis)
    target = home_position + distance * home_ray
    base_from_target[:3, 3] = target

    assert run(mocker.Mock(side_effect=["accept", "quit"]), orbit_axis=axis_name) is None
    assert len(commands) == 2
    first_goal = commands[-1]
    with pytest.raises(ValueError, match="settings changed"):
        run(None)
    assert len(commands) == 2

    assert run(None, orbit_axis=axis_name)["validated"]
    resumed_goal = commands[2]
    np.testing.assert_array_equal(resumed_goal.position.to_list(), first_goal.position.to_list())
    np.testing.assert_array_equal(
        resumed_goal.orientation.to_list(), first_goal.orientation.to_list()
    )
    training, validation = commands[2:9], commands[9:-1]
    assert (len(training), len(validation)) == (7, 4)
    first_directions = []
    for ring, max_angle, max_radius in ((training, 11, 0.055), (validation, 8, 0.040)):
        radius = min(distance * math.sin(math.radians(max_angle)), max_radius)
        directions = []
        for goal in ring:
            position = np.array(goal.position.to_list())
            viewing_ray = Rotation.from_quat(goal.orientation.to_list()).apply(tool_axis)
            np.testing.assert_allclose(position + distance * viewing_ray, target, atol=1e-12)
            offset = position - home_position
            lateral = offset - np.dot(offset, home_ray) * home_ray
            assert np.linalg.norm(lateral) == pytest.approx(radius)
            directions.append(lateral / radius)
        first_directions.append(directions[0])
        for i, direction in enumerate(directions):
            assert direction @ directions[0] == pytest.approx(math.cos(math.tau * i / len(ring)))
    assert first_directions[0] @ first_directions[1] == pytest.approx(math.cos(math.pi / 4))


@pytest.fixture
def orbit_mount(home_tool_pose, observations, marker_image, tip_from_mount):
    """An offset wrist camera looking obliquely at a tag, with no world-aligned tag axes."""
    _, mount_from_camera, world_from_tag = observations
    tip_from_camera = tip_from_mount.to_matrix() @ mount_from_camera
    world_from_camera = Transform.from_pose("tip", home_tool_pose).to_matrix() @ tip_from_camera
    camera_from_tag = np.eye(4)
    camera_from_tag[:3, :3] = Rotation.from_euler("xyz", [155, 8, 40], degrees=True).as_matrix()
    camera_from_tag[:3, 3] = [0.004, -0.003, 0.29]
    world_from_tag[:] = world_from_camera @ camera_from_tag
    return Transform.from_matrix(
        tip_from_camera, frame_id="tip", child_frame_id=marker_image[0].frame_id
    )


def _home_relative_offsets(views, home, world_from_tag):
    """Measure actual optical centers from the captured views on their respective tool poses."""
    camera_from_tag = np.array([view["camera_from_target"] for view in views])
    np.testing.assert_allclose(camera_from_tag[:, :2, 3], 0, atol=1e-8)
    assert np.all(camera_from_tag[:, 2, 3] > 0)
    world_from_camera = world_from_tag @ np.linalg.inv(camera_from_tag)
    world_from_home = world_from_tag @ np.linalg.inv(home["camera_from_target"])
    return world_from_camera[:, :3, 3] - world_from_home[:3, 3]


def _check_sweep_coverage(training, validation):
    lateral = training[:, 1]
    assert abs(lateral[0]) > 0.12
    assert lateral[-1] == pytest.approx(-lateral[0])
    # A center training pose is required even when the requested sweep count is even.
    (center,) = np.flatnonzero(np.isclose(lateral, 0, atol=1e-8))
    assert abs(center - (len(lateral) - 1 - center)) <= 1
    regular_lateral = lateral.copy()
    if len(training) == 7:
        # The first left-side view is halfway toward home from its mirrored right-side view.
        np.testing.assert_allclose(training[4], training[2] * [0.5, -0.5, 0.5], atol=1e-8)
        regular_lateral[4] *= 2
    for side in (regular_lateral[: center + 1], regular_lateral[center:]):
        np.testing.assert_allclose(side, np.linspace(side[0], side[-1], len(side)), atol=1e-8)
    lateral = validation[:, 1]
    assert abs(lateral[0]) > 0.05
    expected = np.linspace(lateral[0], -lateral[0], len(lateral))
    if len(validation) == 4:
        expected[2] *= 0.5
        # View 3 mirrors view 2 across home, at half its displacement in all axes.
        np.testing.assert_allclose(validation[2], validation[1] * [0.5, -0.5, 0.5], atol=1e-8)
    np.testing.assert_allclose(lateral, expected, atol=1e-8)
    assert training[0, 1] * validation[0, 1] < 0  # Validation sweeps back.
    # Include forward/back and above/below home at the minimum capture count.
    all_offsets = np.concatenate((training, validation))
    for axis in (0, 2):
        assert all_offsets[:, axis].min() < -0.01
        assert all_offsets[:, axis].max() > 0.01
    distances = np.linalg.norm(training[:, None] - validation, axis=2)
    assert distances.min() > 1e-3


def test_target_centered_sweep_resumes_and_observes_tag_from_both_sides(
    tmp_path, automatic_run, observations, orbit_mount, mocker
):
    run, commands, _ = automatic_run
    _, mount_from_camera, world_from_tag = observations
    assert run(mocker.Mock(side_effect=["accept", "quit"]), orbit_camera=orbit_mount) is None
    first_goal = commands[-1]
    changed_mount = Transform.from_matrix(
        orbit_mount.to_matrix(), frame_id="tip", child_frame_id=orbit_mount.child_frame_id
    )
    changed_mount.translation.y += 0.02
    with pytest.raises(ValueError, match="mount changed"):
        run(None, orbit_camera=changed_mount)
    assert len(commands) == 2

    result = run(None, orbit_camera=orbit_mount)
    assert result["validated"]
    np.testing.assert_allclose(result["camera_to_ee"], mount_from_camera, atol=1e-8)
    np.testing.assert_allclose(
        Transform.from_pose("tip", commands[2]).to_matrix(),
        Transform.from_pose("tip", first_goal).to_matrix(),
    )

    samples = load_session(tmp_path)["samples"]
    training = _home_relative_offsets(samples[1:8], samples[0], world_from_tag)
    validation = _home_relative_offsets(samples[8:], samples[0], world_from_tag)
    assert (len(training), len(validation)) == (7, 4)
    _check_sweep_coverage(training, validation)


def test_camera_seed_steers_views_but_solver_refits_mount(automatic_run, observations, orbit_mount):
    run, _, _ = automatic_run
    _, mount_from_camera, _ = observations
    orbit_mount.translation.z += 0.012

    result = run(None, orbit_camera=orbit_mount)

    assert result["validated"]
    np.testing.assert_allclose(result["camera_to_ee"], mount_from_camera, atol=1e-8)


def test_low_endpoint_behind_the_tag_stops_before_leaving_home(
    automatic_run, observations, orbit_mount, home_tool_pose
):
    run, commands, _ = automatic_run
    _, _, world_from_tag = observations
    camera_at_home = (
        Transform.from_pose("tip", home_tool_pose).to_matrix() @ orbit_mount.to_matrix()
    )
    world_from_tag[:3, :3] = np.eye(3)
    world_from_tag[:3, 3] = camera_at_home[:3, 3] + [0.20, 0, -0.08]

    with pytest.raises(ValueError, match="crosses the tag plane"):
        run(None, orbit_camera=orbit_mount)

    assert commands == [HOME]


def test_requested_halfway_move_replaces_only_the_rejected_validation_view(
    tmp_path, automatic_run, observations, orbit_mount, mocker
):
    run, commands, images = automatic_run
    _, mount_from_camera, world_from_tag = observations

    def detect(image, info, measured, marker_size, marker_id):
        if image.data.flat[0] == 10:  # Validation 3/4.
            raise MarkerObservationError("Tag not detected")
        return HandEyeSample(
            measured, np.linalg.solve(measured @ mount_from_camera, world_from_tag), 0.1
        )

    mocker.patch("dimos.calibration.hand_eye_session.detect_sample", side_effect=detect)
    review = mocker.Mock(side_effect=["accept"] * 11 + ["quit"])
    rejected = mocker.Mock(return_value="halfway")
    assert run(review, review_rejected=rejected, orbit_camera=orbit_mount) is None
    samples = load_session(tmp_path)["samples"]
    assert [s["role"] for s in samples] == ["train"] * 8 + ["validation"] * 3
    assert [s["image_timestamp"] for s in samples] == [image.ts for image in images[:10]] + [
        images[11].ts
    ]
    assert "Validation 3/4" in rejected.call_args.args[1]
    assert "Validation 3/4" in review.call_args_list[-2].args[1]
    assert "Validation 4/4" in review.call_args_list[-1].args[1]
    home = world_from_tag @ np.linalg.inv(samples[0]["camera_from_target"])
    full_goal = Transform.from_pose("tip", commands[10]).to_matrix() @ orbit_mount.to_matrix()
    retry = _home_relative_offsets(samples[-1:], samples[0], world_from_tag)[0]
    np.testing.assert_allclose(retry, 0.5 * (full_goal[:3, 3] - home[:3, 3]), atol=1e-8)


@pytest.mark.parametrize(
    ("space", "initial_views", "rise"),
    [
        ("tag_azimuth_deg_radius_height_m", 1, 0.0),
        ("tag_arc", 1, 0.04),
        ("tool_tilts_then_tag_arc", 5, 0.04),
    ],
)
def test_prior_tag_plane_sweep_resumes_its_saved_camera_goals(
    tmp_path, automatic_run, orbit_mount, tip_from_mount, mocker, space, initial_views, rise
):
    run, _, _ = automatic_run
    options = (
        {"orbit_camera": orbit_mount} if initial_views == 1 else {"tip_from_mount": tip_from_mount}
    )
    assert run(mocker.Mock(side_effect=["accept"] * initial_views + ["quit"]), **options) is None
    session = load_session(tmp_path)
    current = session["settings"]["automatic_views"]
    # These are the angle/radius/height values written by the previous releases.
    training = [
        [float(a), 0.17, float(0.18 + rise * (1.0 - (a / 50.0) ** 2))]
        for a in np.linspace(-50.0, 50.0, 8 - initial_views)
    ]
    validation = [[float(a), 0.09, 0.19] for a in np.linspace(60.0, -60.0, 4)]
    schedule = {k: current[k] for k in ("home_joints", "joint_names", "tip_frame")}
    schedule.update(
        space=space, train=current["train"][:initial_views] + training, validation=validation
    )
    if initial_views == 1:
        schedule["camera_frame"] = orbit_mount.child_frame_id
    else:
        schedule["initial_tilt_views"] = initial_views
    session["settings"]["automatic_views"] = schedule
    (tmp_path / "session.json").write_text(json.dumps(session))

    result = run(None, **options)

    assert result["validated"]
    samples = load_session(tmp_path)["samples"]
    assert samples[:initial_views] == session["samples"]
    centers = np.linalg.inv(np.array([s["camera_from_target"] for s in samples]))[:, :3, 3]
    home_azimuth = math.atan2(centers[0, 1], centers[0, 0])
    for measured, requested in ((centers[initial_views:8], training), (centers[8:], validation)):
        azimuth, radius, height = np.array(requested).T
        angles = np.radians(azimuth) + home_azimuth
        expected = np.column_stack((radius * np.cos(angles), radius * np.sin(angles), height))
        np.testing.assert_allclose(measured, expected, atol=1e-8)


@pytest.mark.usefixtures("orbit_mount")
@pytest.mark.parametrize("denser_count", [9, 12])  # Even and odd numbers of broad training views.
def test_fresh_run_aims_both_arcs_from_its_first_five_observations(
    tmp_path, automatic_run, observations, tip_from_mount, mocker, denser_count
):
    run, commands, _ = automatic_run
    _, mount_from_camera, world_from_tag = observations
    quit_at_first_wide_view = mocker.Mock(side_effect=["accept"] * 5 + ["quit"])
    assert run(quit_at_first_wide_view, tip_from_mount=tip_from_mount) is None
    pilot = load_session(tmp_path)["samples"]
    assert len(pilot) == 5
    assert not (tmp_path / "hand_eye.json").exists()
    first_goal = commands[-1]
    commands.clear()

    result = run(None, tip_from_mount=tip_from_mount)

    np.testing.assert_allclose(
        Transform.from_pose("tip", commands[0]).to_matrix(),
        Transform.from_pose("tip", first_goal).to_matrix(),
    )
    assert result["validated"]
    np.testing.assert_allclose(result["camera_to_ee"], mount_from_camera, atol=1e-8)
    samples = load_session(tmp_path)["samples"]
    assert samples[:5] == pilot
    assert result["training_sample_ids"] == [s["id"] for s in samples[:8]]
    assert result["validation_sample_ids"] == [s["id"] for s in samples[8:]]
    assert len(samples) == 12
    training = _home_relative_offsets(samples[5:8], samples[0], world_from_tag)
    validation = _home_relative_offsets(samples[8:], samples[0], world_from_tag)
    _check_sweep_coverage(training, validation)

    # Adding views fills in the same range on both sweeps.
    denser = tmp_path / "denser"
    result = run(
        None,
        directory=denser,
        tip_from_mount=tip_from_mount,
        count=denser_count,
        validation_count=5,
    )
    assert result["validated"]
    samples = load_session(denser)["samples"]
    more_training = _home_relative_offsets(samples[5:denser_count], samples[0], world_from_tag)
    more_validation = _home_relative_offsets(samples[denser_count:], samples[0], world_from_tag)
    assert (len(more_training), len(more_validation)) == (denser_count - 5, 5)
    _check_sweep_coverage(more_training, more_validation)
    np.testing.assert_allclose(more_training[[0, -1]], training[[0, -1]], atol=1e-8)
    np.testing.assert_allclose(more_validation[[0, -1]], validation[[0, -1]], atol=1e-8)


def test_unobservable_initial_tilts_stop_before_any_wide_move(
    tmp_path, automatic_run, home_tool_pose, tip_from_mount, mocker
):
    run, commands, _ = automatic_run
    # The reported robot and camera poses never rotate, even though tilt commands were issued.
    camera_from_target = np.eye(4)
    camera_from_target[2, 3] = 0.29
    mocker.patch(
        "dimos.calibration.hand_eye_session.detect_sample",
        return_value=HandEyeSample(np.eye(4), camera_from_target, 0.1),
    )

    with pytest.raises(ValueError, match="rotations"):
        run(None, tip_from_mount=tip_from_mount)

    assert len(commands) == len(load_session(tmp_path)["samples"]) == 5
    for tilt in commands[1:]:
        np.testing.assert_allclose(tilt.position.to_list(), home_tool_pose.position.to_list())
    assert not (tmp_path / "hand_eye.json").exists()


def test_orbit_needs_an_accepted_home_observation_before_its_first_view(automatic_run, mocker):
    run, commands, _ = automatic_run
    mocker.patch(
        "dimos.calibration.hand_eye_session.detect_sample",
        side_effect=MarkerObservationError("No tag in view"),
    )
    with pytest.raises(MarkerObservationError, match="home.*No tag"):
        run(None, orbit_axis="x")
    assert commands == [HOME]


@pytest.mark.parametrize(("depth", "error"), [(-0.24, "in front"), (0.6, "too far")])
def test_unsuitable_home_target_stops_before_any_orbit_move(automatic_run, mocker, depth, error):
    run, commands, _ = automatic_run
    tag = np.eye(4)
    tag[2, 3] = depth
    mocker.patch(
        "dimos.calibration.hand_eye_session.detect_sample",
        return_value=HandEyeSample(np.eye(4), tag, 0.1),
    )
    with pytest.raises(ValueError, match=error):
        run(None, orbit_axis="x")
    assert commands == [HOME]


def test_unreviewed_run_collects_and_validates_both_rings(tmp_path, automatic_run, mocker):
    run, commands, images = automatic_run
    park = mocker.Mock()

    # A home equal to the return target must still use the capture tolerance on the first move.
    result = run(None, home=A1Z_RETURN, return_joints=A1Z_RETURN, return_move=park)

    assert commands[0] == A1Z_RETURN
    assert len(commands) == len(images) == 12
    park.assert_called_once_with(A1Z_RETURN)
    assert result["validated"]
    assert json.loads((tmp_path / "hand_eye.json").read_text()) == result
    roles = [entry["role"] for entry in load_session(tmp_path)["samples"]]
    assert roles == ["train"] * 8 + ["validation"] * 4


@pytest.mark.parametrize(
    ("view", "attempts", "label"),
    [
        (1, 1, "Train 1/8 (home)"),
        (2, 2, "Train 2/8 (halfway retry)"),
        (10, 2, "Validation 2/4 (halfway retry)"),
    ],
)
def test_unreviewed_run_stops_at_home_or_after_one_rejected_retry(
    tmp_path, automatic_run, observations, mocker, view, attempts, label
):
    run, commands, images = automatic_run
    samples, _, _ = observations
    error = MarkerObservationError("Expected exactly one tag36h11 ID 0; detected 0")
    mocker.patch(
        "dimos.calibration.hand_eye_session.detect_sample",
        side_effect=[*samples[: view - 1], *([error] * attempts)],
    )

    with pytest.raises(MarkerObservationError) as caught:
        run(None)

    assert str(caught.value) == f"{label}: {error}"
    assert len(commands) == len(images) == view - 1 + attempts
    assert len(load_session(tmp_path)["samples"]) == view - 1
    assert len(list(tmp_path.glob("*.png"))) == view - 1


def test_unattended_sweep_recovers_training_and_validation_views(
    tmp_path, automatic_run, observations, orbit_mount, tip_from_mount, home_tool_pose, mocker
):
    run, commands, images = automatic_run
    _, mount_from_camera, world_from_tag = observations

    def detect(image, info, measured, marker_size, marker_id):
        if image.data.flat[0] == 9:  # Training 10/12.
            raise MarkerObservationError("Tag not detected")
        if image.data.flat[0] == 15:  # Validation 3/4, after the extra training exposure.
            raise MarkerObservationError("Marker reprojection error exceeds 2 px")
        return HandEyeSample(
            measured, np.linalg.solve(measured @ mount_from_camera, world_from_tag), 0.1
        )

    mocker.patch("dimos.calibration.hand_eye_session.detect_sample", side_effect=detect)
    result = run(None, count=12, tip_from_mount=tip_from_mount)

    assert result["validated"]
    samples = load_session(tmp_path)["samples"]
    assert [s["role"] for s in samples] == ["train"] * 12 + ["validation"] * 4
    assert [s["image_timestamp"] for s in samples] == [
        image.ts for i, image in enumerate(images) if i not in (9, 15)
    ]
    tool_camera = orbit_mount.to_matrix()
    home_camera = Transform.from_pose("tip", home_tool_pose).to_matrix() @ tool_camera
    for rejected, slot in [(9, 9), (15, 14)]:
        requested = Transform.from_pose("tip", commands[rejected]).to_matrix() @ tool_camera
        observed = _home_relative_offsets(samples[slot : slot + 1], samples[0], world_from_tag)[0]
        np.testing.assert_allclose(
            observed, 0.5 * (requested[:3, 3] - home_camera[:3, 3]), atol=1e-8
        )


def test_stop_after_rejection_prevents_retry_move(tmp_path, automatic_run, observations, mocker):
    run, commands, images = automatic_run
    samples, _, _ = observations
    mocker.patch(
        "dimos.calibration.hand_eye_session.detect_sample",
        side_effect=[samples[0], MarkerObservationError("Missing tag")],
    )

    def check_cancelled():
        if len(images) == 2:
            raise InterruptedError("Stop requested")

    with pytest.raises(InterruptedError, match="Stop requested"):
        run(None, check_cancelled=check_cancelled)

    assert len(commands) == len(images) == 2
    assert len(load_session(tmp_path)["samples"]) == 1


def test_stop_during_stationary_capture_prevents_save_and_next_move(tmp_path, automatic_run):
    run, commands, images = automatic_run

    def check_cancelled():
        if images:
            raise InterruptedError("Stop requested")

    with pytest.raises(InterruptedError, match="Stop requested"):
        run(None, check_cancelled=check_cancelled)

    assert commands == [HOME]
    assert load_session(tmp_path)["samples"] == []


def test_resume_before_first_accepted_image_uses_remeasured_home(
    tmp_path, automatic_run, home_tool_pose, mocker
):
    run, commands, _ = automatic_run
    assert run(mocker.Mock(return_value="quit")) is None
    home_tool_pose.position.x += 0.003

    assert run(mocker.Mock(side_effect=["accept", "quit"])) is None

    assert load_session(tmp_path)["home_tool_pose"]["position"] == home_tool_pose.position.to_list()
    assert commands[-1].position.x == home_tool_pose.position.x


@pytest.mark.parametrize("bad_sample", [8, 12], ids=["failed_training", "failed_validation"])
def test_returns_to_zero_when_quality_checks_fail(automatic_run, observations, mocker, bad_sample):
    run, commands, images = automatic_run
    samples, _, _ = observations
    samples[bad_sample - 1].camera_from_target[0, 3] += 0.1
    mocker.patch("dimos.calibration.hand_eye_session.detect_sample", side_effect=samples)

    result = run(mocker.Mock(return_value="accept"))

    assert not result["validated"]
    assert len(images) == bad_sample
    assert commands[-1] == (0.0,) * 6


def test_failed_return_reports_error_and_preserves_result(
    tmp_path, automatic_run, home_tool_pose, mocker
):
    run, commands, _ = automatic_run

    def move_joints(joints):
        commands.append(joints)
        if joints != HOME:
            raise RuntimeError("Return plan rejected")
        return home_tool_pose

    with pytest.raises(RuntimeError, match="Return plan rejected") as error:
        run(mocker.Mock(return_value="accept"), move_joints=move_joints, return_joints=A1Z_RETURN)

    assert f"return to joint positions {list(A1Z_RETURN)} rad" in str(error.value)
    assert commands[-1] == A1Z_RETURN
    assert json.loads((tmp_path / "hand_eye.json").read_text())["validated"]


@pytest.mark.parametrize(
    ("home", "joint_count", "settings"),
    [
        (HOME, 7, {}),
        ((*HOME[:-1], math.nan), 6, {}),
        (HOME, 6, {"count": 7}),
        (HOME, 6, {"validation_count": 3}),
        (HOME, 6, {"return_joints": (0.0,)}),
        (HOME, 6, {"return_joints": (*HOME[:-1], math.inf)}),
    ],
)
def test_invalid_run_settings_rejected_before_motion(
    tmp_path, automatic_run, mocker, home, joint_count, settings
):
    run, commands, _ = automatic_run

    with pytest.raises(ValueError, match="finite positions|at least"):
        run(mocker.Mock(), home=home, joint_count=joint_count, **settings)

    assert commands == []
    assert not (tmp_path / "session.json").exists()


def test_old_joint_schedule_requires_a_new_session_before_motion(tmp_path, automatic_run, mocker):
    from_session = {
        "version": 1,
        "settings": {"automatic_views": {"train": [list(HOME)], "validation": []}},
    }
    (tmp_path / "session.json").write_text(json.dumps(from_session))
    run, commands, _ = automatic_run

    with pytest.raises(ValueError, match="settings changed.*new session"):
        run(mocker.Mock())

    assert commands == []
    assert load_session(tmp_path) == from_session


def test_motion_failure_stops_before_review_or_further_motion(tmp_path, marker_image, mocker):
    _, info, _ = marker_image
    move_joints = mocker.Mock(side_effect=RuntimeError("Joint feedback timed out"))
    move_pose, capture, review = mocker.Mock(), mocker.Mock(), mocker.Mock()

    with pytest.raises(RuntimeError, match="Joint feedback timed out"):
        run_hand_eye(
            tmp_path,
            info,
            home=HOME,
            joint_names=["a", "b", "c", "d", "e", "f"],
            base_frame="base",
            ee_frame="mount",
            tip_frame="tip",
            move_joints=move_joints,
            move_pose=move_pose,
            capture=capture,
            review=review,
        )

    move_joints.assert_called_once_with(HOME)
    move_pose.assert_not_called()
    capture.assert_not_called()
    review.assert_not_called()
    assert load_session(tmp_path)["samples"] == []


@pytest.mark.parametrize("orbit_axis", [None, "x"])
def test_missing_tag_holds_tilt_until_operator_requests_halfway_and_saves_fresh_image(
    tmp_path, marker_image, home_tool_pose, mocker, orbit_axis
):
    template, info, _ = marker_image
    images = [template.copy() for _ in range(6)]
    for index, image in enumerate(images):
        image.ts += index
    for image in images[1:4]:
        image.data.fill(255)
    capture = mocker.Mock(
        side_effect=[
            (image, Transform(frame_id="base", child_frame_id="mount", ts=image.ts))
            for image in images
        ]
    )
    review = mocker.Mock(side_effect=["accept", "accept", "quit"])
    move_pose = mocker.Mock()
    rejected = []
    decisions = iter(["retry", "halfway", "retry"])

    def review_rejected(image, label, can_move_halfway):
        rejected.append((image.ts, label, can_move_halfway, move_pose.call_count))
        return next(decisions)

    result = run_hand_eye(
        tmp_path,
        info,
        home=HOME,
        joint_names=["a", "b", "c", "d", "e", "f"],
        base_frame="base",
        ee_frame="mount",
        tip_frame="tip",
        move_joints=mocker.Mock(return_value=home_tool_pose),
        move_pose=move_pose,
        capture=capture,
        review=review,
        review_rejected=review_rejected,
        # At this printed size the synthetic tag is about 26 cm away.
        marker_size_m=0.065,
        orbit_axis=orbit_axis,
    )

    assert result is None
    assert [call.args[0].ts for call in review.call_args_list] == [images[i].ts for i in (0, 4, 5)]
    assert [(ts, can_move, moves) for ts, _, can_move, moves in rejected] == [
        (images[1].ts, True, 1),
        (images[2].ts, True, 1),
        (images[3].ts, False, 2),
    ]
    assert all("Train 2/8" in label and "detected 0" in label for _, label, _, _ in rejected)
    missed, recovered, next_view = [call.args[0] for call in move_pose.call_args_list]
    assert home_tool_pose.orientation.angle_to(missed.orientation) == pytest.approx(
        math.radians(11)
    )
    assert home_tool_pose.orientation.angle_to(recovered.orientation) == pytest.approx(
        math.radians(5.5)
    )
    # A half-angle recovery shortens both the rotation and the distance along the circular arc.
    assert home_tool_pose.position.distance(recovered.position) == pytest.approx(
        home_tool_pose.position.distance(missed.position)
        * math.sin(math.radians(5.5 / 2))
        / math.sin(math.radians(11 / 2))
    )
    assert next_view.orientation != recovered.orientation
    assert [s["image_timestamp"] for s in load_session(tmp_path)["samples"]] == [
        images[0].ts,
        images[4].ts,
    ]
