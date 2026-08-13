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

"""What has to hold for the learned candidate to be judged honestly."""

from __future__ import annotations

import numpy as np
import pytest
import torch

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.motion.control.profile import encode_precision
from dimos.navigation.motion.control.research.ml import env, obs, policy


def _pose(x: float, y: float, yaw: float = 0.0, t: float = 0.0) -> PoseStamped:
    return PoseStamped(
        ts=t,
        frame_id="world",
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _straight(n: int = 20, step: float = 0.25) -> Path:
    return Path(frame_id="world", poses=[_pose(i * step, 0.0) for i in range(n)])


def test_obs_dim_matches_the_builder() -> None:
    feat = obs.build((0.0, 0.0, 0.0), _straight(), 0.0, None, np.zeros(3), np.zeros(3), np.zeros(3))
    assert feat.shape == (obs.DIM,)
    assert feat.dtype == np.float32
    assert np.all(np.isfinite(feat))


def test_scales_cover_every_input() -> None:
    """A missing scale would silently leave one input unnormalised."""
    assert len(obs.MARK_SCALE) == obs.MARK_DIM
    assert len(obs.SCALAR_SCALE) == obs.SCALAR_DIM
    assert obs.DIM == obs.MARK_TOTAL + obs.SCALAR_DIM


def test_ego_frame_turns_with_the_body() -> None:
    """The path ahead reads as +x forward whatever the world yaw is."""
    for yaw in (0.0, 1.0, -2.5, 3.0):
        c, s = np.cos(yaw), np.sin(yaw)
        path = Path(
            frame_id="world",
            poses=[_pose(i * 0.25 * c, i * 0.25 * s, yaw) for i in range(20)],
        )
        feat = obs.build((0.0, 0.0, yaw), path, 0.0, None, np.zeros(3), np.zeros(3), np.zeros(3))
        marks = feat[: obs.MARK_TOTAL].reshape(-1, obs.MARK_DIM)
        assert np.all(marks[:, 0] >= -1e-6), "forward path must have non-negative ego x"
        assert np.allclose(marks[:, 1], 0.0, atol=1e-6), "straight path has no ego y"


def test_cross_track_sign_is_left_positive() -> None:
    left = obs.build((1.0, 0.3, 0.0), _straight(), 0.0, None, np.zeros(3), np.zeros(3), np.zeros(3))
    right = obs.build(
        (1.0, -0.3, 0.0), _straight(), 0.0, None, np.zeros(3), np.zeros(3), np.zeros(3)
    )
    assert left[obs.MARK_TOTAL + obs.S_CROSS] > 0.0
    assert right[obs.MARK_TOTAL + obs.S_CROSS] < 0.0


def test_blind_recovers_the_hinted_room_from_the_stamps() -> None:
    """The two tracks fill the same slot, which is what lets one net serve both."""
    room = np.full(20, 0.18)
    hinted_path, blind_path = _straight(), _straight()
    encode_precision(blind_path, room, t0=0.0)

    hinted = obs.build(
        (0.0, 0.0, 0.0), hinted_path, 0.0, room, np.zeros(3), np.zeros(3), np.zeros(3)
    )
    blind = obs.build((0.0, 0.0, 0.0), blind_path, 0.0, None, np.zeros(3), np.zeros(3), np.zeros(3))
    marks_h = hinted[: obs.MARK_TOTAL].reshape(-1, obs.MARK_DIM)
    marks_b = blind[: obs.MARK_TOTAL].reshape(-1, obs.MARK_DIM)
    np.testing.assert_allclose(marks_b[:, 4], marks_h[:, 4], atol=1e-6)

    assert hinted[obs.MARK_TOTAL + obs.S_TRACK] == obs.HINTED
    assert blind[obs.MARK_TOTAL + obs.S_TRACK] == obs.BLIND


def test_unstamped_path_reads_as_room_not_as_a_pinch() -> None:
    feat = obs.build((0.0, 0.0, 0.0), _straight(), 0.0, None, np.zeros(3), np.zeros(3), np.zeros(3))
    marks = feat[: obs.MARK_TOTAL].reshape(-1, obs.MARK_DIM)
    assert np.allclose(marks[:, 4], obs.NO_HINT_ROOM)


def test_twist_clips_the_planar_pair_as_a_vector() -> None:
    """Per-axis clipping would let a diagonal reach max_speed * sqrt(2)."""
    vx, vy, wz = policy.to_twist(np.array([1.0, 1.0, 1.0]), max_speed=0.5, max_yaw_rate=1.4)
    assert np.hypot(vx, vy) == pytest.approx(0.5)
    assert wz == pytest.approx(1.4)


def test_action_saturation_cannot_leave_the_envelope() -> None:
    for a in (np.array([5.0, -5.0, 9.0]), np.array([-3.0, 0.0, -7.0])):
        vx, vy, wz = policy.to_twist(a, max_speed=0.95, max_yaw_rate=1.4)
        assert np.hypot(vx, vy) <= 0.95 + 1e-9
        assert abs(wz) <= 1.4 + 1e-9


def test_agent_holds_on_a_veto_stub() -> None:
    """A single-pose path is the planner saying stop; every law returns zero."""
    agent = env.Agent(policy.ActorCritic(), deterministic=True)
    stub = Path(frame_id="world", poses=[_pose(0.0, 0.0)])
    tw = agent.update(_pose(0.0, 0.0), stub, 0.0, None)
    assert (tw.linear.x, tw.linear.y, tw.angular.z) == (0.0, 0.0, 0.0)


def test_reset_makes_a_used_agent_fresh() -> None:
    """The discipline laws/hinted.py holds itself to, for the same reason."""
    agent = env.Agent(policy.ActorCritic(), deterministic=True)
    path = _straight()
    first = agent.update(_pose(0.0, 0.0), path, 0.0, None)
    agent.update(_pose(0.4, 0.0), path, 0.02, None)
    agent.reset()
    again = agent.update(_pose(0.0, 0.0), path, 0.0, None)
    assert again.linear.x == first.linear.x
    assert again.angular.z == first.angular.z


def test_tape_records_one_row_per_commanded_tick() -> None:
    tape = env.Tape()
    agent = env.Agent(policy.ActorCritic(), tape=tape)
    path = _straight()
    for k in range(5):
        agent.update(_pose(k * 0.1, 0.0), path, k * 0.02, None)
    assert len(tape) == 5
    assert len(tape.feat) == len(tape.action) == len(tape.logp) == len(tape.value) == 5
    assert tape.feat[0].shape == (obs.DIM,)


def test_velocity_survives_the_zero_order_hold() -> None:
    """Pose updates at 29 Hz, the controller ticks at 50: repeats are not stops."""
    v = obs.Velocity()
    # 0.5 m/s forward, but the pose only refreshes every other tick
    v.update((0.0, 0.0, 0.0), 0.0)
    v.update((0.0, 0.0, 0.0), 0.02)  # held sample: no new information
    est = v.update((0.02, 0.0, 0.0), 0.04)
    assert est[0] == pytest.approx(0.5, abs=1e-9)
    held = v.update((0.02, 0.0, 0.0), 0.06)  # held again
    assert held[0] == pytest.approx(0.5, abs=1e-9), "a repeat must hold, not read zero"


def test_velocity_is_in_the_ego_frame() -> None:
    """Moving along world +y while facing +y is FORWARD, not sideways."""
    v = obs.Velocity()
    v.update((0.0, 0.0, np.pi / 2), 0.0)
    est = v.update((0.0, 0.05, np.pi / 2), 0.1)
    assert est[0] == pytest.approx(0.5, abs=1e-9)
    assert est[1] == pytest.approx(0.0, abs=1e-9)


def test_velocity_resets_with_the_agent() -> None:
    agent = env.Agent(policy.ActorCritic(), deterministic=True)
    path = _straight()
    agent.update(_pose(0.0, 0.0), path, 0.0)
    agent.update(_pose(0.5, 0.0), path, 0.1)
    agent.reset()
    feat = obs.build(
        (0.0, 0.0, 0.0),
        path,
        0.0,
        None,
        np.zeros(3),
        np.zeros(3),
        agent._vel.update((0.0, 0.0, 0.0), 0.0),
    )
    assert feat[obs.MARK_TOTAL + obs.S_VEL_X] == 0.0


def test_teacher_tapes_the_law_it_wraps() -> None:
    """BC data is the law's own action against the net's own observation."""
    from dimos.navigation.motion.control.laws.seed import make as make_seed

    law = make_seed()
    tape = env.Tape()
    teacher = env.TeacherAgent(law, config=law.config, tape=tape)
    path = _straight()
    twist = teacher.update(_pose(0.0, 0.0), path, 0.0, None)
    assert len(tape) == 1
    assert tape.feat[0].shape == (obs.DIM,)
    # the tape stores the law's twist, normalised into the net's action space
    np.testing.assert_allclose(tape.action[0][0], twist.linear.x / law.config.max_speed, atol=1e-9)
    np.testing.assert_allclose(
        tape.action[0][2], twist.angular.z / law.config.max_yaw_rate, atol=1e-9
    )


def test_teacher_passes_the_law_through_untouched() -> None:
    """Taping must not change what the law commands, or BC clones a ghost."""
    from dimos.navigation.motion.control.laws.seed import make as make_seed

    path = _straight()
    bare, wrapped = make_seed(), make_seed()
    teacher = env.TeacherAgent(wrapped, config=wrapped.config, tape=env.Tape())
    for k in range(6):
        p = _pose(k * 0.2, 0.05, 0.1)
        a = bare.update(p, path, k * 0.02, None)
        b = teacher.update(p, path, k * 0.02, None)
        assert (a.linear.x, a.linear.y, a.angular.z) == (b.linear.x, b.linear.y, b.angular.z)


def test_clone_round_trips_through_the_candidate_envelope() -> None:
    """Normalise with the config the Agent will denormalise with, or the clone
    commands a different speed than the law it copied."""
    from dimos.navigation.motion.control.laws.seed import make as make_seed
    from dimos.navigation.motion.control.research.ml.candidate import config as cand_config

    cfg = cand_config()
    law = make_seed()  # a law whose own envelope differs from the candidate's
    assert law.config.max_speed != cfg.max_speed, "test needs mismatched envelopes"

    tape = env.Tape()
    teacher = env.TeacherAgent(law, config=cfg, tape=tape)
    path = _straight()
    twist = teacher.update(_pose(0.0, 0.1, 0.0), path, 0.0, None)
    vx, vy, wz = policy.to_twist(tape.action[0], cfg.max_speed, cfg.max_yaw_rate)
    assert (vx, vy, wz) == pytest.approx((twist.linear.x, twist.linear.y, twist.angular.z))


def test_standing_still_is_not_free() -> None:
    """The hole W_TIME closes: a timeout must score worse than moving."""
    assert env.W_TIME > 0.0
    # 40 s of standing still costs more than the judge pays for a clean
    # precision pillar on a world the robot never entered
    assert env.W_TIME * 40.0 > 10.0 * env.TERMINAL_SCALE


def test_value_loss_cannot_reach_the_actor() -> None:
    """The bug that destroyed a cloned policy: a random critic's gradients
    flowing through a shared trunk into the actor's features."""
    net = policy.ActorCritic()
    feat = torch.randn(16, obs.DIM)
    _, _, value = net(feat)
    value.pow(2).mean().backward()

    for name, p in net.named_parameters():
        touched = p.grad is not None and bool(torch.any(p.grad != 0))
        if name.startswith(("critic_trunk", "v.")):
            assert touched, f"critic param {name} got no gradient"
        else:
            assert not touched, f"value loss reached actor param {name}"


def test_actor_and_critic_parameter_sets_partition_the_net() -> None:
    """Two optimisers, so every parameter must be owned by exactly one."""
    net = policy.ActorCritic()
    actor = {id(p) for p in net.actor_parameters()}
    critic = {id(p) for p in net.critic_parameters()}
    every = {id(p) for p in net.parameters()}
    assert not actor & critic, "a parameter is stepped by both optimisers"
    assert actor | critic == every, "a parameter is stepped by neither"


def test_rewards_are_scaled_for_the_critic() -> None:
    """Judge-point weights stay readable; the optimiser gets O(1) targets."""
    assert 0.0 < env.REWARD_SCALE < 1.0
    assert abs(env.TERMINAL_SCALE * 115.0 * env.REWARD_SCALE) < 10.0


def test_checkpoint_rejects_a_stale_observation_format(tmp_path) -> None:
    """A changed builder must break loudly, not mismatch silently."""
    net = policy.ActorCritic()
    path = str(tmp_path / "net.pt")
    policy.save(path, net)
    blob = torch.load(path, map_location="cpu", weights_only=False)
    blob["format"] = "something-else-v9"
    torch.save(blob, path)
    with pytest.raises(ValueError, match="observation format"):
        policy.load(path)


def test_gae_is_terminal_at_the_end_of_an_episode() -> None:
    """No bootstrap past the end: a timeout is a failure, not a truncation."""
    reward = np.array([0.0, 0.0, 10.0], dtype=np.float32)
    value = np.zeros(3, dtype=np.float32)
    from dimos.navigation.motion.control.research.ml.train import gae

    adv, ret = gae(reward, value, gamma=1.0, lam=1.0)
    np.testing.assert_allclose(adv, [10.0, 10.0, 10.0])
    np.testing.assert_allclose(ret, adv)


def test_training_seeds_avoid_every_eval_battery() -> None:
    """Seed hygiene, as a test rather than a comment."""
    from dimos.navigation.motion.control.referee.battery import OOD_SEED
    from dimos.navigation.motion.scenarios import GEN_COUNT, GEN_SEED

    battery = range(GEN_SEED, GEN_SEED + max(GEN_COUNT, 1000))
    ood = range(OOD_SEED, OOD_SEED + 1000)
    for base, count in ((env.TRAIN_SEED0, 5000), (env.VAL_SEED0, 1000)):
        used = range(base, base + count)
        assert not set(used) & set(battery)
        assert not set(used) & set(ood)
    assert not set(range(env.VAL_SEED0, env.VAL_SEED0 + 1000)) & set(
        range(env.TRAIN_SEED0, env.TRAIN_SEED0 + 5000)
    )
