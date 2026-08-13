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

"""Replay a recorded Go2 mcap into flat-ground MuJoCo and score joint tracking.

python -m dimos.navigation.motion.simulation <file>.mcap

Recordings and the networks that produced them ship as the
``ml-trajectory-research`` LFS archive; resolve one with
``get_data("ml-trajectory-research/unitree_himloco01.mcap")``.
"""

from __future__ import annotations

import argparse

import numpy as np

from dimos.navigation.motion.simulation import model as go2_model, replay as replay_mod


def main() -> None:
    ap = argparse.ArgumentParser(prog="motion.simulation")
    ap.add_argument("dataset", help="recorded .mcap")
    ap.add_argument("--limit", type=int, default=5000, help="lowcmd samples to replay")
    ap.add_argument("--no-seed", action="store_true", help="start from keyframe, not lowstate")
    ap.add_argument("--view", action="store_true", help="open an interactive MuJoCo window")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed for --view")
    ap.add_argument(
        "--policy",
        help="FREE .bin — drive the policy from the recording's control_log "
        "instead of replaying lowcmd",
    )
    ap.add_argument("--seconds", type=float, default=None, help="policy run length")
    ap.add_argument(
        "--start",
        type=float,
        default=6.0,
        help="skip this many seconds of the recording; the sim always begins "
        "standing, so use this to step over a stand-up",
    )
    ap.add_argument(
        "--ghost", action="store_true", help="draw the recorded vive base_link as a box"
    )
    ap.add_argument(
        "--eval",
        action="store_true",
        help="score the rollout against the recording and print the table",
    )
    ap.add_argument(
        "--physics",
        default="",
        help="leg-joint overrides, e.g. armature=0.03,damping=2.0",
    )
    ap.add_argument(
        "--fitted",
        action="store_true",
        help="use the best-known configuration from FINDINGS (physics, command "
        "delay, actuator lag); --physics entries override individual keys",
    )
    ap.add_argument(
        "--preset",
        default=None,
        help="named physics: 'fitted' (the validated tune), 'stock', or a path "
        "to a preset JSON written by search.py --save-preset",
    )
    ap.add_argument("--command-delay", type=float, default=None, help="seconds")
    ap.add_argument("--actuator-tau", type=float, default=None, help="seconds")
    ap.add_argument(
        "--mount-yaw",
        type=float,
        default=94.0,
        help="robot forward heading within the tracker xy plane, degrees",
    )
    ap.add_argument(
        "--tracker-z",
        type=float,
        default=0.207,
        help="base offset from the tracker in tracker frame, metres; positive is "
        "down in world because the tracker is mounted inverted",
    )
    args = ap.parse_args()

    from dimos.navigation.motion.simulation import evaluate as ev

    overrides = dict(
        (k, float(v)) for k, v in (p.split("=", 1) for p in args.physics.split(",") if p)
    )
    delay = args.command_delay
    tau = args.actuator_tau
    if args.fitted or args.preset:
        preset = ev.load_preset(args.preset)
        overrides = {**preset.physics, **overrides}
        delay = preset.command_delay if delay is None else delay
        tau = preset.actuator_tau if tau is None else tau
    delay = 0.0 if delay is None else delay
    tau = 0.0 if tau is None else tau

    if args.eval:
        if not args.policy:
            raise SystemExit("--eval needs --policy <bin>")
        print(
            ev.evaluate(
                args.dataset,
                args.policy,
                start=args.start,
                seconds=args.seconds,  # None scores the whole recording
                mount_yaw=args.mount_yaw,
                tracker_z=args.tracker_z,
                physics=overrides,
                command_delay=delay,
                actuator_tau=tau,
            ).table()
        )
        return

    if args.policy:
        from dimos.navigation.motion.simulation.policy import FreePolicy
        from dimos.navigation.motion.simulation.walk import (
            read_control_log,
            read_gait_height,
            walk,
        )

        policy = FreePolicy.load(args.policy)
        schedule = read_control_log(args.dataset)
        heights = read_gait_height(args.dataset)
        ghost = None
        if args.ghost:
            from dimos.navigation.motion.simulation.vive import base_track, mount_rotation

            ghost = base_track(
                args.dataset,
                tracker_offset=np.array([0.0, 0.0, args.tracker_z]),
                mount=mount_rotation(args.mount_yaw),
                anchor_at=args.start,
                anchor_pos=np.array([0.0, 0.0, 0.27]),
            )
            print(f"vive: {len(ghost[0])} samples over {ghost[0][-1]:.1f}s")
        print(f"control_log: {len(schedule[0])} walk cmds over {schedule[0][-1]:.1f}s")
        if len(heights[0]):
            print(
                f"gait_height: {len(heights[0])} cmds, "
                f"{heights[1].min():.2f}-{heights[1].max():.2f} m"
            )
        if overrides or delay or tau:
            shown = " ".join(f"{k}={v:g}" for k, v in sorted(overrides.items()))
            print(f"config: {shown} command_delay={delay:g} actuator_tau={tau:g}")
        with ev._physics(overrides):
            track = walk(
                policy,
                schedule=schedule,
                heights=heights,
                seconds=args.seconds,
                start=args.start,
                command_delay=delay,
                actuator_tau=tau,
                view=args.view,
                speed=args.speed,
                ghost=ghost,
            )
        drift = float(np.linalg.norm(track.pos[-1] - track.pos[0]))
        print(f"simulated {track.t[-1]:.1f}s  net displacement {drift:.3f} m")
        print(f"base z: {track.pos[0, 2]:.3f} -> {track.pos[-1, 2]:.3f} m")
        return

    commands = replay_mod.read_commands(args.dataset, limit=args.limit)
    states = replay_mod.read_states(args.dataset, limit=args.limit)
    if not commands:
        raise SystemExit("no lowcmd in dataset")

    print(f"lowcmd  {len(commands)} samples, {commands[-1].ts - commands[0].ts:.2f}s")
    print(f"lowstate {len(states)} samples")

    seed = None if args.no_seed else (states[0] if states else None)
    if args.view:
        replay_mod.view(commands, init_state=seed, speed=args.speed)
        return

    rollout = replay_mod.replay(commands, init_state=seed)
    model, _ = go2_model.load()

    print(f"\nbase drift: {np.linalg.norm(rollout.base_pos[-1] - rollout.base_pos[0]):.3f} m")
    print(f"base z: {rollout.base_pos[0, 2]:.3f} -> {rollout.base_pos[-1, 2]:.3f} m")

    if states:
        e = replay_mod.tracking_error(rollout, states, model)
        print(
            f"\njoint RMS overall: {e['rms_overall']:.4f} rad   max |err|: {e['max_abs']:.4f} rad"
        )
        for name, v in zip(go2_model.MUJOCO_ACTUATOR_NAMES, e["rms_per_joint"], strict=False):
            print(f"  {name:10s} {v:.4f}")


if __name__ == "__main__":
    main()
