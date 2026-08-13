#!/usr/bin/env python3
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

"""Standalone joint-space jog teleop for OpenYAM — no coordinator stack.

Single process, pygame window in the main thread, OpenYamAdapter driven
directly at 100 Hz. Exists because the full keyboard-teleop blueprint
currently hangs on macOS (pygame cannot run inside a worker process); use
``keyboard-teleop-openyam-can`` on Linux instead.

Controls (window must be focused):
    1..6:       select joint
    UP / DOWN:  jog selected joint +/-
    [ / ]:      gripper open / close (uncalibrated range — go gently)
    SPACE:      stop jogging (hold pose)
    ESC:        stop, disable, exit

Safety: joint targets are clamped to --range rad around the start pose,
jog speed is --speed rad/s, and the loop aborts (torque off) if any joint
diverges from its command by more than --abort-error rad.
"""

from __future__ import annotations

import argparse
import sys
import time

from dimos.hardware.manipulators.openyam.adapter import OpenYamAdapter

# The CANable 2.0 is a Full-Speed USB device: ~1 CAN frame per ~1 ms libusb
# round-trip, so sustained bus traffic must stay well under ~1000 frames/s
# (each command produces a reply AND a TX echo). 50 Hz x 6 joints + 10 Hz
# gripper ≈ 700 frames/s. At 100 Hz the reader falls behind and motor state
# goes stale mid-session (observed).
RATE_HZ = 50.0
GRIPPER_EVERY_N_TICKS = 5


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--channel", default="gs_usb:0")
    ap.add_argument("--speed", type=float, default=0.15, help="jog speed, rad/s")
    ap.add_argument("--gripper-speed", type=float, default=0.4, help="gripper jog speed, rad/s")
    ap.add_argument(
        "--range", type=float, default=0.6, help="max excursion from start pose per joint, rad"
    )
    ap.add_argument("--abort-error", type=float, default=0.35)
    args = ap.parse_args()

    import pygame

    adapter = OpenYamAdapter(address=args.channel)
    if not adapter.connect():
        sys.exit("connect failed")
    if not adapter.write_enable(True):
        adapter.disconnect()
        sys.exit("enable failed")
    # Motors reply only to command frames; retry the initial read for a
    # couple of seconds, re-pulsing enable (each pulse elicits one state
    # reply per motor) instead of failing on the first 50 ms.
    q_start = None
    last_err: Exception | None = None
    deadline = time.monotonic() + 2.5
    while q_start is None and time.monotonic() < deadline:
        try:
            q_start = adapter.read_joint_positions()
        except RuntimeError as e:
            last_err = e
            adapter.write_enable(True)
            # Short wait only: the state reply lands within ms of the enable
            # pulse, and the freshness window is 100 ms — sleeping 0.1 s here
            # made every read exactly stale.
            time.sleep(0.03)
    if q_start is None:
        adapter.write_enable(False)
        adapter.disconnect()
        sys.exit(f"no joint state after 2.5s — replug the dongle and retry ({last_err})")

    q_target = list(q_start)
    gripper_target = adapter.read_gripper_position()

    pygame.init()
    screen = pygame.display.set_mode((560, 330))
    pygame.display.set_caption("OpenYAM jog teleop")
    font = pygame.font.SysFont("monospace", 15)
    # Pace with time.sleep, NOT pygame.time.Clock.tick: tick() holds the
    # GIL during its wait, starving the USB reader thread — motor state goes
    # stale within seconds (verified: identical traffic paced by time.sleep
    # runs indefinitely clean).
    next_tick = time.monotonic()

    selected = 5  # joint 6, the wrist — safest default
    running = True
    exit_code = 0
    tick = 0
    t0 = time.monotonic()
    stall_started: float | None = None
    q_now = list(q_start)
    try:
        while running:
            tick += 1
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif pygame.K_1 <= event.key <= pygame.K_6:
                        selected = event.key - pygame.K_1

            keys = pygame.key.get_pressed()
            dt = 1.0 / RATE_HZ
            if keys[pygame.K_UP]:
                q_target[selected] += args.speed * dt
            if keys[pygame.K_DOWN]:
                q_target[selected] -= args.speed * dt
            lo = q_start[selected] - args.range
            hi = q_start[selected] + args.range
            q_target[selected] = max(lo, min(hi, q_target[selected]))
            # Leash: never let any target run further than 0.25 rad ahead of
            # the measured position. Jogging into an obstacle or a joint that
            # can't keep up then stalls gently (bounded torque) instead of
            # diverging until the tracking abort fires.
            for i in range(6):
                q_target[i] = max(q_now[i] - 0.25, min(q_now[i] + 0.25, q_target[i]))

            if gripper_target is not None:
                if keys[pygame.K_LEFTBRACKET]:
                    gripper_target -= args.gripper_speed * dt  # opening decreases position
                if keys[pygame.K_RIGHTBRACKET]:
                    gripper_target += args.gripper_speed * dt
                gripper_target = max(-0.2, min(1.6, gripper_target))

            if not adapter.write_joint_positions(q_target):
                print("write failed — aborting")
                exit_code = 2
                break
            if gripper_target is not None and tick % GRIPPER_EVERY_N_TICKS == 0:
                adapter.write_gripper_position(gripper_target)

            # Tolerate sub-second state dropouts (transient RX stalls happen
            # on this dongle): hold the last known state, log the stall, and
            # abort only if the blackout exceeds 1.0 s. Jog speed and range
            # clamps bound the blind travel to well under 0.2 rad.
            try:
                q_now = adapter.read_joint_positions()
                if stall_started is not None:
                    print(
                        f"t={time.monotonic() - t0:7.2f}s stall ended after "
                        f"{time.monotonic() - stall_started:.2f}s",
                        flush=True,
                    )
                    stall_started = None
            except RuntimeError as e:
                if stall_started is None:
                    stall_started = time.monotonic()
                    print(f"t={time.monotonic() - t0:7.2f}s stall began ({e})", flush=True)
                elif time.monotonic() - stall_started > 1.0:
                    print(f"state blackout >1s — aborting ({e})")
                    exit_code = 2
                    break
            worst = max(abs(a - b) for a, b in zip(q_now, q_target, strict=True))
            if worst > args.abort_error:
                print(f"tracking error {worst:.3f} rad — aborting")
                exit_code = 2
                break

            # HUD at 10 Hz only: per-tick font rendering holds the GIL long
            # enough to starve the USB reader thread, and the dongle's FIFO
            # overflows (observed as stale motor state mid-session).
            next_tick += 1.0 / RATE_HZ
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
            if tick % 5 != 0:
                continue

            screen.fill((18, 18, 24))
            header = f"joint {selected + 1} selected   UP/DOWN jog   [/] gripper   ESC quit"
            screen.blit(font.render(header, True, (240, 240, 240)), (14, 12))
            for i in range(6):
                color = (120, 220, 120) if i == selected else (170, 170, 170)
                line = (
                    f"j{i + 1}  target {q_target[i]:+.3f}  actual {q_now[i]:+.3f}  "
                    f"range [{q_start[i] - args.range:+.2f}, {q_start[i] + args.range:+.2f}]"
                )
                screen.blit(font.render(line, True, color), (14, 46 + i * 24))
            if gripper_target is not None:
                g_now = adapter.read_gripper_position()
                g_txt = (
                    f"gripper  target {gripper_target:+.3f}  actual {g_now:+.3f}"
                    if g_now is not None
                    else "gripper  (no state)"
                )
                screen.blit(font.render(g_txt, True, (200, 180, 120)), (14, 46 + 6 * 24))
            screen.blit(
                font.render(f"tracking worst-case error {worst:.3f} rad", True, (140, 140, 160)),
                (14, 46 + 7 * 24 + 8),
            )
            pygame.display.flip()
    finally:
        try:
            adapter.write_stop()
            time.sleep(0.1)
            adapter.write_enable(False)
        finally:
            adapter.disconnect()
            pygame.quit()
            print("disabled and disconnected")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
