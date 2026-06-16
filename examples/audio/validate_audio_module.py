#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Validate AudioModule: construct → start → subscribe → sleep → stop.

Usage:
    # Synthetic (no mic needed):
    python examples/audio/validate_audio_module.py --synthetic

    # Real microphone (macOS: grant mic permission on first run):
    python examples/audio/validate_audio_module.py
"""

import argparse
import time

import numpy as np

from dimos.hardware.sensors.audio.module import AudioModule
from dimos.msgs.audio_msgs.AudioStamped import AudioStamped


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--synthetic", action="store_true", default=True,
                        help="Use sine-tone source instead of mic (default: True)")
    parser.add_argument("--real-mic", dest="synthetic", action="store_false",
                        help="Use real microphone input")
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Seconds to collect audio (default: 2.0)")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--channels", type=int, default=1)
    parser.add_argument("--frame-ms", type=int, default=20)
    args = parser.parse_args()

    # ------------------------------------------------------------------ #
    # 1. LCM round-trip smoke test (before any hardware)                  #
    # ------------------------------------------------------------------ #
    print("=== LCM round-trip test ===")
    n_samples = args.sample_rate * args.frame_ms // 1000
    pcm_orig = (np.random.randn(n_samples).clip(-1, 1) * 32767).astype(np.int16)
    orig = AudioStamped.from_pcm(
        pcm_bytes=pcm_orig.tobytes(),
        sample_rate=args.sample_rate,
        channels=args.channels,
        sample_format="S16LE",
        coding_format="pcm",
        ts=time.monotonic(),
    )
    wire = orig.lcm_encode()
    decoded = AudioStamped.lcm_decode(wire)

    assert decoded.sample_rate == orig.sample_rate, "sample_rate mismatch"
    assert decoded.channels == orig.channels, "channels mismatch"
    assert decoded.sample_format == orig.sample_format, "sample_format mismatch"
    assert decoded.coding_format == orig.coding_format, "coding_format mismatch"
    assert decoded.data == orig.data, "PCM data mismatch"
    assert abs(decoded.ts - orig.ts) < 1e-6, "timestamp mismatch"
    print(f"  wire bytes : {len(wire)}")
    print(f"  orig       : {orig}")
    print(f"  decoded    : {decoded}")
    print("  PASS\n")

    # ------------------------------------------------------------------ #
    # 2. Construct + start AudioModule                                     #
    # ------------------------------------------------------------------ #
    source = "synthetic sine-tone" if args.synthetic else "real microphone"
    print(f"=== AudioModule ({source}) ===")
    print(f"  sample_rate={args.sample_rate}, channels={args.channels}, "
          f"frame_ms={args.frame_ms}, duration={args.duration}s")

    mod = AudioModule(
        sample_rate=args.sample_rate,
        channels=args.channels,
        frame_ms=args.frame_ms,
        sample_format="S16LE",
        synthetic=args.synthetic,
    )

    # Print io() shape (mirrors CameraModule)
    print("\n--- io() ---")
    print(mod.io(color=False))
    print("------------\n")

    # ------------------------------------------------------------------ #
    # 3. Subscribe and collect                                             #
    # ------------------------------------------------------------------ #
    counts = [0]
    last_ts = [None]
    gaps: list[float] = []

    def on_audio(msg: AudioStamped) -> None:
        counts[0] += 1
        arr = msg.to_numpy()
        ts = msg.ts
        if last_ts[0] is not None:
            gaps.append(ts - last_ts[0])
        last_ts[0] = ts

        if counts[0] <= 5 or counts[0] % 10 == 0:
            print(
                f"  [{counts[0]:4d}] rate={msg.sample_rate} ch={msg.channels} "
                f"shape={arr.shape} dtype={arr.dtype} ts={ts:.6f}"
            )

    unsub_fn = mod.audio.subscribe(on_audio)
    mod.start()

    time.sleep(args.duration)

    mod.stop()
    unsub_fn()

    # ------------------------------------------------------------------ #
    # 4. Report                                                            #
    # ------------------------------------------------------------------ #
    print("\n=== Results ===")
    expected_rate = 1000.0 / args.frame_ms
    actual_rate = counts[0] / args.duration if args.duration > 0 else 0.0
    avg_gap_ms = (sum(gaps) / len(gaps) * 1000) if gaps else 0.0

    print(f"  frames received : {counts[0]}")
    print(f"  expected rate   : {expected_rate:.1f} Hz")
    print(f"  actual rate     : {actual_rate:.1f} Hz")
    print(f"  avg frame gap   : {avg_gap_ms:.2f} ms (expected {args.frame_ms} ms)")

    # Check monotonic timestamps
    if len(gaps) > 0:
        assert all(g > 0 for g in gaps), "timestamps are not monotonically increasing!"
        print("  timestamps      : monotonically increasing ✓")

    assert counts[0] > 0, "No audio frames received!"
    print("\nDONE — AudioModule validated successfully.")


if __name__ == "__main__":
    main()
