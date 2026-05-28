# Pirate — Go2 Vision-Guided Object Interaction

Hackathon submission from 0xmandy.

## Summary

Pirate is a vision-servoing stack for Unitree Go2 that makes the robot hunt down
objects like a pirate hunting treasure. The robot scans with its onboard camera,
runs YOLO detection to find a target (bottle or ball), closes in with bearing-locked
visual control, pushes the object, then celebrates with sport gestures.

The core loop is:

```text
camera frame → YOLO detection → bearing + distance estimate
             → approach burst → realign → push → Hello → backup → Scrape
```

This submission keeps DimOS as the robot/runtime layer and adds a vision-servoing
control layer with replayable per-run decision traces on top.

## Links

- Project repo: https://github.com/0xmandy/pirate
- Final demo video: https://github.com/0xmandy/pirate/blob/main/artifacts/showcase/final_demo.mp4
- Decision traces: https://github.com/0xmandy/pirate/tree/main/artifacts/decision_traces
- Sensor stream profile: https://github.com/0xmandy/pirate/blob/main/artifacts/sensor_stream_profile.json

## What We Built

- YOLO visual servoing: detect bottle/ball → extract bearing → drive toward it
- Closed-loop realign: if yaw error > 8° at standoff, back off and re-approach
- Push phase: forward burst 0.35 m/s × 0.5 s, displacement confirmed via ball pose
- Full gesture sequence: approach → push → Hello (api_id=1016) → backup → Scrape (api_id=1029)
- MuJoCo sim proof: ball freejoint physics, push displacement 0.118–0.196 m confirmed
- Real hardware smoke test: odom ~19 Hz, camera ~10–12 Hz, lidar ~7–8 Hz verified
- Per-run decision trace JSON: yaw error, realign decision, push outcome, ball displacement

## Why It Fits DimOS

DimOS already exposes everything needed: streams, camera frames, odometry, and
Go2 control skills. Pirate adds a thin vision-and-action evidence layer on top:

```text
DimOS observes and executes
YOLO guides the approach bearing
decision traces explain each run
```

That boundary keeps DimOS as the hardware/control system while making robot
decisions inspectable and reproducible.

## Results

| Metric | Value |
|---|---|
| Push displacement — run 001 | **0.118 m** (yaw 11.5° → realign → 7.1°) |
| Push displacement — run 003 | **0.196 m** (yaw 9.6° → realign → 1.5°) |
| Vision approach success rate | 100% (2/2 valid runs) |
| Premature contact abort rate | ~33% (1/3); fix: offset approach target_y by −0.15 m |
| Real odom rate | ~19 Hz |
| Real camera rate | ~10–12 Hz |
| Confirmed gestures on hardware | Hello ✅  Scrape ✅ |

## Repro

Clone the project repo and run:

```bash
# Syntax check all scripts
make check

# Mujoco sim — ball scene (no robot needed)
make sim-run

# Real robot — water bottle
python examples/go2_bottle_demo_sequence.py \
    --backend webrtc-ap --ball-distance 0.8 \
    --execute --i-understand-this-is-real-robot
```

## Submission Note

This PR intentionally does not vendor the full project into DimOS. The full source,
generated artifacts, decision traces, and demo video live in the project repo
linked above. DimOS core files are not modified by this submission.
