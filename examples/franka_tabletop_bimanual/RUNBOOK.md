# Morning runbook: tabletop bimanual FR3 demo

This folder is `examples/franka_tabletop_bimanual/` in the dimOS repo; commands written for `~/franka_3_setup` run from this folder.

Two FR3s clamped to one table, driven by dimOS (`fr3-tabletop-bimanual`).
"Right" = arm 1 (the original), "left" = arm 2, seen from behind the arms.

| | Right (arm 1) | Left (arm 2) |
|---|---|---|
| Desk / FCI | https://10.42.0.230/desk | https://10.43.0.53/desk |
| Laptop port | USB adapter `enx9c69d3c7dc9f` (10.42.0.1) | built-in `enp130s0` (10.43.0.1) |
| Wrist camera | RealSense 261922077925 | RealSense 261922072855 |

Tripod ("scene") camera: RealSense 261922070328, on the short USB cable.

## 1. Power on (cold start, ~5 min)

1. Power on both control boxes. Wait for Desk to load in each tab.
2. In each Desk:
   1. **Unlock** the joints.
   2. **Execution** mode (not Programming).
   3. End effector: **initialised** and **powered**. Initialising opens the Hand.
   4. Menu under the robot's address: **Activate FCI**.
3. On the laptop:
   ```bash
   ping -c2 10.42.0.230 && ping -c2 10.43.0.53
   pgrep -af "dimos run"        # must print nothing; kill any leftover stack
   ```

## 2. Start the stack

For the handover demo, use the defaults: the guard keeps the arms 5 cm apart and the
grippers 5 mm apart, and the Hands grip at 20 N.

```bash
cd ~/dimos-franka && .venv/bin/dimos run fr3-tabletop-bimanual \
    --right_arm.hold_on_short_end true --left_arm.hold_on_short_end true
```

Wait for `ControlCoordinator started at 100.0Hz` (about 15 s). Leave it running in its
own terminal. All the commands below run from `~/franka_3_setup` in another terminal:

```bash
cd ~/franka_3_setup
PY=~/dimos-franka/.venv/bin/python
$PY handoff.py recover       # both arms to ready (lifts any arm left low first)
```

## 3. The demo: pick, hand over, place, any box-like object

Put the eraser (or the small box standing on its side) anywhere on the table, at least
25 cm from either base and clear of the taped board.

```bash
$PY handoff.py look          # find it (moves a wrist camera over it), no grasp
$PY handoff.py cycle         # find, pick, hand over mid-air, place on the other side
$PY handoff.py cycle --relay # same, but the giver sets it down in the middle instead
$PY handoff.py loop -n 3     # three cycles, back and forth
```

Each cycle:
1. Re-measures the tripod from the board.
2. Finds the object with no prior knowledge of it.
3. Picks it with the arm on its side.
4. Searches for a handover pose that both arms reach and the guard allows.
5. Hands the object over.
6. Places it on the other arm's side.

Thin (<2 cm) or short (<9 cm) objects go by the relay automatically. A cycle takes
about 80 s. Results are logged to `handoff_log.jsonl`.

Things worth pointing out while it runs:
- **Wrist camera close-look:** the giver measures the object from 35 cm.
- **Handover search:** prints how many handover poses it found and how long it took.
- **Hand-over order:** the giver opens only after the receiver's fingers have closed.
- **Movement checks:** every move is verified against the measured joints.

## 4. If an arm faults (both did, repeatedly, late on 2026-11-22/23)

Symptom: the arm's light goes off or red, the script prints `STOPPED`/`FAILED`, and the
stack log shows `UDP receive: Timeout` then `Connection to FCI refused`.

1. Stop the stack (Ctrl+C in its terminal) so nothing keeps reconnecting.
2. **If that arm's Hand holds something, hold it now:** recovery opens the Hand.
3. Desk: acknowledge the error, or **Reboot** (about 1 min). If "error could not be
   acknowledged": shut down, wait for the fans to stop, power on. A "Generic joint error"
   asks to run the self tests: accept, with the Hand empty.
4. Unlock, Execution, end effector initialised and powered, FCI on.
5. Start the stack again (step 2), then `$PY handoff.py recover`.

**Never:**
- Restart the stack while a Hand holds something: on startup the connection sends the
  Hand a "move", which drops its grip force.
- Run `recover` or any planned move while a Hand holds something long (a pen, a marker):
  the planner does not know about it and can sweep it into the table. Open the Hand and
  take the object out first.

## 5. Optional: the pen demo (unfinished)

It has never run start to finish. Its pieces that did work:
- finding the pen and the paper, including which end is the tip;
- the relay;
- the pick;
- one camera-aimed click;
- finding the paper by touch.

It needs its own stack settings:

```bash
cd ~/dimos-franka && .venv/bin/dimos run fr3-tabletop-bimanual \
    --right_arm.grasp_force 70 --left_arm.grasp_force 15 \
    --separationguard.arm_separation_m 0.015 \
    --right_arm.hold_on_short_end true --left_arm.hold_on_short_end true
```

Run it one stage at a time, checking `pen_wrist.png` after `look` (and after `relay`):
the red circle must be on the silver tip. If it is on the button, run `flip`.

```bash
$PY pen_demo.py look relay plan          # perception and planning only
$PY pen_demo.py flip plan                # only if the ends were swapped
$PY pen_demo.py pick click touch draw back click putdown
```

Safe to show even if motion is risky: `look` and `plan` move only to camera poses, and
`pen_scene.png` shows the planned smiley on the paper.
