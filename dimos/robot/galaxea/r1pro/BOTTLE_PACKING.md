# Five-bottle ACT packing

The R1Pro picks five matching bottles and places them upright in planned tray
slots using ACT. The `policy-packing-augmented` checkpoint passed repeated full
native DimOS runs and offline checks. The base stays parked by default; add
`--deliver-to-laptop` to carry the loaded tray to the laptop table afterward.
The single-bottle demo remains in [ACT_SIM.md](ACT_SIM.md).

## Run with the full display

Run from a desktop terminal on this workstation:

```bash
cd /home/mustafa/dimos-wt/r1pro-act-sim
source .venv/bin/activate
export PYTHONPATH="$PWD"
export MUJOCO_GL=glfw
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
export DIMOS_TRANSPORT=lcm
export LCM_DEFAULT_URL="udpm://224.0.0.224:19467?ttl=0"

python -m dimos.robot.galaxea.r1pro.demo_packing_stack \
  --artifact "$PWD/recordings/r1pro-act-task/policy-packing-augmented" \
  --output "$PWD/recordings/r1pro-act-task/my-five-bottle-run" \
  --zenoh-scout-addr 224.0.0.224:19467 \
  --scene-package /home/mustafa/dimos/data/scene_packages/hssd_102344115 \
  --seed 5000 --stay-open
```

If prompted about optional LCM socket-buffer optimization, answer **n**; the
validated runs used the existing host settings. No OpenAI API key is needed.
The viewer stays open after completion; close its window or press Ctrl-C to
stop the stack. Use a fresh output directory for a separate result history.

This command explicitly uses LCM, which passed the native runs. Keep its address
and `--zenoh-scout-addr` value equal: the latter also names the launcher's session
lock, despite its legacy name. Close another R1Pro demo on the same address
before starting. Concurrent stacks need distinct addresses and output paths.
Zenoh previously exhibited intermittent RPC stalls and is not the validated
transport for this command.

## Pack, carry, and deliver to the laptop

After the environment setup above, run:

```bash
python -m dimos.robot.galaxea.r1pro.demo_packing_stack \
  --artifact "$PWD/recordings/r1pro-act-task/policy-packing-augmented" \
  --output "$PWD/recordings/r1pro-act-task/my-five-bottle-delivery" \
  --zenoh-scout-addr 224.0.0.224:19467 \
  --scene-package /home/mustafa/dimos/data/scene_packages/hssd_102344115 \
  --deliver-to-laptop --seed 5000 --stay-open
```

ACT first packs all five bottles using the same checkpoint. Once it stops,
`tray_manipulation` coordinates both hands to grasp and lift the tray, while
`base_transport` executes a collision-checked route. The arms lower the tray onto
the actual tabletop beside the laptop, release it only after surface contact,
and retreat. These are ControlCoordinator trajectories; **no new ACT training**
is required. The tray and bottles remain physically free throughout the trip.
Allow about five minutes including startup. The native view switches to an
elevated angle after pickup to see over cabinets; you can still orbit and zoom.

The full trip starts only after all five placements pass. Route planning includes
the robot, tray and every bottle. Runtime checks stop motion if a bottle tips or
leaves the tray, grip is lost, or an obstacle is contacted. An obstructed route
raises an error. The output includes separate `packing_success`, `delivery.success`
and overall `success`; check the overall result for an end-to-end run.

## What is learned

A geometry planner selects an accessible bottle and an empty tray slot. ACT gets
the head and wrist RGB images, 20 measured joint positions, and eight simulator
goal values: source XYZ, destination XYZ, radius and half-height. It produces
joint commands for grasping, placing, releasing and returning home, executed by
the ControlCoordinator `policy_rollout` trajectory task. The model uses profile
`r1pro-sim-bottle-packing-v1`; the earlier `policy-free-tray` is incompatible.

The default source order works from left to right while clearing blocked rear
bottles. `--random-order` varies accessible choices for robustness testing.
Placements fill rows and reserve space for opening the gripper. When no safe slot
remains, the planner reports `tray_full` and stops. It does not rearrange bottles
or claim optimal packing. A tilted bottle gets a larger conservative footprint.

Five original bottles (5 cm diameter, 14 cm tall) and the 21 cm square tray rest
on the worktop. A local MJCF overlay supplies them; the house package is unchanged.
This checkpoint covers matching bottles in a small workspace with ±3 mm source
variation. Mixed shapes, wider layouts, arbitrary objects, and learned tray
handling remain future work. Simulator geometry supplies goals; this demo does
not establish perception or grasp generalization to unseen items.

## Validation

Success requires bilateral finger contact, a real lift, upright placement within
15 degrees, full containment, release, settling, and an open gripper back at
home. ACT stops at completion, then physics is checked again while the controller
holds its last command. All five bottles must still pass at the end. Neither
native nor offline evaluation uses scripted motion fallback or attachments or
teleportation after reset.

- Teacher: 20/20 complete scenes and 100/100 picks, seeds 8200–8219. This validates
  demonstrations, not learned performance.
- Training dataset: 130 unique picks, with 120 training episodes and 10 held-out
  picks from two complete scenes. The 20,000-update continuation took 58 minutes;
  its best default-order offline test completed 3/5 scenes.
- Small image augmentation added 5,000 fine-tuning updates on the same dataset,
  about 15 minutes. The first fresh offline test completed **3/3 scenes and
  15/15 bottles**, seeds 9700–9702.
- Extended default-order offline check: **10/10 complete scenes**, all 50 bottles,
  seeds 9800–9809. Combined with the initial check, this is 13/13 scenes.
- Native augmented checkpoint: **6/6 complete runs**, all 30 bottles, seeds 9700,
  5000, 9800, 9801, 5001 and 5002, with clean cancellation and shutdown. The last
  run also verified the final display angle and captured the completed tray.
- Randomized accessible order: **5/5 complete scenes**, all 25 bottles, seeds
  9900–9904. These are five different source orders. Across the three offline
  evaluations, all **18/18 scenes and 90/90 bottles** passed.

The optional full tray delivery completed **two native end-to-end runs**, seeds
5000 and 5001, with all ten bottle deliveries successful. Both runs retained
four handle contacts and upright cargo throughout transport, then physically
released the tray on the laptop tabletop. The final placement error was below
5 mm in each run. The first development attempt stopped during the first ACT
pick on an observation synchronization error, before tray handling; the timing
guard remains unchanged. These two successes are repeat checks, not a broad
reliability estimate. Exact results and screenshots are in
`jobs/packing-delivery-5001/validation-summary.json`.

These tests cover the matching-bottle setup and small position changes described
above; they do not establish reliability for new shapes or larger rearrangements.

The cameras now render timestamped snapshots outside the physics lock. Full
native display is retained, and measured physics/control timing is approximately
real time. Camera initialization finishes before viewer creation to avoid the
GLFW X11 startup race. The initial viewer angle now faces the table from an
unobstructed side, showing the robot and tray. Policy camera views are unchanged.
Shutdown retains ownership of a blocked renderer and
rejects reconnect until it has actually stopped. Other stacks retain the default
of rendering on the simulation thread.

## Artifacts and background jobs

All paths below are under `recordings/r1pro-act-task` in this worktree:

| Path | Contents |
| --- | --- |
| `policy-packing-augmented` | Deployment checkpoint for the command above |
| `dataset-packing-full-conditioned` | Training and held-out demonstration episodes |
| `train-packing-augmented` | Fine-tuning configuration and checkpoints |
| `eval-packing-augmented-canonical/result.json` | Initial three-scene offline evaluation |
| `native-packing-resume-augmented-9700/result.json` | First complete native run |
| `native-packing-validated-5000/result.json` | Native run matching the documented seed |
| `eval-packing-augmented-extended/result.json` | Ten default-order offline trials |
| `eval-packing-augmented-random/result.json` | Five randomized-order offline trials |
| `jobs/packing-resume-validation` | Completed repeat-validation commands and logs |
| `jobs/packing-display-verified/native-final.png` | Captured native window after all five placements |
| `jobs/packing-display-verified/validation-summary.json` | Packing-only validation summary |
| `native-packing-delivery-5000-retry/result.json` | First full five-bottle tray delivery |
| `native-packing-delivery-5001/result.json` | Repeat delivery with elevated native view |
| `jobs/packing-delivery-5001/native-final.png` | Five bottles delivered beside the laptop |
| `jobs/packing-delivery-5001/validation-summary.json` | End-to-end measurements and earlier failure |

Each job saves its exact `run.sh`, `pid`, `stage`, log and final `exit-code`.
Jobs use independent sessions and survive terminal disconnection. A zero exit
code from the demo now means success; an unsuccessful completed rollout exits 1.
Closing with Ctrl-C exits 130 after saving result.json. Historical job wrappers
have their own exit behavior, so check physical success in result.json as well.
All fitting and validation jobs are complete, and their runtime processes have
exited. No further training is queued. Raw demonstrations, datasets and weights
are ignored local artifacts, separate from source commits. Keep them when moving
or cleaning this worktree.

```bash
cat recordings/r1pro-act-task/jobs/packing-resume-validation/stage
tail -f recordings/r1pro-act-task/jobs/packing-resume-validation/run.log
```
