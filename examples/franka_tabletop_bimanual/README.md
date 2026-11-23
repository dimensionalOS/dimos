# Tabletop bimanual FR3: skills, calibration and notes

Scripts for two Franka Research 3 arms clamped to one table and driven by dimOS's
`fr3-tabletop-bimanual` blueprint (see
`docs/capabilities/manipulation/franka_integration.md`). They attach to a running
stack with `Dimos.connect()`, so start the stack first:

```bash
dimos run fr3-tabletop-bimanual \
    --right_arm.hold_on_short_end true --left_arm.hold_on_short_end true
cd examples/franka_tabletop_bimanual
python handoff.py recover        # both arms to ready
python handoff.py cycle          # find an object, pick, hand over, place
```

Run every script from this folder: they import each other and read their calibration
files next to them.

## What is here

| File | What it does |
|---|---|
| `tabletop.py` | Finds whatever stands on the table with no object model: a table-plane fit per camera, points above it grouped into boxes, plus a colour fallback for flat things. Also re-measures the tripod camera from the taped 4x3 ChArUco board on every run. |
| `handoff.py` | **Handover skill**: look, pick with the nearer arm, search a handover pose both arms reach and the separation guard allows, take, place on the other side. Thin or short objects relay through the table. Stages: `look plan pick present take place cycle loop recover`. |
| `pen_demo.py` | **Pen skill**: one arm picks a click pen and holds it upright, the other clicks it (aimed by its wrist camera), the first draws a smiley on paper, clicks off and puts the pen down. `lower_pen.py` sets the drawing height by eye. |
| `marker_demo.py` | An earlier marker uncap-and-draw attempt (the cap was too tight); its perception and force helpers are reused by `pen_demo.py`. |
| `handeye.py` | Wrist-camera hand-eye calibration (`--arm 2` for the left arm). |
| `bimanual_cal.py` | Arm-to-arm calibration (planar fit) and the tripod, from two taped boards. |
| `remeasure_marker_board.py` | Re-measures the tripod's reference board after it is moved. |
| `checks.py`, `height_check.py`, `hover_check.py`, `soak.py` | Checks used while bringing the setup up (`live_test.py` holds helpers they and `handeye.py` share). `view_cam.py` is a live camera viewer. |
| `dimensional_tabletop_calibration.json` | Dimensional's `TabletopCalibration`, the format to copy for another table (point `DIMOS_FR3_TABLETOP_CALIBRATION` at your own). |
| `*.json`, `*.npz`, `*.npy` | This setup's raw calibration data (re-measure after moving a base, a camera or the board). |
| `RUNBOOK.md` | Power-on to demo, and fault recovery. |
| `HOW_IT_WORKS.md` | The system and the code, file by file. |
| `failure_notes.txt` | Every hardware fault with time, message and circumstances, for Franka support. |
| `PRESENTER_NOTES.md` | Per-topic notes and likely questions. |

`hold_on_short_end` is needed on these two arms since 2026-11-23 (their robots end
many motions short of the target; see below); it is off by default.

## Status (2026-11-23)

- Handover: reliable for box-like objects up to 6.8 cm across, in both directions (about 85-105 s a cycle).
- Pen: runs end to end; the drawing height and click aim are tuned by eye and vary with how the pen sits in the grip.
- Hardware: both arms have intermittent FSoE (safety link) faults, and since 2026-11-23 the
  robots end many motions 1-10 deg short of their target. See `failure_notes.txt`.
