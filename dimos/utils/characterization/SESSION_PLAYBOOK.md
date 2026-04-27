# Full Characterization Session Playbook

Step-by-step copy-paste-able commands for a single ~75 minute data
collection session covering E1, E2, E3, E4, E7a, E7b, E8.

You manually approve each run from the terminal prompt and use WASD on
the pygame teleop window to reposition the robot between runs.

**Battery budget**: start ≥ 90% SOC. Plan to check SOC after E2 and
swap if it drops below 50%. The harness records BMS automatically into
each `run.json`.

**Surface**: pick one and stick with it the whole session — `--surface`
is a free-form metadata string but consistency across runs is what
makes the data comparable later. Examples: `carpet-low`, `vinyl`,
`concrete`, `hardwood`. The commands below use `carpet`; change it once
at the top.

---

## 0. Pre-flight (1 minute)

```
cd ~/Documents/repos/dimos
pkill -f python -m dimos.utils.characterization.scripts.run_session 2>/dev/null   # nothing stale running
pkill -f mujoco 2>/dev/null
git status                                 # check what branch you're on
```

Fully charge the robot. Clear ~5 m of forward floor space, ~3 m
sideways. Make sure your laptop has the pygame teleop window in focus
when you're driving the robot — keys only register when the pygame
window has focus.

Set the surface tag once, used in every command below:

```
SURFACE=carpet
```

---

## 1. Sanity pass (3 minutes)

One run, just to confirm everything works on real hardware before
committing to a long session. Confirms:

- WebRTC connects, odom flows
- BMS captured (`bms_start` is a real number)
- Plot renders with non-zero measured trace

```
python -m dimos.utils.characterization.scripts.run_session \
    --recipes "dimos.utils.characterization.examples:step_vx_1:1" \
    --surface $SURFACE --notes "morning sanity" \
    --out-dir ~/char_runs
```

After it finishes, look at the plot:

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
python -m dimos.utils.characterization.scripts.analyze_run $SESSION/000_*
xdg-open $SESSION/000_*/plot.svg
```

**Stop here if anything looks wrong** — investigate before burning a
session's worth of robot time on bad data.

---

## 2. E1 — vx step response matrix (~17 minutes, 40 runs)

What this is measuring: the linear-velocity dynamics of the Go2 plant
(WebRTC + onboard locomotion policy). Steady-state gain at 4
amplitudes × 2 directions, with 5 repeats per condition for noise
estimation.

```
python -m dimos.utils.characterization.scripts.run_session \
    --randomize --rng-seed 1 \
    --surface $SURFACE --notes "E1 vx step matrix, 5 reps" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e1_vx_pos_0p3:5,dimos.utils.characterization.experiments:e1_vx_neg_0p3:5,dimos.utils.characterization.experiments:e1_vx_pos_0p6:5,dimos.utils.characterization.experiments:e1_vx_neg_0p6:5,dimos.utils.characterization.experiments:e1_vx_pos_1p0:5,dimos.utils.characterization.experiments:e1_vx_neg_1p0:5,dimos.utils.characterization.experiments:e1_vx_pos_1p5:5,dimos.utils.characterization.experiments:e1_vx_neg_1p5:5"
```

What the syntax does:

- `--randomize --rng-seed 1` — shuffles the order of all 40 runs so
  amplitude doesn't correlate with battery drift over the session.
  The seed makes the order reproducible if you ever rerun.
- `--recipes "<a:n>,<b:n>,..."` — comma-separated `module:recipe:repeats`.
  Each entry says "run this recipe N times". The session expands to a
  flat plan of 40 runs and shuffles it.
- `--randomize` will alternate ± directions naturally, which keeps the
  robot from walking out of the test space.

**During the session:**

1. Pre-session park prompt — drive robot to your start mark with WASD,
   release keys, ENTER.
2. For each of 40 runs:
   - Header line shows recipe + duration.
   - Drive robot back to start mark with WASD.
   - **Release keys**, ENTER to fire the recipe.
   - Robot runs the 4.5 s step, returns sample count + exit reason.
3. Post-session park prompt — drive robot home, ENTER to shut down.

**After:**

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
echo "$SESSION"
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done

# Note: '+' in recipe names is sanitized to '_' in dir names, so the
# positive amplitudes match with double-underscore (e.g. e1_vx__1.0).
for amp in _0.3 -0.3 _0.6 -0.6 _1.0 -1.0 _1.5 -1.5; do
  python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e1_vx_${amp}_* \
      --out $SESSION/compare_e1_vx_${amp}.svg 2>/dev/null
done

# List what got generated, then open one
ls $SESSION/compare_e1_vx_*.svg
xdg-open $SESSION/compare_e1_vx__1.0.svg
```

The comparison plots overlay the 5 repeats per amplitude — you want
the measured traces to sit on top of each other (low run-to-run noise).

---

## 3. E3 — vx saturation ramp (~5 minutes, 5 runs)

Sweeps vx from 0 to 1.2 m/s over 7 s. Reveals the gain curve — should
be roughly linear up to wherever the Go2 saturates.

```
python -m dimos.utils.characterization.scripts.run_session \
    --recipes "dimos.utils.characterization.experiments:e3_vx_ramp_0_to_1p2_5m:5" \
    --surface $SURFACE --notes "E3 vx ramp x5 (5m runway recipe)" \
    --out-dir ~/char_runs
```

**During the session:** position robot at one end of your 5 m runway,
facing the other end. Each run travels ~4.2 m + braking distance.

**After:**

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done
python -m dimos.utils.characterization.scripts.compare_runs $SESSION/0* --out $SESSION/compare_e3.svg
xdg-open $SESSION/compare_e3.svg
```

Look for: did the measured trace flatten anywhere? That's the
saturation point. If it stayed linear up to 1.2 m/s, saturation is
above this range and we'd need a longer runway to find it.

---

## 4. E2 — wz step response matrix (~17 minutes, 40 runs)

Same shape as E1, but for angular velocity. In place — no translation.

```
python -m dimos.utils.characterization.scripts.run_session \
    --randomize --rng-seed 2 \
    --surface $SURFACE --notes "E2 wz step matrix, 5 reps" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e2_wz_pos_0p3:5,dimos.utils.characterization.experiments:e2_wz_neg_0p3:5,dimos.utils.characterization.experiments:e2_wz_pos_0p6:5,dimos.utils.characterization.experiments:e2_wz_neg_0p6:5,dimos.utils.characterization.experiments:e2_wz_pos_1p0:5,dimos.utils.characterization.experiments:e2_wz_neg_1p0:5,dimos.utils.characterization.experiments:e2_wz_pos_1p5:5,dimos.utils.characterization.experiments:e2_wz_neg_1p5:5"
```

**During the session:** the robot rotates in place. Reposition
between runs only if it has drifted out of place.

**After:**

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done

for amp in _0.3 -0.3 _0.6 -0.6 _1.0 -1.0 _1.5 -1.5; do
  python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e2_wz_${amp}_* \
      --out $SESSION/compare_e2_wz_${amp}.svg 2>/dev/null
done
ls $SESSION/compare_e2_wz_*.svg
xdg-open $SESSION/compare_e2_wz__1.0.svg
```

---

## ☕ Battery check halftime

After E1 + E2 you've done 80+ runs. Open any recent `run.json` and
check SOC:

```
.venv/bin/python -c "
import json, glob
last = sorted(glob.glob(f'$SESSION/0*/run.json'))[-1]
d = json.load(open(last))
print(f'last SOC: {d[\"bms_end\"][\"soc\"]}%')
"
```

If SOC < 50%, swap to a fresh battery and **note the swap** in your
external session log. Battery state changes the plant gain — runs
before and after a swap aren't strictly comparable.

---

## 5. E4 — wz saturation ramp (~5 minutes, 5 runs, in place)

In-place wz ramp 0 → 4 rad/s over 15 s. Should reveal max yaw rate.

```
python -m dimos.utils.characterization.scripts.run_session \
    --recipes "dimos.utils.characterization.experiments:e4_wz_ramp_0_to_4:5" \
    --surface $SURFACE --notes "E4 wz ramp x5" \
    --out-dir ~/char_runs
```

**After:**

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done
python -m dimos.utils.characterization.scripts.compare_runs $SESSION/0* --out $SESSION/compare_e4.svg
xdg-open $SESSION/compare_e4.svg
```

---

## 6. E7a — pure wz, measure vx leak (~8 minutes, 18 runs, in place)

Cross-coupling test: command pure rotation, see how much the robot
unintentionally drifts forward/sideways. **Pure rotation, no
translation in the commanded signal**, but measured x/y/yaw drift
tells you if the channels are coupled.

```
python -m dimos.utils.characterization.scripts.run_session \
    --randomize --rng-seed 71 \
    --surface $SURFACE --notes "E7a pure-wz cross-coupling matrix" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e7a_wz_pos_0p3:3,dimos.utils.characterization.experiments:e7a_wz_neg_0p3:3,dimos.utils.characterization.experiments:e7a_wz_pos_0p8:3,dimos.utils.characterization.experiments:e7a_wz_neg_0p8:3,dimos.utils.characterization.experiments:e7a_wz_pos_1p5:3,dimos.utils.characterization.experiments:e7a_wz_neg_1p5:3"
```

**After:**

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done

for amp in _0.3 -0.3 _0.8 -0.8 _1.5 -1.5; do
  python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e7a_wz_${amp}_* \
      --out $SESSION/compare_e7a_${amp}.svg 2>/dev/null
done
ls $SESSION/compare_e7a_*.svg
```

Look at any `compare_e7a_*.svg`. The blue dashed lines (commanded wz)
should be flat at the amplitude. The red lines (measured wz) should
match. The interesting question is what `meas_vx` and `meas_vy`
look like — but the standard plot shows the dominant channel. To see
the leak channel, eyeball `meas_x` / `meas_y` in the run's `run.json`
or open the recording.db directly (see "Spot-check raw data" below).

---

## 7. E7b — pure vx, measure wz leak (~10 minutes, 18 runs)

Forward motion at fixed speed for 5 s; measure how much the robot
yawed during the forward drive. Caps at 1.0 m/s × 5 s = 5 m travel.

```
python -m dimos.utils.characterization.scripts.run_session \
    --randomize --rng-seed 72 \
    --surface $SURFACE --notes "E7b pure-vx cross-coupling matrix" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e7b_vx_pos_0p3:3,dimos.utils.characterization.experiments:e7b_vx_neg_0p3:3,dimos.utils.characterization.experiments:e7b_vx_pos_0p6:3,dimos.utils.characterization.experiments:e7b_vx_neg_0p6:3,dimos.utils.characterization.experiments:e7b_vx_pos_1p0:3,dimos.utils.characterization.experiments:e7b_vx_neg_1p0:3"
```

**During the session:** randomization alternates ± directions, so the
robot will go forward then backward. Reposition to start mark
between runs (~5 m each way).

**After:**

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done

for amp in _0.3 -0.3 _0.6 -0.6 _1.0 -1.0; do
  python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e7b_vx_${amp}_* \
      --out $SESSION/compare_e7b_${amp}.svg 2>/dev/null
done
ls $SESSION/compare_e7b_*.svg
```

---

## 8. E8 — deadtime precision (~10 minutes, 30 runs)

Sharp 1.0 m/s vx step, 0.5 s hold, alternating forward/back so the
robot stays in place. Many repeats give a deadtime distribution
(mean, p95, jitter). **Most actionable single number for controller
tuning.**

```
python -m dimos.utils.characterization.scripts.run_session \
    --randomize --rng-seed 8 \
    --surface $SURFACE --notes "E8 deadtime 30 reps alternating" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e8_vx_short_step:15,dimos.utils.characterization.experiments:e8_vx_short_step_neg:15"
```

**During the session:** each run only travels ~0.5 m, so repositioning
is minimal. Robot will tend to drift, occasionally drive it back to
center with WASD.

**After:**

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done
python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e8_vx_short_step_r* --out $SESSION/compare_e8_pos.svg 2>/dev/null
python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e8_vx_short_step_neg_r* --out $SESSION/compare_e8_neg.svg 2>/dev/null
```

The standard plots show rise/settle at recipe level. Aggregating the
30 deadtimes into a histogram is a follow-up analysis (a
`python -m dimos.utils.characterization.scripts.process_session deadtime` script — not shipped yet; the per-run
`metrics.json` files have the numbers).

---

## 9. Backup the data (1 minute)

**Critical**: `~/char_runs/` may live on temp storage or disk you'll
clean. Copy the day's sessions to a persistent location with a labeled
parent dir.

```
TODAY=$(date +%Y-%m-%d)
mkdir -p ~/char_data/${TODAY}_${SURFACE}
cp -r ~/char_runs/session_* ~/char_data/${TODAY}_${SURFACE}/
ls ~/char_data/${TODAY}_${SURFACE}/
```

You should see 7 session subdirs (one per experiment family).

---

## 10. Per-session inventory and total run count

```
echo "session counts:"
for s in ~/char_data/${TODAY}_${SURFACE}/session_*/; do
    n=$(ls $s | grep -E '^[0-9]' | wc -l)
    notes=$(.venv/bin/python -c "import json; print(json.load(open('${s}session.json'))['operator']['notes'])" 2>/dev/null)
    echo "  $(basename $s): $n runs  notes='$notes'"
done
```

Expected: ~156 runs across 7 sessions.

---

## 11. Spot-check raw data (optional, anytime)

If a plot looks weird or you want to see the actual numbers:

```
SESSION=~/char_data/${TODAY}_${SURFACE}/session_<the-one-you-want>
RUN=$(ls -d $SESSION/0* | head -1)

# Run-level metadata
cat $RUN/run.json | head -40

# First/last commanded samples (the runner's authoritative cmd log)
head -5 $RUN/cmd_monotonic.jsonl
tail -5 $RUN/cmd_monotonic.jsonl

# Measured samples count, decoded via memory2
.venv/bin/python -c "
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
import json
meta = json.load(open('$RUN/run.json'))
win = meta['ts_window_wall']
s = SqliteStore(path='$SESSION/recording.db'); s.start()
try:
    obs = [o for o in s.stream('measured', PoseStamped).to_list() if win['start']<=o.ts<=win['end']]
    print(f'measured samples in run: {len(obs)}')
    if obs:
        for o in obs[:3]:
            print(f'  ts={o.ts:.3f} x={o.data.x:+.3f} y={o.data.y:+.3f} yaw={o.data.yaw:+.3f}')
finally:
    s.stop()
"
```

---

## End-of-session checklist

- [ ] All 7 sessions completed; final session `exit_reason: ok` for all runs.
- [ ] Battery SOC drop tracked; if a battery swap happened, log when.
- [ ] All sessions copied to `~/char_data/<date>_<surface>/`.
- [ ] Eyeballed at least one `compare_*.svg` per session — the
      measured traces overlap each other (low noise) and aren't all
      flat (real signal).
- [ ] Wrote a paragraph somewhere (separate `notes.md`) about anything
      anomalous: weird drifts, surface inconsistencies, key teleop
      events, robot misbehavior.

You're now ready for data processing (Rung 2: FOPDT modeling, Bode
estimation, controller-budget calculations). The harness's job is
done; from here it's pandas / scipy on the parquet... uh, sqlite.
