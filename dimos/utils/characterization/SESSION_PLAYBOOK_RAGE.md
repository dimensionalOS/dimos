# Rage-Mode Characterization Session Playbook

Companion to `SESSION_PLAYBOOK.md`. Same recipes, same structure, but
adds `--rage` to every session-launch command so the GO2Connection
boots into rage mode (StandUp → BalanceStand → enable_rage_mode) and
the teleop module's WASD speeds are bumped to 1.25 m/s / 1.2 rad/s to
match the higher motion envelope.

**Run the default-mode playbook first** (`SESSION_PLAYBOOK.md`).
Default-mode data is the baseline; rage data is the comparison set.
The two should not be merged into one session — they share
`recording.db` and that file knows nothing about which mode was
active. Run rage as a separate sweep, on a separate day if possible.

**Battery**: rage is more energetic. Start at ≥ 95% SOC. Plan to swap
mid-session for sure.

**Space**: rage is faster. Repositioning takes more attention. Same
~5 m runway works for the recipes below; the robot just gets there
quicker.

---

## 0. Pre-flight (1 minute)

```
cd ~/Documents/repos/dimos
pkill -f python -m dimos.utils.characterization.scripts.run_session 2>/dev/null
git status

SURFACE=carpet
NOTES_TAG=RAGE     # appended to every --notes so rage data is searchable
```

Robot fully charged. The sanity step below uses the actual
characterization session path (with `obstacle_avoidance=False`, matching
all the recipe runs that follow), so a successful sanity confirms rage
activation *and* that the path you'll use for E1+ is working — no
separate teleop-blueprint test needed.

Note: ``unitree_go2_coordinator_rage_teleop`` (the standalone teleop
blueprint) has ``obstacle_avoidance=True``, intentional for human-driven
teleop. Our session bypasses that — see the verification below if you
want to confirm.

---

## 1. Sanity pass (3 minutes, 1 run)

```
python -m dimos.utils.characterization.scripts.run_session \
    --rage \
    --recipes "dimos.utils.characterization.examples:step_vx_1:1" \
    --surface $SURFACE --notes "${NOTES_TAG} sanity" \
    --out-dir ~/char_runs
```

After:

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
python -m dimos.utils.characterization.scripts.analyze_run $SESSION/000_*
xdg-open $SESSION/000_*/plot.svg
```

Confirm:
- ``session.json`` has ``"rage": true``
- ``run.json`` has ``bms_start.soc`` populated (real percentage, not null)
- The blueprint's ``obstacle_avoidance`` is False:

```
.venv/bin/python -c "
from dimos.utils.characterization.session import build_session_blueprint
import tempfile, pathlib
db = pathlib.Path(tempfile.mkdtemp()) / 'x.db'
bp = build_session_blueprint(db, backend='go2', rage=True, include_teleop=False)
print('obstacle_avoidance:', bp.global_config_overrides.get('obstacle_avoidance'))
"
```

**Stop here if anything looks wrong.** Rage failures are typically
WebRTC-handshake-time, not run-time, so a clean sanity pass means the
rest of the session will work.

---

## 2. E1 — vx step in RAGE (~17 minutes, 40 runs)

Same amplitudes as default-mode E1 so the two datasets line up
amplitude-by-amplitude for comparison. If a 1.5 m/s step looks far
from saturation in rage (which it probably will), we'll add a rage-only
extension at higher amplitudes after.

```
python -m dimos.utils.characterization.scripts.run_session \
    --rage \
    --randomize --rng-seed 1 \
    --surface $SURFACE --notes "${NOTES_TAG} E1 vx step matrix, 5 reps" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e1_vx_pos_0p3:5,dimos.utils.characterization.experiments:e1_vx_neg_0p3:5,dimos.utils.characterization.experiments:e1_vx_pos_0p6:5,dimos.utils.characterization.experiments:e1_vx_neg_0p6:5,dimos.utils.characterization.experiments:e1_vx_pos_1p0:5,dimos.utils.characterization.experiments:e1_vx_neg_1p0:5,dimos.utils.characterization.experiments:e1_vx_pos_1p5:5,dimos.utils.characterization.experiments:e1_vx_neg_1p5:5"
```

After:

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done

for amp in _0.3 -0.3 _0.6 -0.6 _1.0 -1.0 _1.5 -1.5; do
  python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e1_vx_${amp}_* \
      --out $SESSION/compare_e1_vx_${amp}.svg 2>/dev/null
done
ls $SESSION/compare_e1_vx_*.svg
xdg-open $SESSION/compare_e1_vx__1.0.svg
```

What to expect compared to default-mode E1: faster rise time, possibly
higher steady-state gain (less dropout under load), and saturation
likely well above 1.5 m/s — meaning your 1.5 m/s commanded step still
sits in the linear region, unlike default mode where it might be at
the edge.

---

## 3. E3 — vx ramp in RAGE (~5 minutes, 5 runs)

Same recipe as default-mode E3. The plant response will differ; the
recipe input is identical so they're directly comparable.

```
python -m dimos.utils.characterization.scripts.run_session \
    --rage \
    --recipes "dimos.utils.characterization.experiments:e3_vx_ramp_0_to_1p2_5m:5" \
    --surface $SURFACE --notes "${NOTES_TAG} E3 vx ramp x5 (5m runway)" \
    --out-dir ~/char_runs
```

After:

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done
python -m dimos.utils.characterization.scripts.compare_runs $SESSION/0* --out $SESSION/compare_e3_rage.svg
xdg-open $SESSION/compare_e3_rage.svg
```

Expectation: parametric (cmd vs meas) curve still linear up to 1.2
m/s. Saturation should remain well above this in rage. If you have a
bigger space later, run `e3_vx_ramp_0_to_3` to actually find the rage
saturation point.

---

## 4. E2 — wz step in RAGE (~17 minutes, 40 runs)

```
python -m dimos.utils.characterization.scripts.run_session \
    --rage \
    --randomize --rng-seed 2 \
    --surface $SURFACE --notes "${NOTES_TAG} E2 wz step matrix, 5 reps" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e2_wz_pos_0p3:5,dimos.utils.characterization.experiments:e2_wz_neg_0p3:5,dimos.utils.characterization.experiments:e2_wz_pos_0p6:5,dimos.utils.characterization.experiments:e2_wz_neg_0p6:5,dimos.utils.characterization.experiments:e2_wz_pos_1p0:5,dimos.utils.characterization.experiments:e2_wz_neg_1p0:5,dimos.utils.characterization.experiments:e2_wz_pos_1p5:5,dimos.utils.characterization.experiments:e2_wz_neg_1p5:5"
```

After:

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

After E1 + E2 you've done ~80 runs in rage — battery drain higher
than default mode. Check SOC:

```
.venv/bin/python -c "
import json, glob
runs = sorted(glob.glob(f'$SESSION/0*/run.json'))
last = json.load(open(runs[-1]))
end = last.get('bms_end') or {}
print(f'last run end SOC: {end.get(\"soc\")}%')
"
```

If < 50%, swap battery before continuing. Note the swap in your
external session log.

---

## 5. E4 — wz ramp in RAGE (~5 minutes, 5 runs, in place)

```
python -m dimos.utils.characterization.scripts.run_session \
    --rage \
    --recipes "dimos.utils.characterization.experiments:e4_wz_ramp_0_to_4:5" \
    --surface $SURFACE --notes "${NOTES_TAG} E4 wz ramp x5" \
    --out-dir ~/char_runs
```

After:

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done
python -m dimos.utils.characterization.scripts.compare_runs $SESSION/0* --out $SESSION/compare_e4_rage.svg
xdg-open $SESSION/compare_e4_rage.svg
```

Rage may push max wz higher than 4 rad/s (the recipe's target). If
the parametric plot looks linear all the way to 4, queue a follow-up
recipe to ramp to 6 rad/s.

---

## 6. E7a — pure wz cross-coupling in RAGE (~8 minutes, 18 runs)

```
python -m dimos.utils.characterization.scripts.run_session \
    --rage \
    --randomize --rng-seed 71 \
    --surface $SURFACE --notes "${NOTES_TAG} E7a pure-wz cross-coupling" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e7a_wz_pos_0p3:3,dimos.utils.characterization.experiments:e7a_wz_neg_0p3:3,dimos.utils.characterization.experiments:e7a_wz_pos_0p8:3,dimos.utils.characterization.experiments:e7a_wz_neg_0p8:3,dimos.utils.characterization.experiments:e7a_wz_pos_1p5:3,dimos.utils.characterization.experiments:e7a_wz_neg_1p5:3"
```

After:

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done

for amp in _0.3 -0.3 _0.8 -0.8 _1.5 -1.5; do
  python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e7a_wz_${amp}_* \
      --out $SESSION/compare_e7a_${amp}.svg 2>/dev/null
done
ls $SESSION/compare_e7a_*.svg
```

---

## 7. E7b — pure vx cross-coupling in RAGE (~10 minutes, 18 runs)

Robot moves faster — pay attention to space, especially on the +1.0
amplitude (5 m / 5 s — same in rage, same travel).

```
python -m dimos.utils.characterization.scripts.run_session \
    --rage \
    --randomize --rng-seed 72 \
    --surface $SURFACE --notes "${NOTES_TAG} E7b pure-vx cross-coupling" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e7b_vx_pos_0p3:3,dimos.utils.characterization.experiments:e7b_vx_neg_0p3:3,dimos.utils.characterization.experiments:e7b_vx_pos_0p6:3,dimos.utils.characterization.experiments:e7b_vx_neg_0p6:3,dimos.utils.characterization.experiments:e7b_vx_pos_1p0:3,dimos.utils.characterization.experiments:e7b_vx_neg_1p0:3"
```

After:

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

## 8. E8 — deadtime precision in RAGE (~10 minutes, 30 runs)

```
python -m dimos.utils.characterization.scripts.run_session \
    --rage \
    --randomize --rng-seed 8 \
    --surface $SURFACE --notes "${NOTES_TAG} E8 deadtime 30 reps alternating" \
    --out-dir ~/char_runs \
    --recipes "dimos.utils.characterization.experiments:e8_vx_short_step:15,dimos.utils.characterization.experiments:e8_vx_short_step_neg:15"
```

After:

```
SESSION=$(ls -td ~/char_runs/session_*/ | head -1); SESSION=${SESSION%/}
for d in $SESSION/0*; do python -m dimos.utils.characterization.scripts.analyze_run "$d"; done
python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e8_vx_short_step_r* --out $SESSION/compare_e8_pos.svg 2>/dev/null
python -m dimos.utils.characterization.scripts.compare_runs $SESSION/*e8_vx_short_step_neg_r* --out $SESSION/compare_e8_neg.svg 2>/dev/null
```

Expectation: deadtime in rage should be similar or slightly less than
default mode (faster locomotion policy reaction). Jitter
characteristics likely similar (WebRTC + onboard policy structure
unchanged).

---

## 9. Backup (1 minute)

Tag the rage data so it doesn't blend with the default-mode data:

```
TODAY=$(date +%Y-%m-%d)
mkdir -p ~/char_data/${TODAY}_${SURFACE}_rage
cp -r ~/char_runs/session_* ~/char_data/${TODAY}_${SURFACE}_rage/
ls ~/char_data/${TODAY}_${SURFACE}_rage/
```

(Note: this picks up *all* recent sessions. If you ran default-mode
runs earlier today, move them out first or filter by `notes`.)

---

## 10. Per-session inventory

```
echo "rage session counts:"
for s in ~/char_data/${TODAY}_${SURFACE}_rage/session_*/; do
    n=$(ls $s | grep -E '^[0-9]' | wc -l)
    notes=$(.venv/bin/python -c "import json; print(json.load(open('${s}session.json'))['operator']['notes'])" 2>/dev/null)
    rage=$(.venv/bin/python -c "import json; print(json.load(open('${s}session.json')).get('rage', False))" 2>/dev/null)
    echo "  $(basename $s): $n runs  rage=$rage  notes='$notes'"
done
```

Every line should show `rage=True`. If any row shows `rage=False`,
that session was run without `--rage` and shouldn't be in this dir —
move it back to the default-mode pile.

---

## End-of-session checklist

- [ ] All 7 rage sessions completed; `exit_reason: ok` for all runs.
- [ ] Every `session.json` has `"rage": true`.
- [ ] Battery swap(s) noted in external session log.
- [ ] Sessions copied to `~/char_data/<date>_<surface>_rage/`.
- [ ] Eyeballed at least one `compare_*.svg` per session.
- [ ] Anomalies / observations logged separately (`notes_rage.md`).
- [ ] Default-mode dataset still intact in
      `~/char_data/<date>_<surface>/` (unchanged by today's rage work).

---

## Comparing rage vs default-mode after both are collected

The most informative single chart is amplitude-matched cmd/meas
overlay. Run both compare commands and put the SVGs side-by-side:

```
DEFAULT=~/char_data/<date>_<surface>/<E1-session>
RAGE=~/char_data/<date>_<surface>_rage/<E1-session>

# Open the +1.0 amplitude compare from each
xdg-open $DEFAULT/compare_e1_vx__1.0.svg
xdg-open $RAGE/compare_e1_vx__1.0.svg
```

What to look for:
- Steady-state level: rage commonly tracks closer to commanded.
- Rise time: rage typically shorter.
- Overshoot: rage can show more, especially at high amplitudes.
- Cross-coupling: shouldn't change much (same kinematic platform);
  if it does, that's interesting and worth a deeper look.

A proper rage-vs-default analysis tool would aggregate the metrics
across all 5 repeats of each amplitude and emit a side-by-side
parameter table. That's a Rung-2 deliverable, not Rung 1 — but the
data this session produces is what feeds it.
