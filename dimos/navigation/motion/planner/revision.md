# Spec revision: lattice anchoring + motion-conditioned envelope

AGREED 2026-08-06 (measurement results folded in from
[envelope_results.md](envelope_results.md)). One revision, gold + candidate +
judge move together, one re-baseline, one autoresearch re-earn. Phase 1 =
spec + gold + referee; phase 2 = the rust candidate re-earns.

## Evidence

- One lidar return 5 m behind the robot flips the door route 13.27 m ↔ 1.96 m
  (door.zenoh t=7.09; pinned as strict xfail in `referee/test_grid_invariance.py`).
- Cause: both gold (`scenarios.se2_path`) and the rust candidate anchor their
  lattices at `min(obstacle bbox, goal) − pad` — sample positions are a
  continuous function of the far corner of the cloud.
- door2.zenoh: a doorway the trunk (0.31 m) walks through reads as a wall
  because the search carries the all-gait union footprint (0.50 m) + 0.05
  precision per side = 0.60 m minimum everywhere, regardless of travel
  direction.

## Changes

1. **Absolute-lattice origin.** Grids anchor to the world frame's own lattice
   (`floor(corner / period) · period`), odom frame on the robot, scenario
   frame in the referee. Obstacles appearing/vanishing can add content but
   can never move a sample position.
2. **Pitch snapped to the map's voxel size.** `fine = voxel / 2` (0.04 for the
   go2's 0.08), `cell = 3 · fine` (0.12, unchanged). Every voxel centre lands
   exactly on a fine sample: a voxel pattern reads the same clearance wherever
   it sits, and whole-voxel translation of a scene translates the answer
   bit-exactly. Voxel size is a config constant of the deployment (never
   sniffed from data); changing it is a new spec = new baseline.
3. **Per-heading envelope.** Feasibility uses the swept box for the edge's
   body-frame drift angle, not the all-gait union; arcs add a curvature
   inflation term. Rows measured in the fitted sim
   (`simulation/envelope.py --bake`) as max swept outline over the gait cycle
   at the executable governed band (stand + 0.35 + 0.50 — the sim cannot
   execute 0.20, see min_speed below). `precision` (0.05) is untouched — it
   is the measured follower tracking floor.

   Storage on `Embodiment` (frozen dataclass, plain floats — serializes into
   the rust config blob as-is):

   ```python
   # (deg, length, width, off_x, off_y); |angle| rows — 0 = nose-first,
   # 90 = strafe, 180 = reverse. off_y is stored for POSITIVE drift and
   # mirrored by sign at lookup (the swept box lags the drift laterally by
   # up to 57 mm; a blind fold burns 15-56 mm of width). EMPTY = the union
   # length/width/center_off applies at every heading (today's behavior,
   # and the fallback for any unmeasured embodiment).
   envelope: tuple[tuple[float, float, float, float, float], ...] = ()
   # extra width per rad-per-metre of curvature (edge dyaw / edge length):
   # measured 0.0334, residuals <= 12 mm. Curvature, not per-edge yaw, so
   # the number survives lattice pitch changes unmeasured.
   arc_inflate: float = 0.0

   def envelope_at(self, drift: float) -> tuple[float, float, float, float]: ...
   def offsets(self, step=0.05, drift: float | None = None) -> np.ndarray: ...
   ```

   - **The union re-baselines to the honest 0.883 × 0.593** (the recorded
     0.852 × 0.495 was the max over a smaller command sweep — fast strafe and
     slow tight arcs were outside it). The union's jobs — judge veto,
     `half_diag`, fallback for unmeasured embodiments, body carve — are
     exactly where honest-conservative is the only acceptable property. The
     measured rows are the only non-conservative path.
   - Rows sit at the lattice's own drift angles (0, 26.6, 45, 63.4, 90 +
     reverse family) — nearest-row lookup is exact for every edge the search
     generates, no interpolation semantics.
   - No speed column: baked into the rows at measurement time (governor
     section below).
   - **min_speed (0.2) is suspect but unproven**: the SIM marches in place at
     0.2 (wider than walking at most headings), but the sim also undertracks
     slow commands vs the field (gain 0.62 vs 0.80 at 0.35). Verify on the
     real robot (command 0.2, run the tracking pass + field.py gain) before
     raising min_speed to 0.35 — a strictly-dominated crawl is a follower
     change that rides this same re-baseline if confirmed. The sim's low-speed
     undershoot is a tuning item of its own now: `simulation/FINDINGS.md` §E.

4. **Governor-time pricing in the gold cost** (adopted after phase 1.5; gold
   side only until the candidate re-earns in phase 2). An edge's tightness
   multiplier is `MAX_SPEED / governor_speed(clearance)` — what a metre there
   costs in time under the follower's own committed speed law, normalized so
   open space is 1.0 — and no longer a comfort ramp. Three reasons:

   - Planner and follower optimize the **same clock**. The follower is
     contractually slowed to `governor(clearance)` in tight places; a planner
     that priced tightness by a separate preference was choosing detours
     against a cost the robot never pays.
   - `comfort` **leaves the cost entirely**. It stays a labelling radius and
     the smoothing cap, and (with the spawn disk, same revision) it is finally
     a knob that can be turned without re-baselining the battery.
   - The charge **caps itself**: the governor floors at MIN_SPEED, so the
     multiplier tops out at MAX_SPEED/MIN_SPEED = 2.5x at contact — the same
     ceiling the comfort ramp had by hand, now derived. In between it is
     cheaper: 1.65x vs 1.92x at 0.15 m of clearance, 1.00x vs 1.19x at 0.35 m,
     and 2.50x vs 2.31x right at the precision floor, where fiction begins.

   Clearance is still read on the **union** (a preference has to be comparable
   across edges); feasibility stays per-heading. `control/profile.py` is the
   one copy of the curve, imported by the search, and its constants are in the
   gold cache key (`v11-governor-time`) — retuning the law may not serve a gold
   searched under the old one.

   *Measured, curated 16 + gen 40: doors and corridors thread more.
   `corridor` (0.9 m gap) stops detouring — 5.77 m around → 3.96 m straight
   through the middle, min truth clearance 0.242 m; `door_side` 5.55 → 3.85 m.
   `narrow_gap` (0.26 m opening) still goes around and `boxed_in` still
   refuses: pricing cannot reach what feasibility forbids. No veto flip, no
   label flip, no DQ on any of the 56 worlds; gold's pillar stays 1.0 and its
   gate goes 108.98 → 109.32 curated / 108.13 → 108.49 on the mixed roster.
   The rust candidate, which still prices comfort, drops 97.50 → 92.68 — the
   gap phase 2 exists to close.*

## Acceptance

- [x] `test_grid_invariance.py` xfail flips to passing (phase 2, when the rust
  candidate re-anchors); add whole-voxel-translation bit-exactness and
  far-point invariance over the full battery.
  *Flipped. Far-point invariance is bit-exact in the crate
  (`a_far_point_cannot_move_the_answer`) because the anchored lattice indexes
  absolutely — `k * FINE`, never `origin + i * FINE`. Whole-period translation
  is route-exact, not bit-exact, and cannot be: 0.24 is not a dyadic rational,
  so `(x + d) / FINE` and `x / FINE + d / FINE` differ in the last bit and a
  sample on a rounding boundary may tip.*
- [x] Field scenarios: door.zenoh and door2.zenoh (recorded worlds) route through
  the doorway with the forward envelope.
  *door.zenoh replays at 2.17 m mean published arc against 6.99 m before (the
  recorded run was 5.71 m), holds 14 -> 11; door2.zenoh 1.05 m against 2.24 m,
  holds unchanged at 9/9 agreeing. Both deterministic. Plan wall time falls,
  61.5 -> 54.1 ms/tick on door and 12.2 -> 11.5 on door2, so the finer pitch is
  paid for by the union-first fast path — but door is still 2.7x over the 20 ms
  budget, as it was before, and that is now the open item on this recording.*
- [x] Judge gains a per-mode envelope-violation metric (planner-assumes vs
  follower-does mismatch shows up named, not just as wall contact).
  *Phase 1.5: the planner judge sweeps the drift row and reports `env_viol`
  (union hits, the row does not); gold's 3 self-DQs became 3 named violations,
  6-27 mm. The control judge keeps the union as its pillar — it grades the
  follower, which is the thing that may leave the row — and reports the same
  metric as the share of ticks spent outside it.*
- [x] **Start witness** (adopted phase 1.5): a pose the robot actually occupies
  may always be departed. The seed's feasibility is read at the true start
  pose, not at the cell it snaps to.
  *`door_side` routes again (the snap cost it 0.083 -> 0.043 against a 0.05
  margin); a start with negative true clearance still refuses.*
- **Gold before/after review**: old vs new gold paths overlaid per world
  (curated 16 + gen 40) as a browsable artifact — Ivan reviews before phase 2.
- Referee re-baselined; runtime planner re-earns via the lab on the new spec.
  *Phase 2 landed the port, not the re-earn: curated 94.65 → 107.62 (gold
  pillar 0.842 → 0.976) and mixed gen40 92.85 → 102.22 (0.830 → 0.926) against
  frozen gates of 109.32 / 108.49. What is left is not port drift — the crate
  and `target-py` agree route-for-route on every offender (gen012 7.80 vs
  7.75 m, gen039 11.26 vs 11.15, gen007 10.91 vs 11.05) — it is the honest
  candidate's own handicap: gen012 and gen039 score ~10 because their cloud-built
  field will not thread a gap gold's box-exact SDF walks, and both scored ~10
  before this revision too. That is what the lab is for.*

## Amendment: standing is not the union (adopted 2026-08-07)

The envelope split feasibility per-heading but left every *seed-entry* test on
the union: the witness (`scenarios.py` / `planner.rs`) and the rust repair's
`free()` all ask "does the union fit" at a pose the route only ever justified
with a drift row. So the planner threads a row-passable gap, the robot follows
it in, and the next replan from mid-gap refuses — single-pose stub, follower
holds, stuck until timeout. `--score --gen 40`: 6/40 fail (5 timeouts + 1
collision); gen000 freezes at a pose reading union +0.033 < margin 0.05 while
the nose row reads +0.121. Pre-envelope this was impossible: one shape meant
"the plan accepted this pose" and "the witness accepts it" were the same test.

The doctrine error is "standing has no direction of travel, so the union is
the honest shape." Standing occupies the *static body*, not the union of all
swept walking boxes. No new bake needed: use the **intersection of all
envelope rows** — for GO2, 0.781 × 0.416 at off_x −0.039 (union is
0.883 × 0.593). It is the largest shape nested in every row, so the invariant
holds *by construction*: a pose whose row clears the margin also passes the
witness — **replanning from your own emitted route can never refuse.**

- Witness reads the intersection box (gold + rust); rust `fit_bin`/repair
  predicate likewise. Derived from the envelope at construction, mirrored-`off_y`
  union of each row's ± drift like `envelope_at` does; falls back to the
  union box for embodiments with no measured envelope (nothing changes there).
- Turn-in-place *edges* keep the union — that is real motion sweeping the full
  shape, and it correctly forbids a pirouette mid-gap.
- Gold cache version bumps (`v12-standing-witness`).

Acceptance:

- [x] New pinned referee test: every k-th pose along every emitted path
  (gold and candidate, curated + gen), replayed as the start of a fresh query,
  yields a plan — refusal from your own route is a failure.
  *`referee/test_replan_invariance.py`: every 5th published pose (0.5 m at the
  0.1 m path resolution), 25 worlds (15 curated + gen000-009), gold replayed
  through gold and the candidate through its own cloud. On the union: gold
  refused on 5 worlds / 11 poses — door_side at the FIRST vertex it published,
  plus corridor_side_goal, gen000, gen004, gen007 — and the candidate on 2 / 7
  (gen000 from (2.51, 1.56, 0.00), gen004 from (-0.72, -0.56, -3.14)). On the
  intersection: 0 and 0. `test_start_witness` moved with the doctrine — its
  under-margin corridor is now sized on the standing box, and door_side no
  longer witnesses the snap at all (0.117 standing vs 0.033 union at the
  snapped cell), so the pose-not-cell rule is pinned on a wall BEHIND, where
  the standing box is only 10 mm shorter than the union.*
- [x] `--score --gen 40 -s 'gen*'`: 0 timeouts, 0 collisions (gen014's
  collision inspected separately if it survives the fix).
  *40/40 goal, 0 DQ, 0 categorical failures, 114.76 mean — was 6/40 failing.
  gen014 did not survive: it reaches goal and is the battery's worst world at
  112.22, so there is no separate collision to inspect.*
- [x] Curated + gen batteries re-earn ≥ 107.62 / 102.22 through the unchanged
  judge; gold gate moves only where refusals became routes.
  *115.34 curated (16/16, boxed_in still refuses) and 114.92 mixed (56 worlds),
  0 DQ, 0 failures, env_viol 6 at 0.0435 m max. Nothing moved down: all 25
  worlds' gold routes for their own start are unchanged vertex for vertex
  across the cache bump — the search never changed, only what the seed
  accepts, and every one of those starts already passed on the union.*

## Amendment: commitment — the incumbent is an input (adopted 2026-08-07)

The search is a stateless argmin over a quantized lattice: the continuous pose
enters as a seed state (22.5° yaw bin, 0.12 m cell), so every replan is a
slightly different query even with a clean pose and a frozen map — and where
two routes near-tie, sub-noise pose motion flips the winner. gen001: ±0.1° of
yaw across the −11.25° bin boundary at (3.33, −2.89) deterministically swaps
routes (Δcost ≈ 0.11 m over 2.4 m); 25 of its 51 replans re-roll while the yaw
sweeps five boundaries — the plan visibly flaps at replan rate. Battery-wide:
**821 same-map rerolls across 40 gen worlds at a near-max 114.76** (`rerolls`
column, judge `REROLL_M = 0.15`). Prior sightings: u_trap's mid-route flip,
door1's jiggle, 050343's 14 pose-driven flips (38 % of drive time in flip
transients), the corridor near-tie, 193827's goal-region flicker. The planner
is correct every single time; it just has no reason to prefer the route it
already published, because that is not one of its inputs.

**Design: the incumbent becomes a parameter; the planner stays pure.**

```
plan(cloud, pose, goal, incumbent: Path | None) -> Path
```

- Rule, inside the search (only the planner can price it): re-validate the
  incumbent on the current map (per drift row, consistent post
  standing-witness), head-trim to the current pose, cost it under the same
  governor-time pricing; run the fresh search; **switch only if the challenger
  wins by more than `commit_margin`**. `incumbent=None` (first plan, reset) is
  bit-identical to today — every existing test and cache entry stands.
- Carrot drift: the carrot advances ~0.2 m between field replans, so the
  incumbent rarely ends at the new goal. Default: extend it — a sub-query from
  the incumbent's endpoint to the new carrot, cost = trimmed incumbent +
  extension. A carrot jump beyond `reset_carrot_m` (1.0) drops the incumbent,
  as the episode reset already does.
- Refusal discipline: refuse only when neither the fresh search nor the
  (extended) incumbent is feasible — a still-walkable incumbent beats a stub,
  which closes the residual giving-up mode from the robot side too.
*Measured, and the shape is an open question.* `referee/measure_margin.py`
bakes both floors in `path_cost`'s unit. **quanta**, eight seeds of one physical
pose (on the bin and cell boundaries, ±0.1° and ±1 mm), over gen40 + curated:
median 0.572, p90 1.146, **p99 2.139**, max 2.548 — and as a fraction of the
route's own price, median 13 %, p99 70 %, so the relative form is not the
tighter statistic either. Sweeping a WHOLE bin and half a cell, the wording
first adopted, reads 2.806 p99, but it measures a robot that really is facing
22.5° elsewhere. **jitter**, one route re-priced across consecutive local_map
frames of door2.zenoh: median 0.000, p90 0.017, **p99 0.170**, max 0.194 (on
050343, where the map window is sweeping over new ground, 4.27 — that is the
world changing, not noise). The spec's formula on those two gives 3.46 m, which
exceeds the entire remaining price of a mid-route replan and would make the
first plan permanent. **Shipped: 1.50** = 1.5 × (gen001's own priced tie 0.825
+ door2's 0.170), which covers the amendment's named case 1.8× over. The gap
between 1.50 and 3.46 is Ivan's call, and what makes it survivable either way
is that the rule is a RATCHET: it only ever switches to a strictly cheaper
route, so the switches per episode are bounded by the price it can still shed.

- `commit_margin` is **measured, not tuned**, and covers BOTH noise sources
  that can fake a better route: the p99 cost spread the seed quanta produce
  (±1 yaw bin, ±half cell around battery query poses) AND the cost jitter of a
  fixed route across consecutive local_map frames of a static field scene
  (real lidar/raycast noise, measured on the recorded worlds). Combined spread
  + headroom; it must cover gen001's 0.11 m class. One constant, one measuring
  script (envelope-bake precedent), in the gold cache key.
- Re-validation is the same per-row clearance test the search runs, on the
  CURRENT map, instantly — a detected obstruction always invalidates the
  incumbent this tick. No persistence/confirmation filter, ever: delaying
  belief in an obstacle is a robustness layer priced in collisions. Map-noise
  flapping (a voxel blinking on the corridor) is perception's ledger — the
  diagnose churn pass names it same-map vs new-map — not the planner's to
  absorb.
- State lives with the callers that already hold the last plan:
  `adapter/planner.py` (kept for `replan_due`) and the episode loop
  (`plans[-1]`). The shell owns memory; the planner owns judgment.
- Gold gets identical semantics (`v13-commitment`), cached on
  (world, query, incumbent digest) — required for referee comparability.
- Scoring moves to the right court: the **planner referee** gains a
  consistency term — replay chains feed each query its predecessor's answer,
  and a switch not earned by `commit_margin` is a named, scored violation;
  gold's own chains must score clean (gate). The control battery's `rerolls`
  stays named-not-scored (it grades the follower); it is this amendment's KPI.

Acceptance:

- [x] gen001 pinned repro: same (x, y), yaw ±0.1° across the bin boundary,
  previous answer as incumbent → the route does not change.
  *`referee/test_commitment.py`, rust and python. Without the incumbent: 41
  poses / 2.442 m against 32 / 2.328 m. With it, the second query returns the
  first's route. The `path_cost` gap between the two is **0.825 m**, not 0.11 —
  the 0.11 above is ARC, and under governor-time pricing the SHORTER answer is
  the DEARER one (4.025 vs 3.200), because it is shorter by being tighter. The
  margin is measured in price, so that is the number it has to cover.*
- [x] Battery `rerolls`: 821 → ~0 on `--score --gen 40 -s 'gen*'` (the sim map
  is static, so any residual must be an earned, > margin improvement), with
  40/40 goals and 0 categorical failures kept.
  *0 rerolls, 40/40 goal, 0 DQ, 0 categorical failures, 114.80 (was 114.76).
  **The 821 did not survive contact with its own metric.** `rerolls` compared
  consecutive plans with `divergence`, which resamples each from its OWN start
  and pairs them by index — so a plan held perfectly still and trimmed by the
  two waypoints the robot had walked read 0.20 m and counted as a mind change.
  gen001 with commitment on: 59 of 59 replans an EXACT suffix of the one
  before, and 10 rerolls. Measured by projection instead — the furthest the new
  plan gets from the old one's polyline, which is what the follower feels —
  a held plan reads 0.039 and gen001's real bin-edge flip still reads 0.206.
  Under that metric the battery is **51 → 0**. `geometry.path_offset`; the
  metric stays named-not-scored, and the mean/max choice matters: the two
  flipped routes share a corridor and part at the end, so a MEAN drowns them
  (0.043 against 0.003 for a route that did not move).*
- [x] `incumbent=None` byte-compatibility: grid invariance, replan invariance,
  and every existing referee test pass unchanged.
  *`incumbent=None` returns before any of it, in both implementations. Whole
  motion suite green (338 passed); grid and replan invariance untouched.*
- [x] Planner referee chain test + consistency scoring land together; gold
  chains score clean; the candidate re-earns; new gates frozen and recorded.
  *Chains + `unearned` land in sim.py/score.py (multiplier
  `1 − unearned/chain_steps`); `test_commitment.py` 16/16 both implementations,
  incl. the safety rail (an obstacle across the incumbent diverts the very
  next query). Mixed 56-world gates, FROZEN: gold 111.57 (0 DQ, pillar 1.0,
  commit 1.0, consistency 0.967, 0 unearned), candidate 104.01 (0 DQ,
  0 unearned; worst gen012 11.18 — the pre-existing cloud-SDF handicap).
  Caveat that will bite CI: the chains put ~10 gold solves per world behind
  (query, incumbent) cache keys — the FIRST gold battery after any gold change
  runs cold and DQs on the time limit (34/56 observed); warm the cache before
  reading a gold gate.*
- [x] Runtime: incumbent validation + extension inside the 20 ms tick budget
  (door.zenoh's dense-cloud overrun stays the separate open item).
  *door2.zenoh, 21 ticks, the crate timed directly — both builds loaded into
  ONE process and interleaved, min of 15, so they see the same weather. Fresh
  search alone 12.4 ms/tick, unchanged. Incumbent chained every tick: 26.5 →
  16.8 ms/tick, worst tick 198.6 → 59.6. Under the module's OWN rule —
  `_retask` drops the incumbent when the carrot jumps past `reset_carrot_m` —
  13.5 → 12.6, i.e. on the robot the held route costs nothing measurable.
  Two fixes, both bit-stable (63/63 plans identical across the two builds, the
  crate's 19 tests, the referee's 118 and the battery at 114.80 / 40 goals /
  0 rerolls): re-validate the trimmed route BEFORE carrying it — the two are
  independent tests joined by an AND, and the carry is a full lattice search,
  so on the tick where the map closed the corridor the robot was walking down
  it was 199 ms spent on a route the very next test threw away; and ONE
  clearance table per `plan()`, shared by the fresh search and by whatever the
  incumbent asks. `Clear` is a pure memo of (world, footprints, margin,
  lattice), so a cell the fresh search scanned is a cell the extension reads:
  measured, the extension's own footprint scans fall ~94% (7227 vs 6815
  cumulative union scans on the tick that used to pay 6800 of its own).
  What is LEFT is the extension's own A* on a goal that jumped (12k pops,
  60 ms). Sharing the heuristic sweep would fix that and may not be done: the
  sweep stops the moment ITS start settles, so one shared between two starts
  settles a different set and `h` moves for every cell outside it — admissible
  still, but a different published route, which this line is not allowed to
  buy.
  This item did not open on a commitment regression. `adapter/diagnose.py`'s
  replay pass calls `ep.plan(pts, pose, goal)` with no incumbent, so its "plan
  wall time" has never run this code at all: it reads 16.5 ms/tick on the
  pre-fix crate and 18.0 on the fixed one, the same number, and it is 21 ticks
  of rerun logging and stray-column kd-trees as much as of planning. The 29.9
  that opened the item was a loaded machine.*
- [ ] Field: the diagnose plans-pass reroll line drops on the next robot
  recording (deferred to the next session; not agent-blockable).

## Speed: eliminated via the governor, not modelled

The envelope only binds where clearance is small, and there both tracks obey
`speed ≤ governor(clearance)` by contract. So: measure `envelope(mode, speed)`
on a 2-D grid in the fitted sim (once, to prove monotonicity), then bake one
runtime row per mode at the speeds the governor permits in tight corridors.
The search stays speed-free in the sense that matters — it never *chooses* a
speed, and the envelope it plans with carries none. It does now *price* by one
(change 4): the governor is a function of clearance, which the search already
knows, so reading it costs the search nothing and buys agreement with the
follower about what a tight metre is worth.
Hole to close: yaw rate is not clearance-governed — measure the arc row at
the deployed `max_yaw_rate`, or cap yaw rate in tight segments.

## Open questions

- Follower mode discipline in tight segments (crab correction mid-doorway
  exceeds the forward envelope): rely on stamp-encoded slow-down + judge
  metric, or add an explicit constraint?
- fine = 0.04 costs ~1.6× SDF precompute — measure against the 20 ms budget.
- Yaw resolution at doors: 16 bins mean up to 11.25° misalignment =
  0.85·sin ≈ 0.16 m of phantom width — more than the envelope recovers. More
  bins (2×/4× precompute) vs local yaw refinement in tight cells only. Move
  headings stay at 16 either way (envelope varies mm per bin).
