# Per-heading envelope: measured

Measurement for `revision.md` §3. Tool:
`python -m dimos.navigation.motion.simulation.envelope` (`--json`, `--bake`).
95 cells, fitted preset, `freewalk_mcf.bin`, 2 s settle + 4 s window (converged
to 2 mm against a 16 s window), all robot geoms' oriented boxes projected into
the yaw-aligned base frame every sim step, MAX extents. Left and right are
measured apart and folded by union.

## The surface

Folded |θ| × commanded speed, `length × width` (m). The stand cell is
0.781 × 0.409 @ −0.039.

| \|θ\| | 0.20 | 0.35 | 0.50 | 0.75 | 0.95 |
|---|---|---|---|---|---|
| 0 | 0.786 × 0.435 | 0.781 × 0.396 | 0.813 × 0.402 | 0.853 × 0.403 | 0.873 × 0.397 |
| 26.6 | 0.790 × 0.463 | 0.778 × 0.427 | 0.802 × 0.425 | 0.833 × 0.442 | 0.868 × 0.458 |
| 45 | 0.789 × 0.482 | 0.770 × 0.466 | 0.787 × 0.483 | 0.817 × 0.488 | 0.841 × 0.507 |
| 63.4 | 0.783 × 0.524 | 0.764 × 0.495 | 0.768 × 0.520 | 0.791 × 0.523 | 0.799 × 0.554 |
| 90 | 0.773 × 0.569 | 0.752 × 0.489 | 0.754 × 0.523 | 0.761 × 0.560 | 0.766 × 0.584 |
| 116.6 | 0.759 × 0.563 | 0.754 × 0.456 | 0.759 × 0.496 | 0.759 × 0.523 | 0.765 × 0.546 |
| 135 | 0.750 × 0.508 | 0.756 × 0.436 | 0.756 × 0.459 | 0.761 × 0.500 | 0.763 × 0.524 |
| 153.4 | 0.754 × 0.457 | 0.758 × 0.408 | 0.765 × 0.424 | 0.775 × 0.454 | 0.770 × 0.469 |
| 180 | 0.754 × 0.421 | 0.753 × 0.381 | 0.756 × 0.381 | 0.762 × 0.382 | 0.775 × 0.384 |

Shape of it: **length is a forward story, width is a lateral one.** Nose-first
grows 0.78 → 0.87 long and never exceeds 0.44 wide; strafe stays 0.75 long and
grows 0.49 → 0.58 wide. Reverse is the narrowest thing the robot does (0.38)
and barely moves with speed. The all-gait union (0.85 × 0.50) is a box no
single heading needs: it is the forward length crossed with the strafe width.

## Unreachable cells

**The whole v = 0.20 column, except reverse.** This policy in the fitted sim
has a translation deadband: commanded 0.20 m/s it marches in place (achieved
< 0.01 m/s at every heading with a forward component; gain 0.21 at 153.4°,
0.49 at 180°). The threshold is between 0.25 and 0.30 m/s (0.25 → 0.03 m/s,
0.30 → 0.16). Stock physics has the same deadband one notch lower, so it is
the policy plus the fitted friction, not a preset artifact.

Every other cell tracked: drift angle within 4° of command everywhere,
|yaw drift| ≤ 0.07 rad/s, no cell failed to crab at 0.95.

**Nothing tracked to within 20%.** The command-to-body gain is 0.57–0.70 at
0.35, 0.69–0.79 at 0.50, 0.79–0.87 at 0.75, 0.83–0.92 at 0.95 — a systematic
undershoot, not a per-cell failure, so the spec's 20% gate would have voided
the entire grid. Cells are accepted on direction and yaw instead and the gain
is reported per cell (`envelope.verdict`).

That undershoot is **not** matched by the robot. On the field pair
`20260806-063428` the on-board estimator gives gain 0.82 at command 0.31, 0.81
at 0.44, 0.80 at 0.61 — flat, and where the sim reads 0.62 at 0.35. The sim
converges on reality by ~0.5 and undertracks below it. Consequence for the
bake: a sim cell commanded 0.5 (achieved 0.39) is the closest thing to a real
robot commanded 0.45, and the sim at commanded 0.35 is a robot doing 0.22 that
reality never is.

## Symmetry: mild, real, and it costs width

Widths differ between +θ and −θ by 10–33 mm (worst 0.044 at 116.6°/0.2, 0.033
at 63.4°/0.95); lengths by ≤ 18 mm. All of it is below `precision` (0.05), so
folding to |θ| is sound.

But **the swept box is not centred on the pose point in y** — it lags the
drift by up to 0.057 m at 90°/0.2 and 0.02–0.03 m at walking speeds. Folding
±θ into one row therefore unions two laterally offset boxes and costs 15–56 mm
of width over the wider of the two. The schema has `center_off` for x and
nothing for y, so that cost is structural, not a measurement artifact. See Q2.

## Monotonicity: yes above the deadband, no through it

Over v ≥ 0.35 the folded surface is monotone non-decreasing to within 6 mm in
width (worst −0.006 at 0°) and 5 mm in length (−0.005 at 153.4°) — noise
against a 0.05 precision floor. **Confirmed.**

Below it, refuted, and by more than noise: the v = 0.20 march-in-place row is
*wider* than every walking row at |θ| ≤ 116.6°, by 39 mm at 0°, 80 mm at 90°,
107 mm at 116.6°. A body told to move and refusing splays further than a body
walking. So "bake at the slowest speed the governor permits" is only safe if
the band's floor is a speed the gait can execute; the standing envelope
(0.781 × 0.409) has to be unioned in regardless, because a follower can stop
on any edge.

Speed delta per heading, governed band (stand + 0.35 + 0.50) against the
all-speed union — this is what the governor contract earns:

| \|θ\| | governed | all speeds | Δlength | Δwidth |
|---|---|---|---|---|
| 0 | 0.819 × 0.409 | 0.881 × 0.435 | +0.062 | +0.025 |
| 26.6 | 0.803 × 0.437 | 0.873 × 0.465 | +0.070 | +0.028 |
| 45 | 0.788 × 0.483 | 0.846 × 0.508 | +0.058 | +0.025 |
| 63.4 | 0.781 × 0.520 | 0.805 × 0.554 | +0.025 | +0.034 |
| 90 | 0.781 × 0.523 | 0.781 × 0.593 | 0.000 | +0.070 |
| 116.6 | 0.781 × 0.496 | 0.781 × 0.579 | 0.000 | +0.083 |
| 135 | 0.781 × 0.459 | 0.781 × 0.524 | 0.000 | +0.065 |
| 153.4 | 0.781 × 0.424 | 0.784 × 0.470 | +0.003 | +0.046 |
| 180 | 0.781 × 0.409 | 0.781 × 0.421 | +0.001 | +0.011 |

In a doorway the number that matters is width against today's 0.50 union:
nose-first drops to 0.409 (−0.091, i.e. 45 mm more room per side) and reverse
to 0.409 as well; crab at 63–116° goes the other way, to 0.50–0.52. The
governor is worth 25–83 mm of width on top of that, most of it lateral.

## Turning inflation

Extra width over the straight row at the same speed, worse turn direction:

| v | wz | wz/v (rad/m) | width | extra |
|---|---|---|---|---|
| 0.35 | 0.35 | 1.00 | 0.441 | +0.045 |
| 0.35 | 0.70 | 2.00 | 0.466 | +0.069 |
| 0.35 | 1.40 | 4.00 | 0.544 | +0.148 |
| 0.75 | 0.35 | 0.47 | 0.405 | +0.001 |
| 0.75 | 0.70 | 0.93 | 0.416 | +0.013 |
| 0.75 | 1.40 | 1.87 | 0.440 | +0.037 |

It collapses on yaw-per-metre, not on yaw rate: least squares through the
origin gives **extra width = 0.0334 · (wz/v)** m per rad/m, residuals ≤ 12 mm.
Length is unaffected (0.79–0.86, inside the straight rows' own spread).

Turning also does not move the arc's *length*, and the ± spread at v = 0.35 is
0.03–0.04 m — comparable to the effect itself at low yaw, which is why the fit
is over the max of the two directions.

`arc_inflate` is stored per rad of yaw change **on an edge**, so it carries the
edge length: `arc_inflate = 0.0334 / edge`. At the spec's `cell` = 0.12 m that
is **0.279**. At max yaw rate and governed speed (1.4 / 0.35 = 4 rad/m) an edge
turns 0.48 rad and earns 0.134 m of extra width, matching the measured 0.148.
See Q1.

**Turn in place** (v = 0, wz = ±1.4): 0.760 × 0.479 @ −0.037, i.e. narrower
than a 90° crab and shorter than any forward row. `half_diag` over that box is
0.449, against 0.492 for today's union.

## Union sanity

| set | length × width | centre |
|---|---|---|
| all 95 cells | 0.883 × 0.593 | +0.002 |
| steady, v ≤ 0.75 | 0.866 × 0.587 | −0.007 |
| steady, v ≤ 0.50 | 0.826 × 0.569 | −0.026 |
| arcs + spin only | 0.861 × 0.554 | −0.002 |
| **recorded on `Embodiment`** | **0.852 × 0.495** | **−0.009** |

Length reproduces: 0.852 sits between the v ≤ 0.5 union (0.826) and the
v ≤ 0.75 one (0.866), and the single forward-0.75 cell measures 0.853. Width
does not: we get 0.593 against 0.495, +20%.

Investigated, and it is a command-set difference, not drift. The old union's
0.495 is measured to the millimetre by the single strafe-at-0.50 cell (0.495
for −90°, 0.505 for +90°); every cell above it in width is a strafe at 0.75 or
0.95, or an arc at 0.35 with wz ≥ 0.7 — regimes an earlier "stand / fwd /
reverse / strafe / spin / arc / crab" sweep at moderate speed would not have
contained. The forward length lands on 0.75 the same way. So the sim and the
policy are where they were; the recorded union simply describes a smaller
command set than this protocol does, and is **not conservative** for fast
strafe or for slow tight arcs. If the union stays the fallback for unmeasured
embodiments and the judge's veto, it should be re-baselined to
**0.883 × 0.593, centre +0.002** (or the grid capped at what the follower is
allowed to command).

## Proposed rows

Baked over stand + 0.35 + 0.50 (see the deadband and field-gain findings; this
is `envelope.GOVERNED` and reproduces with `--bake 0,0.35,0.5`).

```python
# (deg, length, width, center_off); |angle|, 0 = nose-first, 90 = strafe,
# 180 = reverse. Measured in the fitted sim over stand + 0.35/0.50 m/s
# commands, both drift signs unioned; max swept outline, not a quantile.
envelope: tuple[tuple[float, float, float, float], ...] = (
    (0.0, 0.819, 0.409, -0.023),
    (26.6, 0.803, 0.437, -0.032),
    (45.0, 0.788, 0.483, -0.035),
    (63.4, 0.781, 0.520, -0.039),
    (90.0, 0.781, 0.523, -0.039),
    (116.6, 0.781, 0.496, -0.039),
    (135.0, 0.781, 0.459, -0.039),
    (153.4, 0.781, 0.424, -0.039),
    (180.0, 0.781, 0.409, -0.039),
)
arc_inflate: float = 0.279  # per rad of edge yaw, at cell = 0.12 m
```

Every row is shorter than today's 0.85 and all but the 63–116° crab rows are
narrower than today's 0.50 — the doorway win is on the nose-first and reverse
edges, which is exactly the door2 case.

### What actually shipped, and why it differs

Phase 1 of the revision takes the AGREED schema in
[revision.md](revision.md), which answers Q1 and Q2 differently from the
defaults proposed above. Re-baked from the same 95 cells (`--json`, identical
protocol, union reproduces to the millimetre at 0.883 × 0.5928 @ +0.0019):

```python
envelope = (
    (0.0, 0.819, 0.416, -0.023, 0.000),
    (26.6, 0.802, 0.436, -0.032, -0.008),
    (45.0, 0.788, 0.472, -0.035, -0.018),
    (63.4, 0.781, 0.500, -0.039, -0.016),
    (90.0, 0.781, 0.507, -0.039, -0.009),
    (116.6, 0.781, 0.497, -0.039, 0.000),
    (135.0, 0.781, 0.463, -0.039, -0.001),
    (153.4, 0.781, 0.422, -0.039, -0.003),
    (180.0, 0.781, 0.416, -0.039, 0.000),
)
arc_inflate = 0.0334
```

- **Q1 → option (a).** revision.md stores curvature, not per-edge yaw, so the
  number is the raw slope 0.0334 m per rad/m and the search multiplies by the
  edge's own `dyaw / length`. `envelope.arc_inflate()` lost its `edge`
  argument accordingly; 0.279 was that slope divided by a `cell` the lattice
  is now free to change.
- **Q2 → option (a), in effect.** The schema gained `off_y`, so `fold()` no
  longer unions ±θ into one laterally-symmetric box. A row is now
  `+θ ∪ mirror(−θ)` (with the stand cell, and its mirror, in every row): it is
  conservative for both signs by construction — verified row by row against
  the measured −θ extents — and it keeps the lag instead of paying for it.
- **Row deltas vs the table above** are ≤ 16 mm and go both ways: 0° and 180°
  widen 0.409 → 0.416 (the stand cell's own ±6 mm asymmetry now has to be
  covered), 90° narrows 0.523 → 0.507 and 45° 0.483 → 0.472 (the ± union that
  used to inflate them is gone). Lengths are unchanged to the millimetre.

The doorway conclusion is untouched: nose-first is 0.416 against the honest
0.593 union, i.e. 88 mm more room per side.

## Questions

**Q1: `arc_inflate` units.** The measurement collapses on yaw-per-metre, but
the field is specified per rad of yaw change *on an edge*, which only agrees
with it when the edge length is fixed. / default: keep the spec's field and
store `0.0334 / 0.12 = 0.279`, valid only while lattice edges are one `cell`
long. / options: (a) store the invariant 0.0334 m per rad/m and let the search
multiply by `wz/v`; (b) keep 0.279 and assert the edge length where the search
builds edges; (c) drop the term and cap yaw rate in tight segments, which
`revision.md` already lists as the alternative.

**Q2: no lateral offset in the schema.** The swept box lags the drift
sideways by 20–57 mm, so folding ±θ costs 15–56 mm of width — more than the
arc term earns at moderate yaw. / default: fold, as proposed above; the rows
stay symmetric and conservative. / options: (a) add a `lateral_off` column and
keep signed rows (32 rows, exact); (b) fold but let the search pick the signed
row when it knows the edge's drift sign, which it does.

**Q3: which speeds the rows bake over.** `revision.md` says the governed slow
band; the sim's 0.20 cell is a stall and its 0.35 cell is a robot doing 0.22
where the field pair says 0.29. / default: stand + 0.35 + 0.50, as above. /
options: (a) literal 0.2 + 0.35, which is 26 mm wider at 0° and 46 mm at 90°
because of the march-in-place row; (b) refit the low-speed gait before baking
anything, since the deadband is the fit's largest visible disagreement with
the field pair.
