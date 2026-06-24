# gsc_pgo hyperparameter analysis + online tuning — sketch

## The problem (from the eval campaign)

cmu wins on big real-drift loops but **over-fires** elsewhere:
- `grassy_field` (05-32): **-0.369** tag with 110 closures — should be a no-op (+0.00x).
  Open grass = feature-poor; ICP matches are garbage but get accepted.
- `gir_park1_2`: +0.578 with 23 closures, but unrefined got **+0.749 with 2 closures**.
  cmu's extra closures are net noise — fewer, better closures would win.

Goal: (a) **detect feature-poor / high-disagreement scans online and back off**
(ideally make PGO a no-op in grassy_field), and (b) accept **fewer, more
consistent** closures. Both reduce to: *trust loops less when the environment
can't support them, and only commit mutually-consistent ones.*

## Signals cmu already computes (cheap to expose)

Every candidate already produces these inside `searchForLoopPairs` /
`scan_context.cpp` — instrument the binary to log them per candidate (accepted
and rejected), keyed by ts:

| signal | where | meaning |
|---|---|---|
| scan-context min cosine distance | `best_distance()` | place-match quality; high = no good match (new place OR feature-poor) |
| ICP fitness (`getFitnessScore`) | `searchForLoopPairs` | mean-sq inlier dist; high = bad geometric alignment |
| ICP converged + inlier ratio | PCL ICP | low inlier ratio = lots of disagreement (crowd/dynamic/degenerate) |
| candidate global distance | `loop_candidate_max_distance_m` gate | drift magnitude at the candidate |
| ring-key occupancy/entropy | scan-context descriptor | **feature richness** — near-uniform/empty descriptor = feature-poor |

Two signals are *not* computed yet but are the highest-value adds:
- **registration Hessian eigenvalues** (J^T J of the ICP cost) — the canonical
  degeneracy detector (below).
- **per-point residual distribution** of the scan-to-submap match — heavy tails =
  dynamic objects (crowd).

## Literature: detecting feature-poor / degenerate environments

1. **Degeneracy factor (Zhang, Kaess, Singh — ICRA 2016, "On Degeneracy of
   Optimization-based State Estimation").** THE standard. Eigen-decompose the
   scan-matching information matrix `H = J^T J`; its smallest eigenvalue (or
   `λ_min / λ_max` condition number) measures how well-constrained the solve is.
   A featureless corridor/field has a small eigenvalue along the unconstrained
   axis. → online "is this scan match trustworthy" scalar. Their **solution
   remapping** projects the update out of degenerate directions; LOAM/LeGO-LOAM
   use it.
2. **X-ICP (Tuna et al., T-RO 2023) — "Localizability-Aware ICP."** Per-axis
   localizability from the optimization constraints; selectively adds soft
   constraints only in observable directions. More principled than a scalar gate.
3. **Eigenvalue geometric features (Demantké 2011 / Weinmann 2015):** local
   neighborhood covariance → linearity/planarity/scattering. Open grass scores
   high "scattering" (3D-spread, no structure) — a direct feature-poverty flag.
4. **Scan-context descriptor entropy/occupancy** (cheap, already have the
   descriptor): a low-entropy / sparsely-occupied ring-key = feature-poor scene
   where cosine matching is unreliable. Raise the match bar or refuse loops.

## Literature: scan disagreement / dynamic crowds

5. **Robust kernels (Huber / Cauchy / Geman-McClure)** on the ICP residual —
   down-weight the moving-people points. cmu currently uses raw fitness; a robust
   cost + reporting the *inlier ratio* exposes "crowd present."
6. **Removert (Kim & Kim, IROS 2020):** range-image visibility differencing
   removes dynamic points before matching. Strong for crowds.
7. **Stationarity / scan-to-scan consistency:** points that move between
   consecutive frames are dynamic; a high dynamic-fraction → distrust this scan
   for loop closure. (Cheap online proxy: residual after rigid alignment of
   consecutive scans.)
8. **Learned moving-object segmentation (4DMOS, LMNet, MotionSeg3D)** — heavier;
   masks people directly. Overkill for now but the ceiling.

## Literature: loop-closure outlier rejection (the over-firing fix)

The single highest-leverage change — cmu accepts each loop independently:

9. **PCM — Pairwise Consistency Maximization (Mangelson et al., ICRA 2018).**
   Build a consistency graph over candidate loops; keep only the **maximal
   mutually-consistent clique**, reject the rest. Directly attacks "23 closures
   where 2 good ones win." Cheap, backend-agnostic, no tuning.
10. **GNC — Graduated Non-Convexity (Yang et al., RA-L 2020) / Cauchy-IRLS.**
    Backend robustly down-weights outlier loops during optimization. iSAM2-friendly.
11. **Switchable Constraints (Sünderhauf & Protzel 2012) / Dynamic Covariance
    Scaling (Agarwal 2013):** the optimizer learns a switch/weight per loop and
    can turn bad ones off. Lighter than PCM, online-friendly.

## Online tuning design (signal → knob)

Frame cmu's static thresholds as **initial values adapted by a measured
"environment trust" `τ ∈ [0,1]`** computed per keyframe from the signals above:

```
τ = f( degeneracy_factor, ringkey_entropy, icp_inlier_ratio, dynamic_fraction )
    # low when feature-poor / crowded / degenerate
```

Then drive the gates from `τ` (running statistics, not magic numbers):

| knob (currently static) | online rule |
|---|---|
| `loop_score_thresh` (ICP gate) | tighten as `τ` drops; track running median/MAD of accepted-loop fitness and gate at `median − k·MAD` |
| `scan_context_match_threshold` | raise when ring-key entropy is low (cosine less discriminative in feature-poor scenes) |
| **accept/suppress** the loop | hard-gate: if `degeneracy_factor < ε` (Zhang) → **suppress** → PGO no-op in directions it can't observe (the grassy_field fix) |
| loop **batch** acceptance | replace independent accept with **PCM** over the recent candidate set → only commit the mutually-consistent subset (the gir_park1_2 fix) |
| loop noise model | already `Σ = fitness·I`; inflate further by `1/τ` so low-trust loops barely move the graph |

Online-tuning techniques that fit (cheapest first):
- **Adaptive thresholding on running stats** (median/MAD, EWMA) — robust, no
  training, the right default.
- **Switchable constraints / DCS** — lets the *optimizer* do the tuning; minimal code.
- **PCM** — not "tuning" but the structural fix for over-firing; do this first.
- (Avoid online Bayesian-opt/bandits for the fast gates — too slow/unstable to
  converge within one run; reserve for offline meta-params.)

## Proposed analysis (to validate before coding the online controller)

1. **Instrument the binary** to emit per-candidate diagnostics (the table above)
   to a sidecar jsonl. One pass per recording → the dataset that grounds `f(·)`.
2. **Static sweep** (via the new eval cell-caching + `--drift-per-sec`): vary
   `loop_score_thresh`, `scan_context_match_threshold`,
   `loop_candidate_max_distance_m`, `min_loop_detect_duration` per recording;
   plot tag/voxel improvement & closure count. Expect grassy_field to be
   monotone-better as the gate tightens (→ confirms "no-op is optimal there").
3. **Correlate** accepted-vs-rejected closures with the signals: does
   degeneracy_factor / ring-key entropy separate grassy_field's bad closures
   from gir_park1_2's good ones? If yes, `τ` is learnable as a simple threshold.
4. **Prototype** the cheapest controller (PCM + degeneracy-gate + adaptive
   `loop_score_thresh`) and re-run the full eval; target: grassy_field → ~0,
   gir_park1_2 → ≥ unrefined's +0.749, no regression on the pointlio wins.

## First concrete step

Add the per-candidate diagnostic logging to `simple_pgo.cpp` (scan-context
distance, ICP fitness, inlier ratio, candidate distance) + compute the Zhang
degeneracy factor from the ICP Hessian. That single instrumentation pass
produces the data to decide which signals actually separate good/bad closures
here — everything else (the `f(·)` and the gates) follows from it.
