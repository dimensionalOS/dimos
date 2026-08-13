# Proposal: bit-exact replay via plan provenance

> **Status: PARKED (2026-08-05).** Ivan's call: with replanning gated on map
> arrival, nearest-before pairing picks the triggering map essentially always
> (two maps can't land inside one 30 ms plan latency at 1 Hz), and the
> odometry residual is a bounded 1-sample / ~2 cm. Guessing-based replay is
> the workhorse; occasional knife-edge divergence is informative, not noise.
> Revisit if replay becomes a strict CI regression gate. The config-hash
> field is the piece most worth salvaging first: nothing today checks that
> diagnose's knobs match what the blueprint deployed.

## Problem

The planner is deterministic (same inputs twice → bit-identical, verified by
diagnose), but offline replay reconstructs its inputs by *nearest-published-
before-the-tick* pairing. The module paired by *arrival in its own process*.
The residual is real: median 0.04 m replay-vs-requested divergence as a noise
floor, with occasional homotopy flips where a one-sample pose difference lands
on a knife edge. Divergence today means "pairing noise, probably" — it should
mean "code drift, investigate".

## Non-goals

- Deterministic scheduling of the live stack (ordered single-queue transport).
  Heavy, touches every module, and unnecessary: we don't need the robot to be
  replayable from first principles, only for each decision to say what it used.
- Changing the Path message or the stamps dialect. Third-party consumers stay
  untouched.

## Design: a witness stream

`MotionPlanner` (python + rust twin, in lockstep) publishes one small message
per planning tick on a new topic, atomically with the plan it describes:

    plan_provenance : PlanProvenance
      ts          float   # equals the published plan's ts -- the join key
      map_ts      float   # msg.ts of the local_map the search ran over
      route_ts    float   # msg.ts of the planner_path the carrot came from
      odom_ts     float   # msg.ts of the odometry sample the pose came from
      pose        f64[4]  # the RESOLVED base pose used: x, y, yaw, z
      goal        f64[2]  # the carrot handed to the search
      ground_z    f64     # the surface the band was referenced to (nan = raw band)
      config_hash u32     # planner-relevant config fields, fnv1a over repr

The first three fields are *identities* (which messages). The next three are
*witness values* (what the module derived from them). Identities make pairing
exact; witnesses make drift localizable: replay recomputes pose/goal/floor from
the identified inputs and compares against the witness before planning.

    identity mismatch impossible -> pairing is exact by construction
    pose differs    -> tf resolution / OdomBasePose drift
    goal differs    -> carrot_along drift
    ground differs  -> base pose / obstacle-model drift
    all match, plan differs -> the search itself changed (or rust/python skew)

That turns "replay diverges" from a shrug into a five-way pointer.

## Message plumbing

New LCM msg `dimos.msgs.diagnostic_msgs.PlanProvenance` (or nav_msgs if that
is the closer home). Fixed-size, ~70 bytes. The zenoh motion blueprints add
the topic to the recorder set. Old recordings simply lack the stream and
replay falls back to today's nearest-before pairing (diagnose prints which
mode it ran in, so numbers are never silently mixed).

## Module changes (both twins, parity-tested)

At `_plan_once` the module already holds every field -- the map message it
snapshotted, the odometry sample, the route, the resolved pose, the carrot,
the floor. The change is capturing their `ts` alongside the payloads it
already stores (three `float` fields on existing state) and one publish.
No new locks, no extra work in the hot path beyond serializing ~70 bytes.

`config_hash` covers exactly the fields that change the search: z knobs
(obstacle_model, embodiment), lookahead,
planner id, embodiment tag. Hash printed at startup; replay prints both on
mismatch and refuses `--strict`.

## Replay changes (diagnose.py)

- If the stream exists: build Ticks from provenance rows -- map/route/odom
  fetched by exact ts (dict lookup, not searchsorted); verify witnesses;
  report any witness drift per stage; then plan.
- `--strict`: exit nonzero on witness or config mismatch (CI-able: a nightly
  job replays a checked-in recording and asserts zero divergence).
- Old pairing kept as fallback and behind `--no-provenance` for A/B.

## What this buys beyond debugging

- The flip-detector gets teeth: with pairing noise at zero, any
  requested-vs-replay divergence is a real behavior change -- a free
  regression test for every planner/anchor/raycaster PR, against every
  recording we keep.
- The same witness pattern extends to the follower later (which plan revision
  + pose it acted on per cmd_vel) if we ever need controller-side replay --
  same message, different stage tag.

## Cost

~30 lines python, ~40 lines rust, one msg definition, one parity test, one
diagnose loader branch. No behavior change for any existing consumer.

## Open questions

1. Msg home: `diagnostic_msgs.PlanProvenance` vs a generic
   `nav_msgs.Provenance` with a stage string. Default: diagnostic_msgs,
   planner-specific fields, no premature generalization.
2. Should the follower get its witness stream in the same PR? Default: no --
   planner first, follower when controller replay actually needs it.
3. `--strict` in CI needs a small checked-in recording (the 15 s one is
   3.9 MB in LFS -- acceptable?). Default: yes, the 173105 run.
