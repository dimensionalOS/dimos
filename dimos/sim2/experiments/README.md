# World-state scaling experiment

Authorized by Pim on 2026-11-02. This is a measurement branch, not a change to
the production simulation interface. No conclusion is an acceptance test for
G1 locomotion, rendered sensors, a remote WAN, GPU simulation, or a second engine.

## Hypotheses and controls

Written before the timed sweep:

1. Publishing roots/joints instead of every derived link reduces representation
   overhead; the benefit grows with articulation/link count. Measure the current
   snapshot against an experimental compact full sample.
2. Suppressing unchanged groups reduces application bytes when most props are
   stationary. It might increase CPU time because it scans and compares state.
   Compare Jeff's normal path against the same path with `_sent` and
   `_sent_joints` cleared before capture. Report wins and losses.
3. A smaller message need not give lower capture-to-usable-state latency. Measure
   sender, codec, receiver projection and cross-process fan-out independently.
4. Full samples recover current state after missed samples; a delta-only mirror
   without a baseline/recovery cannot always do so. Test late join, loss of a
   final movement and old-message delivery, without assuming network loss.
5. Physics/contact costs can limit whole-loop gains (Amdahl's law). Measure the
   same physical trajectory with each state path and with state export disabled.

Pim: `ea95c4486d2ad28135a6ca7d3427c501bea53730` (#4955).
Jeff: `05c7a719bc9d727ed7c0c2832ec045ae369e6850` (#4909).
Jeff's actual generated messages:
`dimensionalOS/dimos-lcm @ 7d3f7cf2611fbb8c911f3fcd99fa14a370381eb5`.
Every source file used is checked against its Git object before running.

## Four paths

| ID | Extraction | Wire format | Receiver |
|---|---|---|---|
| `pim_full` | Unmodified `MujocoBackend.snapshot` | DimOS CBOR | Decode and select roots/joints |
| `compact_full` | Experimental root pose/twist + scalar q/dq/effort | Same CBOR | Decode roots/joints |
| `jeff_full` | Unmodified `frame`, comparison caches cleared | Actual generated WorldChange LCM | Actual WorldMirror + dense projection |
| `jeff_delta` | Unmodified `frame`, normal suppression | Same WorldChange LCM | Same mirror/projection |

`jeff_full` is an ablation, not a shipped mode or a recovery fix. Clearing the
comparison caches also avoids comparison work: this is intentional, to measure
the total cost/benefit of change suppression. Initial entity declarations are
still sent only once. `compact_full` is an experimental baseline, not a proposed
production patch. The current DimOS generic codec is **CBOR, not pickle**.

## Workloads

- One synthetic 29-joint articulated chain, four fixed sensor frames, and
  0 / 32 / 128 / 512 / 2048 free collision boxes. Ground contacts are real.
- Controlled extraction sweep: joints always move; 0 / 1 / 10 / 100% of props
  move. Poses are assigned deterministically and kinematics is refreshed. This
  is a state-distribution experiment, not an integrated physical trajectory.
- Physical sweep: settle 250 steps, then drive joints and apply side forces to
  10% of props, 2 ms steps and 50 Hz publication. Every path receives the exact
  same deterministic trajectory; compare checksums. Include physics-only control.
- Articulation-count supplement: increase the number of 29-joint chains.
- Delivery sweep: actual DimOS Zenoh, localhost TCP, separate spawned processes,
  1 / 4 readers, 50 / 200 Hz requested publication, repeated trials. No renderer.
  Matching reliable/block QoS and a decoded-baseline handshake are used in every
  arm. Report achieved rates and loss rather than assuming the requested rate.
- Supplemental link model: charge payload serialization time at 10 / 100 Mbps
  before sending through the same actual local transport. This is a synthetic
  serial store-and-forward model, not measured WAN performance, bandwidth
  shaping, latency, packet loss, or pipelined transfer.

All extraction arms share exactly the same compiled model and MjData. Each
backend is constructed normally. A startup-only Python profile hook retains
Pim's compiler's MjSpec, exports MJCF, removes empty anonymous default nodes
(MuJoCo 3.10 exporter emits invalid empty child defaults), then constructs Jeff's
backend from that file. Physical fields are checked before the experiment binds
both readers to the same model/data. No capture method is replaced or rewritten.

The physical sweep deliberately uses the same `mj_step` + `mj_forward` loop for
all paths. This controls the integration phase and schedule: its real-time factor
is offline kernel throughput, **not a claim about the complete shipped module
schedulers**. Sensor rendering, TF publication, joint-topic fan-out and policy
inference are outside this sweep. Their inclusion cannot be inferred from this
data. The read-side projection is the same root pose/twist and joint matrix in
all arms; it is more than simply receiving a packet.

Five trials per extraction/physics condition, deterministic shuffled path order.
Twelve warmup frames excluded. 16–120 measured extraction frames per trial (fewer
for larger scenes); raw counts and samples retained. Report median-of-trial
medians and trial range. Short-run p99 is descriptive, not a reliable tail bound.
Do not treat samples within a run as independent experiment replications.
Profiles/tracemalloc run separately from the primary timing sweep. Traced peak
live memory is not cumulative allocation count, total copied bytes or total RSS.

No dedicated isolated machine: record Mac power, versions and load. Confirm AC
power before the primary sweep. Run heavy experiments serially, not concurrently.
Bootstrap intervals, if displayed, resample trial summaries, not individual frames.

The primary run started just before the initial harness commit
`6d989a8476`. Its recorded file SHA-256 identifies that exact extraction harness;
the recorded HEAD still names the parent. A later validation run adds a SHA-256
over every sampled native qpos/qvel array, strengthening the original scalar
trajectory checksum without changing the timed kernels. Subsequent provenance
also hashes every experiment Python file. Report generation and light editing
were allowed during sweeps; this was not an isolated benchmark host.

## Setup and reproduction

Run from this branch's repository root using an environment with the same
versions recorded in the result JSON (the local run reuses the minimal worktree
venv without changing it):

```bash
mkdir -p outputs/state-scaling/deps/jeff
git fetch origin 05c7a719bc9d727ed7c0c2832ec045ae369e6850
git archive 05c7a719bc9d727ed7c0c2832ec045ae369e6850 dimos/sim2 \
  | tar -x -C outputs/state-scaling/deps/jeff
git clone --branch jeff/feat/sim2_world https://github.com/dimensionalOS/dimos-lcm.git \
  outputs/state-scaling/deps/dimos-lcm
git -C outputs/state-scaling/deps/dimos-lcm checkout 7d3f7cf2611fbb8c911f3fcd99fa14a370381eb5

python -m pytest dimos/sim2/experiments/test_state_scaling.py dimos/sim2/test_eval.py \
  --import-mode=importlib --no-cov -o addopts='' \
  --junitxml=outputs/state-scaling/correctness.xml
python -m dimos.sim2.experiments.demo_state_scaling --name primary --physics
python -m dimos.sim2.experiments.demo_state_scaling --name articulations-8 \
  --props 32 --robots 8 --fractions .01 --physics
python -m dimos.sim2.experiments.demo_state_scaling --name articulations-32 \
  --props 32 --robots 32 --fractions .01 --physics
python -m dimos.sim2.experiments.demo_state_delivery --name delivery
```

The source dependency directories are intentionally outside version control.
Every completed trial is saved incrementally under `outputs/state-scaling`.
Initial declaration bytes are reported separately from steady-state traffic.
Delivery byte counts are application payloads; they exclude TCP/Zenoh framing
and are not measurements of NIC bandwidth. Fan-out CPU is measured across the
actual subscriber processes. Traces are Chrome/Perfetto JSON; profiles are
standard Python cProfile cumulative tables.

Expected failure tests are explicit strict xfails for three defects at Jeff's
pinned revision. They are not silently skipped or reported as passes. A changed
revision that fixes the behavior must update this experiment's source pin and
expectations. Full samples alone do not reject stale out-of-order packets or
recover historical events; this experiment does not claim otherwise.
