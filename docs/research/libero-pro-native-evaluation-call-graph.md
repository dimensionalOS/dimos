# LIBERO-PRO native evaluation call graph

Date: 2026-08-10

> Design update: the original sealed-task recommendation in this investigation
> was superseded after checking CaP-X's task-conditioned evaluation flow. See
> ADR-0016. The upstream LIBERO-PRO call-graph findings remain valid.

## Question

What evaluation behavior is actually defined by LIBERO-PRO at commit
[`eafdb80`](https://github.com/Zxy-MLlab/LIBERO-PRO/tree/eafdb809426b13153aa1e4c42d6601844217dfec),
what behavior is inherited from original LIBERO at commit
[`8f1084e`](https://github.com/Lifelong-Robot-Learning/LIBERO/tree/8f1084e3132a39270c3a13ebe37270a43ece2a01),
and can PR #3434 preserve it in a one-task, one-trial vertical slice?

Only first-party source, official configuration, CaP-X at commit
[`53e9966`](https://github.com/capgym/cap-x/tree/53e9966d7a8e2fa7494676772bccc35280f5c0ed),
and the two benchmark papers were used.

## Conclusions

1. **LIBERO-PRO does not ship a standalone canonical rollout program.** It
   ships perturbed suite registrations/assets, a perturbation generator, and a
   README patch recipe for an external OpenVLA evaluator. Its simulator call
   graph remains LIBERO's synchronous `reset -> set_init_state -> five settling
   steps -> policy/action/step loop -> BDDL success` call graph.
2. **The horizon is a count of policy actions, not wall time.** Model inference
   occurs before each `env.step(action)` and therefore does not consume the
   step budget. `control_freq=20` controls how much simulated time robosuite
   advances per action; it does not require a continuously running 20 Hz host
   thread.
3. **The original and PRO suites are distinct artifacts, but they do not define
   an exploration/scoring split.** Following CaP-X, DimOS selects the concrete
   PRO suite and task first and discloses its exact language during exploration.
   Debug and scored trials use fresh initial states of that same task.
4. **One task and one trial can preserve episode semantics**, including the
   native success bit, but it is only an integration result. A published-style
   task success rate requires 50 episodes, and the leaderboard total is the
   mean over suite-by-perturbation success rates.
5. **No change to `Evaluation.run()` or the debug-trial callback is proven
   necessary.** The evaluator can own the fresh environment and blueprint and
   invoke `runtime.execute()` beside them. However, a continuous wall-clock
   simulator design would change benchmark semantics. If early policy-process
   cancellation is required for operational cleanup, the current synchronous
   `CodePolicyRuntime.execute()` lacks that lifecycle control; that is an
   operational gap, not a reason to move LIBERO's horizon or scorer into the
   generic runtime.

## Native call graph

LIBERO-PRO's README tells integrations to select or generate a perturbed suite,
then hand that suite to an external evaluator; it does not replace LIBERO's
rollout loop ([PRO README, evaluation patch recipe](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/README.md#evaluation-on-openvla)).
There are therefore two relevant entry points, neither a PRO-generic CLI:

- LIBERO's reusable rollout function is `evaluate_one_task_success()` in
  `libero/lifelong/metric.py`.
- `libero/lifelong/evaluate.py` is a checkpoint- and algorithm-specific CLI
  around the same environment calls
  ([source](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/lifelong/evaluate.py#L72-L304)).
  The PRO instructions instead ask users to modify an OpenVLA repository's
  `run_libero_eval.py`, which is not present in LIBERO-PRO.

The inherited native path is:

```text
get_benchmark(suite)(task_order_index)
  -> benchmark.get_task(task_id)
     -> Task(problem_folder, bddl_file, init_states_file, language)

OffScreenRenderEnv(
    bddl_file_name=<suite>/<task>.bddl,
    camera_heights=128,
    camera_widths=128,
)
  -> ControlEnv defaults:
       Panda, OSC_POSE, 20 Hz, hard_reset=True,
       agentview + robot0_eye_in_hand RGB

for evaluation batch:
  env.reset()
  init_states = torch.load(<suite>/<task>.pruned_init)
  indices = arange(batch offset) % len(init_states)
  obs = env.set_init_state(init_states[indices])
  repeat 5 times:
    obs = env.step(zeros(7))

  steps = 0
  dones = false
  while steps < max_steps:
    steps += 1
    policy_input = transform(obs, task_embedding)
    action = policy.get_action(policy_input)
    obs, reward, done, info = env.step(action)
    dones |= done
    if all(dones): break

  task_success_rate = sum(dones) / episode_count
```

The loop above is directly implemented by original LIBERO and is unchanged in
the pinned PRO fork
([original `evaluate_one_task_success`](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/8f1084e3132a39270c3a13ebe37270a43ece2a01/libero/lifelong/metric.py#L52-L161),
[PRO copy](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/lifelong/metric.py#L52-L161)).

### Task, reset, init state, and seed

The benchmark `Task` record binds a task name, language, problem folder, BDDL
file, and matching `.pruned_init` file. Original 10-task suites can apply one
of the fixed task-order permutations, whereas PRO's suffixed suites use their
task-map order because they are not in `standard_10_task_suites`
([PRO benchmark registry](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/benchmark/__init__.py#L36-L43),
[ordering behavior](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/benchmark/__init__.py#L185-L231)).
The first vertical slice should therefore use task-order index `0` (the
identity order) and record the exact task name rather than assuming arbitrary
ordered indices correspond across original and PRO suites.

Each evaluation batch first performs a normal hard reset, then overwrites the
MuJoCo state with a deterministic tensor chosen by index. `set_init_state`
sets the flattened state, forwards MuJoCo, refreshes object state and
observables, and returns a new observation
([environment wrapper](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/envs/env_wrapper.py#L90-L145)).
The loop deterministically walks init-state rows modulo the tensor length
([metric](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/lifelong/metric.py#L102-L123)).
Consequently, the comparable episode identity is primarily the BDDL hash,
init-state tensor hash, and row index. The environment `seed()` only calls
NumPy's global seed; it must be recorded, but it is not a substitute for the
fixed state identity
([seed implementation](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/envs/bddl_base_domain.py#L162-L164)).

A fresh environment/process per DimOS submission is stricter isolation than
upstream, which reuses an environment across batches. It preserves episode
semantics as long as the same artifacts, state row, and settling sequence are
used.

### Observations

The official LIBERO data configuration maps the policy surface to:

| Modality | Environment keys |
|---|---|
| RGB, 128 x 128 | `agentview_image`, `robot0_eye_in_hand_image` |
| Proprioception | `robot0_gripper_qpos`, `robot0_joint_pos` |
| Depth | none |

These mappings are unchanged in the pinned PRO fork
([original data config](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/8f1084e3132a39270c3a13ebe37270a43ece2a01/libero/configs/data/default.yaml#L1-L39),
[PRO data config](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/configs/data/default.yaml#L1-L39)).
The exact BDDL language string is a legitimate input. Reward, `done`, BDDL
goal predicates, raw simulator state, object poses, depth, and segmentation are
not part of this configured policy surface and must remain evaluator-private.
Model-side transforms computed from the allowed inputs remain policy behavior.

The frozen `policy(app)` signature has no language argument. Therefore the
DimOS blueprint must expose the current exact instruction through a
non-privileged normal runtime interface if semantic or task perturbations are
to be evaluated. Baking only the original debug instruction into generated
code would make those PRO tracks structurally impossible to solve.

### Action and controller

`ControlEnv` defaults to a Panda using robosuite's fixed-impedance `OSC_POSE`
controller and a default gripper. It sets `control_freq=20` and creates the two
camera observations at 128 x 128
([environment defaults](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/envs/env_wrapper.py#L12-L80)).
LIBERO's five settling actions are explicitly seven-dimensional
([metric](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/lifelong/metric.py#L119-L123)).
For the default controller this is six end-effector pose deltas (translation
and axis-angle rotation) plus one gripper command; robosuite documents six
fixed-impedance `OSC_POSE` arm dimensions and delta control semantics
([robosuite controller documentation](https://robosuite.ai/docs/modules/controllers.html#operational-space-control-pose-with-fixed-impedance)).

Replacing this with Panda `JOINT_POSITION` changes the benchmark action space
and should not be reported as directly comparable. DimOS may translate a
normal manipulation API into the native seven-dimensional commands, but the
commands delivered at the environment boundary and their per-step semantics
must remain native.

### Horizon and start point

The benchmark step budget starts **after** `reset`, `set_init_state`, and five
settling actions. The counter increments immediately before policy inference
and the subsequent action step
([metric loop](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/lifelong/metric.py#L109-L157)).
It is a simulator-policy-step count. There is no wall-clock deadline in this
loop.

Original LIBERO's checked-in evaluator defaults to 600 policy steps and 20
episodes
([original eval config](https://github.com/Lifelong-Robot-Learning/LIBERO/blob/8f1084e3132a39270c3a13ebe37270a43ece2a01/libero/configs/eval/default.yaml#L1-L10)).
LIBERO-PRO's OpenVLA integration recipe instead assigns suite-specific limits:

| Suite family | PRO action-step limit |
|---|---:|
| Goal | 300 |
| Spatial | 220 |
| LIBERO-10 | 520 |
| Object | 280 |

The same family limit is used for all five perturbation suffixes
([PRO README](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/README.md#evaluation-on-openvla)).
These limits are therefore part of the published PRO integration protocol,
not `ControlEnv.horizon` (whose default is 1000). The LIBERO task wrapper
replaces robosuite's returned `done` with BDDL success, so the outer evaluation
loop owns the actual failure horizon
([step override](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/envs/bddl_base_domain.py#L800-L809)).

A continuously advancing wall-clock 20 Hz simulator would penalize inference
latency with lost policy steps, unlike native evaluation. The benchmark-faithful
bridge must advance exactly one native `env.step(action)` per policy tick and
must not start that tick counter during blueprint startup or policy-process
connection.

### Success, score, and aggregation

The task environment parses goal predicates from the selected BDDL and requires
their conjunction. Its `step()` returns that private predicate as `done`; sparse
reward is also 1 on success
([goal conjunction](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/envs/problems/libero_tabletop_manipulation.py#L135-L155),
[reward](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/envs/bddl_base_domain.py#L165-L189)).
The evaluator latches `done` across steps and computes the task success rate as
successful episodes divided by requested episodes
([metric](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/lifelong/metric.py#L145-L161)).

The PRO paper evaluates 50 episodes per task
([paper, section 5.1](https://arxiv.org/pdf/2510.03827)), and the public
leaderboard reports one normalized success rate for every combination of four
suite families and five perturbation types. Its displayed `Total` is the
arithmetic mean of those 20 cells (for example, the shown OpenVLA cells average
to 0.5165, displayed as 0.52)
([PRO leaderboard](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/README.md#libero-pro-model-leaderboard)).
Although the paper calls 50 episodes consistent with original LIBERO, the
pinned original and PRO config files both still say `n_eval: 20`; use the
paper's explicit 50 for a PRO aggregate and preserve this discrepancy in the
run manifest rather than silently treating the checked-in default as canonical.

Thus a one-trial report should publish `success: 0|1`, exact episode identity,
and `completed`/infrastructure status. It must be labeled a vertical-slice
result, not a task success rate or LIBERO-PRO total.

## Perturbations and task disclosure

The PRO paper defines a task as instruction, environment (visual context,
objects, and initial configuration), and binary goal predicate. Its
perturbations change object attributes, initial configuration, language,
task goal/object set, or environment; task perturbation is excluded from
cross-type combinations
([paper, sections 3.1 and 4](https://arxiv.org/pdf/2510.03827)).
The repository implements these changes by rewriting BDDL and generating
matching init-state tensors. Task perturbation changes language, goal, and
objects of interest; other perturbators alter the relevant BDDL elements
([perturbation pipeline](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/perturbation.py#L425-L538),
[asset generation](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/perturbation.py#L541-L670)).

The earlier proposal to treat these artifacts as a hidden transfer boundary
was rejected after checking CaP-X. CaP-X selects the concrete LIBERO suite and
task, injects that task's language into the code-generation prompt, and scores
the generated code on that task. The DimOS boundary is therefore:

```text
Exploration (visible to Pi)             Measured trial
-----------------------------------     -----------------------------------
selected PRO suite and task             same PRO suite and task
exact selected task instruction         same task instruction
fresh debug init-state rows             fresh scored init-state row
native debug success feedback           native result evaluator-private
up to five fresh blueprints       ->    frozen policy, fresh blueprint
```

The task identity and instruction are not secrets. Privileged BDDL predicate
state and the scored episode's native result remain evaluator-only while the
policy runs. The Policy Artifact cannot be regenerated after observing the
scored result.

## Source and asset preparation

The pinned commit's active benchmark task map is
`libero/libero/benchmark/libero_suite_task_map.py`. It contains the complete
set of registered suffixed suites, including `*_lan`, `*_object`, `*_swap`,
`*_task`, and `*_env`
([suffixed mappings](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/libero/libero/benchmark/libero_suite_task_map.py#L1102-L1389)).
The repository also contains an older, shorter similarly named file one level
above `benchmark/`; integrations must follow the import in
`benchmark/__init__.py` and must not select that stale-looking path by filename
alone.

The BDDL and init-state corpus is distributed separately. Upstream instructs
users to download both directories from the official dataset and move them
under `libero/libero`; the pinned March 2026 source already contains the
post-November-2025 benchmark registry update
([preparation instructions](https://github.com/Zxy-MLlab/LIBERO-PRO/blob/eafdb809426b13153aa1e4c42d6601844217dfec/README.md#L157-L173)).
Source preparation must verify the requested commit and hash the separately
installed episode assets.

## Mapping to PR #3434

The existing contracts are sufficient for the vertical slice:

```text
Evaluation.run(config, context)
  -> context.runtime.explore(
       task_input=<selected PRO task instruction>,
       submit_debug_trial=<fresh same-task blueprint callback>,
     )
  -> freeze ExplorationOutcome.policy
  -> create fresh same-task PRO environment + fresh DimOS blueprint
  -> run policy process beside evaluator-owned synchronized native step loop
  -> stop blueprint and environment
  -> EvaluationReport(native success bit + episode manifest)
```

The debug callback already returns a fully stopped `TrialRun`, and
`EvaluationReport` can carry native structured output. Neither requires
LIBERO-specific fields.

Two constraints should shape implementation without widening the generic
interface:

- Blueprint and simulator readiness must complete before the five settling
  actions and measured policy-step counter.
- The evaluator/bridge, not `CodePolicyRuntime`, must own BDDL selection,
  init-state loading, step synchronization, horizon, native predicate, and
  scoring.

`CodePolicyRuntime.execute()` is synchronous and exposes no start barrier or
cancel handle. The evaluator can run it concurrently with its bridge, so this
does not by itself block native stepping. It does mean the evaluator cannot
promptly terminate a still-running policy process on early native success; it
can only stop the blueprint and wait for completion/timeout. Add generic
cancellation only if the vertical slice proves this cleanup behavior is
unacceptable. Do not use a wall-clock timeout as the benchmark horizon.

## Vertical-slice acceptance criteria

One trial preserves native semantics if it records and verifies all of the
following:

- exact selected PRO suite/task shared by debug and scored runs;
- PRO source revision plus BDDL and init-state hashes;
- task-order index `0`, exact task name, exact instruction, and init row;
- Panda, default `OSC_POSE`, seven-dimensional action, and 20 Hz simulated
  policy frequency;
- two 128 x 128 RGB views plus joint/gripper proprioception only;
- reset, exact state restoration, five zero-action settling steps, then the
  suite-specific action-step horizon;
- evaluator-private BDDL conjunction success, exposed only after shutdown;
- a fresh environment and complete fresh DimOS blueprint for every debug and
  measured attempt; and
- a final report explicitly labeled `single_trial`, with no aggregate claim.
