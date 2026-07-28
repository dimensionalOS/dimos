# DimOS Pink teleop IK compared with Interlatent's historical IK

## Scope and evidence boundary

This comparison uses Interlatent's last public teleop implementation before the
package was removed, plus the public data that remains for its YAM robot. It is
important not to conflate the two:

- Interlatent's last public IK implementation was an SO-101-specific solver for
  five arm joints, not the six-DoF YAM solver. The source is preserved at commit
  [`8695afe`](https://github.com/Interlatent/Interlatent/blob/8695afe552f5bde94b2fea69aac47c79be7beafe/packages/teleop/src/interlatent_teleop/laptop/kinematics.py#L172-L293).
- Interlatent removed that public teleop package in
  [commit `347e9d1`](https://github.com/Interlatent/Interlatent/commit/347e9d1e1fbdaede09b51f9a336be149a58423dc).
- Interlatent later published YAM's URDF and tuning/specification data, including
  `solver_type: "decoupled_6dof"`, in
  [commit `97693e6`](https://github.com/Interlatent/Interlatent/blob/97693e635ba3792b81bf86572d0691b8f9d7164f/robots/yam/ik_config.json).
  It did not publish the implementation of that solver. Its current ADR states
  explicitly that the SDK ships kinematic data but the IK solver and
  retargeting stay on the closed platform
  ([ADR 0012](https://github.com/Interlatent/Interlatent/blob/31a5df280a7a6a53bdbc74aa850a8faac309dfe2/docs/adr/0012-teleop-receiver-stub-open-core-boundary.md)).

Therefore, exact algorithm-to-algorithm comparison is possible only against the
historical SO-101 solver. The public YAM files can inform a design comparison,
but not prove the closed solver's equations, step ordering, convergence
criteria, or failure behavior.

## Algorithm comparison

| Concern | Historical Interlatent SO-101 | Current DimOS |
|---|---|---|
| Robot/model scope | Hand-coded elementary-transform chain for SO-101 | URDF-driven Pinocchio model with validated coordinator-to-URDF joint mapping and named end-effector frame |
| Controlled task | Analytic pan, direct wrist roll, and a numerical solve over lift/elbow/wrist-flex for `[radius, z, pitch]` | Full six-coordinate SE(3) frame task: three translation and three orientation errors |
| Linearization | Finite-difference 3×3 Jacobian, perturbing each joint by 0.3° | Analytic Pinocchio frame Jacobian and SE(3) log/Jlog through Pink |
| Solve | Up to 60 damped pseudoinverse iterations per target | One differential-IK QP per coordinator tick, re-anchored to measured joints |
| Regularization | Adaptive scalar `lambda = 1e-6 * (trace(J J^T) + 1e-12)` | Frame-task Levenberg-Marquardt damping, global QP damping, plus a small joint-velocity damping task |
| Translation/orientation trade-off | No general trade-off: the residual mixes `r`, `z`, and one pitch angle without weights | Explicit position and orientation task costs; teleop currently sets both to `1.0` |
| Posture | None | Disabled for teleop (`posture_cost = 0`); no canonical-posture pull |
| Step/velocity limiting | Uniformly scales each inner Newton update to at most 8° per joint, then runtime independently clips each joint toward the result using profile velocity × control period | Uniformly scales the complete QP velocity vector against `min(model velocity limit, 1.0 rad/s)` and integrates it for the measured tick duration |
| Position limits | Cylindrical Cartesian target clamp before IK; joint targets clipped downstream | Robot position limits are constraints in the QP, followed by a tolerance-only boundary clamp/check |
| Failure | A linear-algebra error stops iteration; iteration exhaustion also returns the current partial result silently | Invalid inputs/results and QP failure are explicit errors; the parent task catches them and commands a measured-state hold |

### Historical Interlatent equation

For the three numerically solved joints, Interlatent formed the mixed-unit error

```text
e = [target_radius - radius(q),
     target_z      - z(q),
     target_pitch  - pitch(q)]
```

and applied the damped right pseudoinverse

```text
dq = J^T (J J^T + lambda I)^-1 e
lambda = 1e-6 * (trace(J J^T) + 1e-12)
```

The implementation then uniformly scaled `dq` when its largest absolute
component exceeded 8° and accumulated it into the iterative seed. These steps
are directly visible in
[`kinematics.py` lines 222–269](https://github.com/Interlatent/Interlatent/blob/8695afe552f5bde94b2fea69aac47c79be7beafe/packages/teleop/src/interlatent_teleop/laptop/kinematics.py#L222-L269).
That 8° value is an inner iteration bound, not a time-based robot velocity
limit: a target can consume as many as 60 such iterations before the result is
sent downstream.

The downstream safety gate first clips the finished joint target to profile
position bounds. It then independently clips each joint displacement to
`max_velocity[i] * control_dt`, anchored to the last commanded vector rather
than the current measured vector
([`safety.py` lines 147–169](https://github.com/Interlatent/Interlatent/blob/8695afe552f5bde94b2fea69aac47c79be7beafe/packages/teleop/src/interlatent_teleop/pi/safety.py#L147-L169)).
This gives each joint its full allowed progress but changes the joint-space
direction whenever only some axes saturate.

### Current DimOS equation and policy

Pink's task objective has the form

```text
min_Delta_q  1/2 ||J Delta_q + alpha e||_W^2
```

with the configured task costs forming `W`, plus regularization and position
constraints. Pink returns `v = Delta_q / dt`; see Pink 4.2's
[`Task.compute_qp_objective`](https://github.com/stephane-caron/pink/blob/v4.2.0/pink/tasks/task.py)
and
[`solve_ik`](https://github.com/stephane-caron/pink/blob/v4.2.0/pink/solve_ik.py).

DimOS updates Pink's configuration from the current measured joint snapshot
before every solve, sets the full pose target, solves once, uniformly scales
the returned velocity if any effective joint-speed bound is exceeded, and then
integrates for the current bounded `dt`. The implementation is in
[`pink_control_ik.py`](../../dimos/control/tasks/cartesian_ik_task/pink_control_ik.py).
The teleop policy uses equal numeric position/orientation costs, no posture task,
a small damping task, and a global 1.0 rad/s ceiling in
[`teleop_task.py`](../../dimos/control/tasks/teleop_task/teleop_task.py).

Equal numeric costs do not make meters and radians physically identical.
They mean Pink normalizes each translational coordinate by `1 cost/m` and each
rotational coordinate by `1 cost/rad`. With a nonredundant six-DoF arm, both can
normally be tracked together; the relative cost matters when the pose is
unreachable, constrained, or poorly conditioned.

## Orientation and posture details

The historical SO-101 code did not solve full orientation. Pan was derived from
target position, roll was passed directly, and pitch was the only orientation
coordinate in the numerical residual. There is also a source-level mismatch
worth preserving:

- The retargeter comment says it drops the explicit pitch constraint and relies
  on a minimum-norm solution
  ([`retargeting.py` lines 172–179](https://github.com/Interlatent/Interlatent/blob/8695afe552f5bde94b2fea69aac47c79be7beafe/packages/teleop/src/interlatent_teleop/laptop/retargeting.py#L172-L179)).
- The executed call passes `target_pitch_rad=None`; `ik_jacobian` replaces that
  with `gripper_pitch(current_joints)` and passes it to the three-residual solve
  ([`kinematics.py` lines 276–293](https://github.com/Interlatent/Interlatent/blob/8695afe552f5bde94b2fea69aac47c79be7beafe/packages/teleop/src/interlatent_teleop/laptop/kinematics.py#L276-L293)).

The code therefore does preserve the warm-start pitch as an explicit residual,
despite the comment. Wrist roll remains at the calibration/home value in this
retargeting path.

For public YAM data, `w_rot: 0.1`, separate position/rotation damping fields,
and `solver_type: "decoupled_6dof"` strongly suggest a translation-favoring,
decoupled design. The public data does not establish how the closed solver uses
those values, so it cannot support a stronger algorithmic claim. The spec also
contains per-joint `max_dq` values of 0.05 rad for the first three joints and
0.1 rad for the wrist joints
([YAM `kinematic_spec.json`](https://github.com/Interlatent/Interlatent/blob/31a5df280a7a6a53bdbc74aa850a8faac309dfe2/packages/sdk/src/interlatent_robots/yam/kinematic_spec.json)).

## Is the current DimOS implementation better?

For the intended six-DoF, multi-robot teleop use case, **yes, structurally**:

- It solves the actual full-pose problem rather than a robot-specific planar
  reduction.
- It uses analytic model Jacobians and proper SE(3) orientation error.
- It treats joint position bounds as part of optimization.
- It starts every tick from measured state, preventing command-state drift.
- Its time-based speed cap uniformly preserves the QP's coordinated joint-space
  direction.
- It detects solver and numeric failure and holds measured position instead of
  silently returning a partially converged target.

Interlatent's historical implementation has two useful qualities:

- A simple Cartesian workspace clamp makes obviously unreachable commands
  benign before solving.
- Best-effort iteration avoids a hard “no QP solution” event and may feel
  continuous even when its result is inaccurate.

Those qualities do not make it generally better. Silent nonconvergence, an
unweighted stopping norm that mixes meters and radians, finite-difference
Jacobians, partial orientation, and downstream component-wise velocity clipping
are weaker foundations for general six-DoF control.

The remaining uncertainty is tuning, not the overall architecture. A more
principled next comparison would instrument full-pose tracking error, joint
speed saturation, QP failures, and measured-command lag on the same six-DoF
target traces. The private Interlatent YAM solver cannot be declared better or
worse without either its code or equivalent trace data.
