# Policy modules own rollout lifecycle

A policy module represents one configured checkpoint and exposes start, stop,
and status operations for its rollout. Operator controls live in a separate
rollout controller, and policy commands enter the control coordinator through a
dedicated low-priority task. A higher-priority task emits a typed preemption
transition that stops the rollout and resets its inference state. The operator
must explicitly start it again. This prevents an active policy from continuing
to infer unseen commands while teleoperation or planned motion owns the robot.
