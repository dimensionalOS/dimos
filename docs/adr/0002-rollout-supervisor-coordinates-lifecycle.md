# Rollout supervisor coordinates policy and control tasks

A policy module represents one configured checkpoint and exposes start, stop,
and status operations. A separate supervisor handles semantic operator actions.
It activates the policy's low-priority arm and gripper tasks before starting
inference, then deactivates both tasks when the operator stops or overrides the
rollout. Activation-gated tasks also deactivate themselves after arbitration
preempts them. The operator must explicitly start them again.

Input callbacks and supervisor RPCs only enqueue requests. One worker performs
policy and coordinator RPCs in order, so transport threads never wait on a
cross-module call.
