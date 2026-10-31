# Shared policy rollout runtime

`PolicyModule` owns camera subscriptions, fresh joint-state snapshots, preflight,
start, stop, cancellation and coordinator completion reporting. Independently
installed backends own checkpoint loading, prediction, reset and close.

The initial backend is LeRobot, in `native/python/lerobot`. Its implementation
returns absolute joint targets in configured hardware `joint_names` order.
`policy_joint_names` maps checkpoint order independently.

Use `policy_module(image_mapping={port: feature}, ...)` to declare typed camera
ports before blueprint composition. Preflight loads the checkpoint and validates
live inputs and the coordinator task without dispatching a trajectory.

Start requires successful preflight. Stop invalidates pending predictions, cancels
submitted work and drains the worker. Cancellation uncertainty prevents restart.
Controller grip takeover stops rollout and never resumes it on release.

The runtime waits for coordinator completion. Joint task completion describes
its command rather than proof of physical arrival. Preview mode submits no work.

See [LeRobot rollout](lerobot/README.md).
