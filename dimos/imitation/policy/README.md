# Policy rollout backends

`PolicyModule` owns observation subscriptions, operator controls, episode lifecycle,
and coordinator submission. Backends own inference and protocol conversion.
Each backend runs in its own Python dependency project.

| Backend | Dependency project | Output | Execution task |
| --- | --- | --- | --- |
| `lerobot` | `native/python/lerobot` | Absolute joint targets | `joint_trajectory` |
| `dreamlabs`, joint mode | `native/python/dreamlabs` | Dual YAM joint targets | `joint_trajectory` |
| `dreamlabs`, EEF mode | `native/python/dreamlabs` | Absolute TCP pose trajectory | `eef_trajectory` |

Use `policy_module(image_mapping={port: feature}, ...)` to declare camera input
ports before blueprint composition. LeRobot feature names match the checkpoint.
Dreamlabs feature names are `overhead`, `wrist_l`, and `wrist_r`.
State vectors and returned joint arrays always use configured `joint_names` order.
LeRobot can independently map that order to `policy_joint_names`.

`preflight_rollout()` validates local configuration, live observations, and the
required coordinator task. It loads a local checkpoint when applicable, but never
requests Dreamlabs inference or creates a Dreamlabs client.

`start_rollout()` starts an episode after preflight. Dreamlabs defaults to preview;
use `start_rollout(dispatch_actions=True)` to opt into actuator submission.
LeRobot retains its existing dispatch default. `get_last_preview()` returns the
last validated chunk; it does not make a request or execute anything.

Streaming backends receive distinct capture snapshots on their observation cadence,
including while inference and execution are in progress. Snapshot validation rejects
missing, future, stale, non-finite, and excessively skewed camera/state timestamps.
Start resets history. Stop invalidates pending replies, cancels submitted work, and
drains workers. A draining request can still incur remote billing.
`busy=False` means owned workers have drained. Cancellation uncertainty remains in
`last_error` and prevents restart until cancellation succeeds.

`set_task(text)` stops and drains the episode before changing language. Quest A
starts or stops the configured rollout. Either controller grip stops it; releasing
a grip never resumes rollout.

Counters describe different events: `chunks_received` counts validated previews,
`chunks_accepted` counts successful coordinator submissions, and `chunks_completed`
counts coordinator completion reports. Joint task completion tracks its command;
EEF task completion additionally checks measured TCP and gripper feedback.

The runtime waits for coordinator completion rather than inferring completion from
chunk duration. EEF chunks run sequentially; there is no append, prefetch, or
seamless cross-chunk velocity guarantee. Stop policy and wait for it to drain before
manual planning. The shared trajectory task is not a cross-client ownership lock.

See [LeRobot rollout](lerobot/README.md) and
[Dreamlabs setup and preview](dreamlabs/README.md).
