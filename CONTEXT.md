# Robot policy rollout

The language of learned robot behavior and its execution.

## Language

**Policy backend**: An inference implementation that turns observations and a
task into predicted robot actions. A backend can load different checkpoints.

**Checkpoint**: A saved trained model and the metadata needed to interpret its
inputs and outputs.

**Observation**: The sensor information used for one policy prediction,
including camera views and the robot's measured joint state.

**Action chunk**: An ordered sequence of predicted robot actions. Its prediction
horizon may exceed the portion executed before observing and predicting again.

**Rollout**: A run of a policy in which observation, prediction, and action
execution repeat until stopped.
