# Imitation learning

Record demonstrations with a dimOS robot stack, convert saved episodes into a
LeRobot dataset, train a policy, and run a compatible checkpoint.

`dimos run` launches the robot, cameras, and collection or policy modules.
`dimos imitation control` opens one panel with the episode and policy controls
available in that stack. Q detaches; **Stop stack…** shuts down the runtime after
confirmation. Other `dimos imitation` commands prepare, inspect, visualize, and
train from saved data.

## Start with your task

| Your starting point | Guide | Result |
| --- | --- | --- |
| First time, with one OpenYAM arm and a wrist camera | [Collect demonstrations and train an OpenYAM policy](/docs/capabilities/manipulation/learning/tutorials/openyam-hand-teaching.md) | A recording, LeRobot dataset, and training outputs |
| Working Quest teleoperation, with one or two OpenYAM arms | [Collect demonstrations with Quest](/docs/capabilities/manipulation/learning/guides/collect-with-quest.md) | Saved demonstrations |
| An existing dimOS recording | [Prepare and inspect a recording](/docs/capabilities/manipulation/learning/guides/prepare-and-inspect-data.md) | A checked, viewable LeRobot dataset |
| A compatible pretrained checkpoint | [Run a compatible checkpoint](/docs/capabilities/manipulation/learning/guides/run-a-checkpoint.md) | Explicit policy start and stop controls |
| A robot in your own Python package | [Add a collection blueprint](/docs/capabilities/manipulation/learning/guides/add-a-collection-blueprint.md) | Typed recording inputs and discoverable Blueprint |

## Look up the contracts

[Recordings and profiles](/docs/capabilities/manipulation/learning/reference/recordings-and-profiles.md) describes
recording directories, feature declarations, action semantics, alignment, and
LeRobot export keys. The [CLI reference](/docs/usage/cli.md#dimos-imitation)
lists the operator and offline commands.

## Supported scope

Hand teaching uses measured joint positions as teaching actions. Quest
collection records measured state and attempted joint commands. Single-arm
OpenYAM rollout uses an existing single-camera, absolute-joint-target contract;
dual-arm collection does not imply dual-arm rollout support.

The automated profile-to-dataset check uses Python-generated recordings and
real LeRobot conversion. It does not validate native Rust recording, hardware,
training, and deployment as an end-to-end workflow.
