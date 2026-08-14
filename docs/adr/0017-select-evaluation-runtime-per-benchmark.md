# Select the evaluation runtime per benchmark

Each `Evaluation` declares one runtime profile, and each run specification pins
that profile's model and thinking level. The runner constructs the declared
runtime and records its identity in `run.json`.

`code-policy-v1` lets Pi explore with debug trials, freezes a callable, and
leaves the measured runtime loop. VLN-CE uses `live-agent-v1`: Pi and its
persistent Python workspace remain active while the measured episode runs.
Both profiles use the same evaluation runner and evidence contract.

Navigation requires repeated choices from new observations and often uses a
long wall-clock horizon. Freezing a callable before the episode would remove
the decision-maker this condition intends to measure. A benchmark that measures
a frozen CodePolicy instead must keep the agent outside that loop. The
evaluation therefore owns this choice instead of exposing it as an operator
toggle.
