# Imitation Learning

The public workflow is documented in
[Imitation Learning for Manipulation](../../docs/capabilities/manipulation/imitation-learning.md).

The package is split by concern:

- `profile.py` defines feature-key to typed-stream contracts.
- `collection/` records the ports generated from a collection profile.
- `dataprep/` converts MCAP recordings to LeRobot datasets.
- `policy/runtime.py` owns live synchronization and safe trajectory execution.
- `policy/lerobot/` and `policy/abc/` contain isolated backend adapters.
- `workflows.py` exposes separate lazy collection and rollout catalogs.

Profiles are Python declarations because module ports must exist before a
Blueprint autoconnects. Runtime YAML or checkpoint inspection cannot add ports
after deployment.
