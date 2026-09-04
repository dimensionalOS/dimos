# Imitation Learning

The public collect → prepare → train → run workflow is documented in
[Imitation Learning for Manipulation](../../docs/capabilities/manipulation/imitation-learning.md).

This package contains the implementation layers behind that CLI:

- `workflows.py` binds built-in collection Blueprints, DataPrep Profiles, and
  rollout Blueprints.
- `collection/` records episode boundaries and robot streams.
- `dataprep/` converts recordings into training datasets.
- `policy/lerobot/` runs checkpoints in a pinned isolated environment.

These layers remain usable by maintainers when composing new built-in
workflows. Users should start with `dimos imitation list` and keep robot module
configuration out of their command line.
