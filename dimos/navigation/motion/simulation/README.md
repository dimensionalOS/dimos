# Go2 matched simulator

MuJoCo Go2 fitted to real-robot recordings (VR-tracker pose + the executor's
commanded joint targets), close enough to train against. `FITTED_*` in
`evaluate.py` is the validated physics; `FINDINGS.md` is the full story.
Self-contained: this repo's venv only, data via
`get_data("ml-trajectory-research/...")`.

```bash
# tests
python -m pytest dimos/navigation/motion/simulation -q
python -m mypy dimos/navigation/motion/simulation

# watch the fitted sim against the recorded ghost
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --view --ghost --fitted --start 6

# score it (per-statistic sim vs real, noise-floor units)
python -m dimos.navigation.motion.simulation data/ml-trajectory-research/unitree_himloco01.mcap --policy data/ml-trajectory-research/freewalk_mcf.bin --eval --fitted

# refit physics (joint fit across recordings; --seed-fitted must beat the preset)
python -m dimos.navigation.motion.simulation.search data/ml-trajectory-research/unitree_himloco01.mcap data/ml-trajectory-research/freewalk_mcf.bin --also data/ml-trajectory-research/unitree_v11_gait_height01.mcap data/ml-trajectory-research/v11_final.bin --seed-fitted
```

Swap in `unitree_v11_gait_height01.mcap` + `v11_final.bin` for the
gait-height net (46-obs, commandable body height).

`recorded_world.py` freezes a real navigation run's map into a static world —
a stability-filtered voxel union, greedy-merged into collision boxes for this
scene and into band rectangles the referee reads as scenario obstacles. See
`tools.md` for the oneliners.
