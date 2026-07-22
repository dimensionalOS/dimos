# Map Postprocessing

1. Record a map using a blueprint like:

```
dimos run unitree-go2-mid360-record
```

2. Run it through post processing

```
python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/post_process.py --rec=PATH
```

Thats it. It'll generate a loop-corrected map.

## Knobs worth knowing

- `--no-icp` — tag PGO only, skip the very slow ICP refinement stage.
- `--no-rrd` — skip the cloud export / the viewer.
- `--out=NAME` — output prefix, if you want to keep several corrections in one db
- Tag quality gates (sharpness, reprojection error, distance, view angle, motion blur) are in `dimos/navigation/jnav/utils/apriltags.py`
