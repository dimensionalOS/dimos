# GSC vs Ivan PGO report

Per recording: a top-down map per method (GSC / Ivan) scrubbable over time — each keyframe's lidar points placed by the
newest graph state the method had at t, optional GT overlay, robot pose, loop edges, planner layer — and a
"Loop analysis" list of GSC's closure candidates with gate numbers and ICP clouds.

## Build

Environment (defaults in parentheses): `GSC_WORKTREE` (`~/repos/_mls_reform_wt`), the dimos checkout the exports and
scorers run from, with a `.venv`; `GSC_SCORE_STUBS` (`~/.cache/gsc_score_stubs`), the PYTHONPATH the scorers get;
`GSC_SCRIPTS` (`../gsc_vs_ivan`), where score_run.py and tag_agreement.py live.

    $GSC_WORKTREE/.venv/bin/python build_report.py manifest.json out     # ~1 min for the two placeholders
    build_report.py manifest.json out --only gir2                                  # re-export one recording

Then serve and open (fetch is blocked on file://):

    cd out && python3 -m http.server 8765    # http://localhost:8765

`out/` (index.html + report.json + one folder per recording) is ready to publish as-is; the build fails if any file is
>= 15 MB or the folder >= 60 MB.

## Manifest entry (manifest.json "recordings")

- `name`, `label`; `world_db` (gsc_runs/world_dbs/<rec>.db: world-frame `lidar` + `odom`)
- `gsc` / `ivan`: run dirs with a.kf + a.kf.closures (GSC also a.json, a.kf.edges, icp/); either may be omitted
- `gt` (optional): list tried in order — `{"db", "tf": ["map", "odom"]}` (Map Editor icp_stitch edge) or
  `{"db", "stream", "odom_stream"[, "odom_db"]}` (old corrected stream). Helper: `gt_poses.py`.
- `plan` (optional): `{"gsc": path, "ivan": path}` to JSON `{"states": [{"ts": s since recording start, "goal": [x, y], "path": [[x, y], ...]}]}`;
  drawn when present.
- `score_args` (optional): `{"score": [gt_db, gt_stream], "tags": [db, odom_stream, odom_frame]}` runs score_run.py /
  tag_agreement.py when a run has no score.json / tags.json (otherwise those files are just read).
- `options` (optional): `voxel_m` 0.2, `max_points` 150, `state_budget_bytes` 4.4e6, `icp_accepted` 40, `icp_rejected` 40,
  `icp_single` [1500, 2500], `icp_extended` [2500, 3500] (source, target points kept per ICP round).

One recording by hand: `PYTHONPATH=$GSC_SCORE_STUBS .venv/bin/python export_recording.py '<entry json>' out` from the worktree.
