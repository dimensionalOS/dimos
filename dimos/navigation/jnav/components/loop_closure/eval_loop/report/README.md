# GSC vs Ivan PGO report

Per recording: a top-down map per method (GSC / Ivan) scrubbable over time — each keyframe's lidar points placed by the
newest graph state the method had at t, optional GT overlay, robot pose, loop edges, planner layer — and a
"Loop analysis" list of GSC's closure candidates with gate numbers and ICP clouds. Above them a summary table, below them
performance (Mac, Jetson, MLS planner), the system diagrams and the method notes (`appendix_*.html`).

## Build

Environment (defaults in parentheses): `GSC_WORKTREE` (`~/repos/_mls_reform_wt`), the dimos checkout the exports and
scorers run from, with a `.venv`; `GSC_SCORE_STUBS` (`~/.cache/gsc_score_stubs`), the PYTHONPATH the scorers get;
`GSC_SCRIPTS` (`../gsc_vs_ivan`), where tag_agreement.py and recordings.sh live.

    score_all.py manifest.json scores.json                       # GT error (score_gt.py), tag spread, loops, drops, speed, RAM
    build_report.py manifest.json out --scores scores.json [--jetson <dir of pgo-eval runs on a Jetson>]
    build_report.py manifest.json out --scores scores.json --only gir2   # re-export one recording
    odom_noise.py manifest.json <recording> <gsc run dir>        # odometry drift per keyframe vs GT (the noise defaults)

Then serve and open (fetch is blocked on file://):

    cd out && python3 -m http.server 8765    # http://localhost:8765

`out/` (index.html + report.json + one folder per recording) is ready to publish as-is; the build fails if any file is
>= 15 MB or the folder >= 60 MB. It is written without doctype/head/body, for a host page that supplies them.

## Manifest entry (manifest.json "recordings")

- `name`, `label`; `world_db` (gsc_runs/world_dbs/<rec>.db: world-frame `lidar` + `odom`; `streams` {"lidar", "odom"}
  names them otherwise)
- `gsc` / `ivan`: run dirs with a.kf + a.kf.closures (GSC also a.json, a.kf.edges, icp/); either may be omitted
- `gt` (optional): list tried in order — `{"db", "tf": ["map", "odom"]}` (Map Editor icp_stitch edge) or
  `{"db", "stream", "odom_stream"[, "odom_db"]}` (old corrected stream). Helper: `gt_poses.py`.
- `plan` (optional): `{"gsc": path, "ivan": path}` to JSON `{"states": [{"ts": s since recording start, "goal": [x, y], "path": [[x, y], ...]}]}`;
  drawn when present, at the odometry rate with paths simplified to a quarter voxel. Made by `../gsc_vs_ivan/replay/{gsc,ivan}_plan.py`.
- `score_gt: false` skips GT error (office555 is scored on its QR start/end gap instead); `tags` {"odom_frame",
  "camera_frame"} overrides the tag chain's frames.
- `options` (optional): `voxel_m` 0.2, `max_points` 80, `state_budget_bytes` 1.4e6, `icp_accepted` 10, `icp_rejected` 10,
  `icp_single` [1500, 2500], `icp_extended` [2500, 3500] (source, target points kept per ICP round).

One recording by hand: `PYTHONPATH=$GSC_SCORE_STUBS .venv/bin/python export_recording.py '<entry json>' out` from the worktree.
