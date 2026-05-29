# human-debug — NOT part of the experiment

These files are for manual inspection by a human only. The autoresearch loop
(program.md) must NOT use them. The experiment's input (`data/go2-185959.bin`)
and ground truth (`data/gt_robot_odom.tsv`) are already prepared.

- `mcap_to_plnr1.py` — converts a go2-station `.mcap` recording into the PLNR1
  binary that Point-LIO reads. Already run; output is `data/go2-185959.bin`.
  Usage (human): `uv run --with mcap --with numpy python mcap_to_plnr1.py <in.mcap> <out.bin>`
- `20260529-185959.mcap` — the raw recording (lidar/imu/odom/video/telemetry).
- `20260529-185959.rrd` — the recording converted for Rerun visualization.
