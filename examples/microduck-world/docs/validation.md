# Baseline validation — 2026-09-04

Executed on Omarchy using Python 3.12.14, Deno 2.6.10 and the RTX 3070 Ti.

- Headless MuJoCo EGL render produced a nonuniform 160x120 RGB frame.
- ScenePackage and typed place metadata loaded successfully.
- Every project landmark's coordinates matched its XML geom.
- Project blueprint imported and ran through external entry-point discovery.
- Seven modules started and DimOS reported a successful health check.
- Persistent places database was created at state/places.db.
- Ruff check and format check passed on project Python sources.
- setup completed a second time; uv pip check found all 260 packages compatible.
- Chromium connected to the running cockpit and displayed both camera feeds,
  costmap, named rooms and landmarks. Chase video observed at roughly 18.5–19 fps;
  head camera roughly 6 fps. Relay reported no frame drops in the sampled interval.
- A 2.5-second simulated W-key hold moved x from about 0.001 m to 0.304 m;
  release returned the panel's commanded velocities to zero.
- A nearby map goal entered following_path and reached its destination.
- A separate room goal was cancelled through the UI; state became cancelled.
- Page reload reacquired video and state with teleop disarmed.
- Server restart passed its health check and the browser reconnected.

Evidence: logs/baseline.png. This is a baseline smoke check, not a public-load,
multi-client authorization, real-robot, or boot-recovery test. Occasional camera
renders were skipped by the engine's clock-protection mechanism under load.
