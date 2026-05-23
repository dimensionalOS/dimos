# Memory World — Feature Ideas

Backlog of features we could pull into the VR walkthrough, sourced from a survey of `dimos/memory2/`. Each item lists the memory2 API doing the heavy lifting, scope, and rough effort. Nothing here is committed — pick and choose.

Already shipped (for reference):
- Point cloud from pickled `PointCloud2`, voxel-downsampled server-side
- Top-down density map (HUD minimap + ground projection)
- Image-pose ring markers + toggleable thumbnail quads
- Odom trail polyline
- Smooth walk / yaw / teleport / bimanual-grip scale

---

## Tier 1 — biggest wins, infrastructure already in place

### 1. Semantic search ("find me the chair")
`color_image_embedded` stream already has CLIP vectors; backend supports it via sqlite-vec cosine search.

- **API**: `embedded_stream.search(model.embed_text("chair"), k=20)` returns top-k poses (`stream.py:194`)
- **VR UX**: fixed query menu (8–10 buttons) or voice input → matching markers light up in a distinct color on HUD + 3D, with similarity scores
- **Scope**: ~150 lines (query UI + `MSG_SEARCH_RESULTS` + scene method to recolor markers)
- **Effort**: medium-high (the text-input-in-VR is the hardest part; fixed query list is the easy MVP)

### 2. Keyframe sampling for image markers
Replace uniform `throttle()` with motion-aware sampling so markers land at interesting moments.

- **API**: `stream.transform(speed()).transform(peaks(prominence=0.1, distance=2.0)).transform(significant(method="mad", k=2.0))` (`transform.py:176,206,317`)
- **Server-only**, no client change
- **Effort**: low (~30 lines, swap the `throttle` line in `_build_image_poses`)

### 3. Per-pose lidar scan overlay
When you stand near or pinch an image marker, render the actual lidar scan from that exact moment in space.

- **API**: `lidar_stream.at(image_obs.ts, tolerance=0.5)` → single PointCloud2; transform to world frame via `obs.pose`
- **VR UX**: cyan overlay points around the marker showing what the robot saw at capture time
- **Effort**: medium (~120 lines, mostly server side; new `MSG_LIDAR_SCAN` per index or fetch on focus)

---

## Tier 2 — strong UX additions with moderate effort

### 4. Trajectory fly-through / playback
Animate the camera along the odom stream with play/pause/scrub. Watch the robot's POV in VR.

- **API**: iterate `odom.transform(throttle(0.05))` with pose-interp; `smooth_time(0.5)` (`transform.py:377`) prevents jitter
- **VR UX**: 3D play/pause panel; left thumbstick maps to playback speed in this mode
- **Effort**: medium (~200 lines, new mode toggle + camera-target animation)

### 5. Spatial proximity auto-thumbnails
Use `.near(my_pose, radius=2.0)` so image markers within 2m of your head auto-expand to thumbnails.

- **API**: R*Tree-backed `.near()` (`observationstore/sqlite.py:81`); client sends current pose every ~500ms, server returns nearby indices
- Replaces the gross "left-X toggle all" with smooth proximity reveal
- **Effort**: low-medium (~80 lines; reuses existing thumbnail pipeline)

### 6. Voxel-map render instead of raw point cloud (Implemented)
Use `VoxelMapTransformer` on the lidar stream to build a downsampled, deduplicated cloud server-side.

- **API**: `lidar_stream.transform(throttle(...)).transform(FnTransformer(to_world_frame)).transform(VoxelMapTransformer(emit_every=0, voxel_size=0.05)).last()`
- Implemented in `module.py:_build_voxel_cloud_from_lidar`; enable with `--cloud-source lidar` (or `cloud_source="lidar"`). Each scan is pose-transformed to world frame first; result is height-coloured. Falls back to pickle if the build fails.

### 7. Cross-stream synchronized scrub
Pick a timestamp; show that exact frame in 3D space: image quad + lidar scan + robot pose marker, all consistent.

- **API**: `.at(ts, tolerance=0.1)` on each of `color_image`, `lidar`, `odom`
- **VR UX**: timeline bar with grab handle; on scrub, all three streams update together
- **Effort**: medium (~150 lines server + client timeline panel)

---

## Tier 3 — fancy but a real lift

### 8. VLM caption overlays
Run Moondream over each pose's image. Show floating text labels above markers: "office hallway", "person at desk".

- **API**: custom `Transformer` wrapping `MoondreamVlModel.query()`; persist via `.save("color_image_caption").drain_thread()`
- **VR UX**: text-above-marker via canvas textures; toggleable like images
- **Effort**: medium-high (~250 lines; mostly the captioning pipeline + text→texture renderer)

### 9. Embedding-cluster colored markers
K-means on the CLIP vectors → color each marker by visual cluster (k=6). Similar scenes become visually grouped.

- **API**: pull vectors from `color_image_embedded`, run k-means, encode cluster id per marker
- **VR UX**: marker color = cluster; legend HUD panel with one thumbnail per cluster
- **Effort**: medium (~150 lines, mostly the legend UI)

### 10. Live mode — watch a robot's memory grow
If a robot is recording right now, `.live(buffer)` tails new observations. The VR world updates in real time.

- **API**: `image_stream.live().subscribe(...)` server-side, push `MSG_IMAGE_POSES_APPEND` to clients
- **VR UX**: new ring markers fade in as the robot drives
- **Effort**: medium-high — needs a live `SqliteStore`; robot must be recording. Strong demo

### 11. Per-pose pose-frustum view
Render each capture as a small camera frustum wireframe with the thumbnail at the back, oriented exactly by `obs.pose` quaternion. Sketchfab-style photogrammetry inspector.

- Same data, different geometry — clearer than the current flat quad
- **Effort**: low-medium (~70 lines in `scene.js`)

### 12. Quality-filtered markers
Only show the best image per spatial region using `QualityWindow(quality_fn=lambda o: o.tags["sharpness"])` (`transform.py:416`). Drops blurry/dark frames.

- Combine with keyframe sampling for a hand-picked set
- **Effort**: trivial (~10 lines, server-only)

---

## Tier 4 — small QoL additions

### 13. "Drop a pin" — user-created waypoints
While in VR, pinch + voice → add an observation to a `notes` stream with current pose + text + screenshot. Persists across sessions via `stream.save()`.

- **Effort**: medium (~200 lines; voice transcription is the hardest. MVP: controller-text "pin only")

### 14. Heatmap of robot dwell time
Histogram `odom` positions onto a 2D grid → colored overlay on the existing ground costmap. "The robot spent a lot of time here."

- **API**: pure numpy histogram of `[obs.pose[:2] for obs in odom.transform(throttle(0.1))]`
- **Effort**: low (~60 lines, server-only — reuses the existing top-down render code)

### 15. Two-store A/B mode
Load two `SqliteStore`s side by side ("before move" vs "after move"); each renders its own colored point cloud.

- **Effort**: low-medium — mostly config; messaging is already per-store

---

## Recommended next-build order

For maximum **wow per effort**:

1. **#2 Keyframe sampling** — 30 lines, immediately visible improvement in marker distribution
2. **#6 Voxel-map render** — 40 lines, the world will look noticeably cleaner
3. **#1 Semantic search** — the headline feature; even with a fixed 8-query menu it's a step-change in capability

For pushing the **GTA / immersion feel** further:

4. **#5 Spatial proximity auto-thumbnails** — makes walking through the world feel alive
5. **#4 Trajectory fly-through** — converts the memory into a movie you can step into
