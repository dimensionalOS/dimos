"""End-to-end stack smoke test for the memory2-native perception, bypassing the agent.

What it verifies:
  1. ``recording_xarm6.db`` exists and has streams.
  2. CLIP is continuously embedding (``color_image_embedded`` stream growing).
  3. CLIP semantic search matches a text prompt to a recorded frame.
  4. Moondream VLM produces bbox detections on that frame.
  5. ``Object.from_2d_to_list`` projects to 3D against aligned depth + intrinsics.

Run while ``dimos run coordinator-xarm6 xarm6-perception-agent`` is alive (so the
recorder is actively writing). It writes ``perception_diag.log`` in the cwd.

Usage:
    python dimos/manipulation/memory2/test_perception_stack.py [PROMPT] [DB_PATH]

    python dimos/manipulation/memory2/test_perception_stack.py "cup"
    python dimos/manipulation/memory2/test_perception_stack.py "the red mug" recording_xarm6.db
"""

from __future__ import annotations

import sys
import time
from contextlib import redirect_stdout
from pathlib import Path
from typing import Any

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection3d.object import Object as DetObject

LOG = Path("perception_diag.log").resolve()


def hr(title: str) -> None:
    print()
    print("=" * 78)
    print(f"== {title}")
    print("=" * 78)


def run(prompt: str, db_path: str) -> None:
    hr("A. DB file + streams")
    db = Path(db_path).resolve()
    if not db.exists():
        print(f"NOT FOUND: {db}")
        print("Is the recorder running? Check: `dimos run coordinator-xarm6 xarm6-perception-agent`")
        return
    print(f"db: {db}  size: {db.stat().st_size / 1e6:.1f} MB")

    from dimos.memory2.store.sqlite import SqliteStore

    # read-only access; recorder still has its own connection
    store = SqliteStore(path=str(db))
    store.start()
    streams = store.list_streams()
    print(f"streams found: {streams}")

    # Show the codec each stream was created with (persisted in the registry).
    # depth_image MUST be a lossless codec (lz4+lcm / lcm). If it shows 'jpeg',
    # the recorder.py codec fix is NOT deployed, OR the DB predates the fix.
    print()
    print("per-stream codec (from registry — the load-bearing line for depth):")
    try:
        for name in ["color_image", "depth_image", "camera_info", "color_image_embedded"]:
            stored = store._registry.get(name)
            cid = stored.get("codec_id") if stored else "(not registered)"
            flag = ""
            if name == "depth_image":
                flag = "  <-- MUST be lossless (lz4+lcm/lcm). 'jpeg' = fix not deployed."
            print(f"  {name}: codec={cid}{flag}")
    except Exception as e:
        print(f"  codec introspection failed: {e}")

    expected = ["color_image", "depth_image", "camera_info", "color_image_embedded"]
    newest_ts: dict[str, float] = {}
    for name in expected:
        if name not in streams:
            print(f"  MISSING: {name}")
            continue
        try:
            st = store.streams.__getattr__(name)
            cnt = sum(1 for _ in st.order_by("ts"))
            last = st.order_by("ts", desc=True).first()
            newest_ts[name] = last.ts
            age = time.time() - last.ts
            print(f"  {name}: {cnt} obs | newest ts={last.ts:.1f} ({age:.1f}s ago)")
        except Exception as e:
            print(f"  {name}: count/recency failed ({e})")

    hr("A2. staleness — is the embed pipeline keeping up with the camera?")
    if "color_image" in newest_ts and "color_image_embedded" in newest_ts:
        lag = newest_ts["color_image"] - newest_ts["color_image_embedded"]
        print(f"newest color_image           : {newest_ts['color_image']:.1f}")
        print(f"newest color_image_embedded  : {newest_ts['color_image_embedded']:.1f}")
        print(f"embed lag behind live camera : {lag:.1f}s")
        if lag > 30:
            print("  -> STALE. The embed pipeline is NOT producing recent embeddings.")
            print("  -> find_objects searches recent frames first; if the newest embedding")
            print("     is minutes/hours old, it can never match what the camera sees NOW.")
        elif lag > 5:
            print("  -> Lagging but alive. Pipeline can't keep up with frame rate.")
        else:
            print("  -> Fresh. Embed pipeline is current with the camera.")

    hr("B. CLIP semantic search for prompt")
    print(f"prompt: {prompt!r}")
    from dimos.models.embedding.clip import CLIPModel

    clip = CLIPModel()
    clip.start()
    t0 = time.perf_counter()
    vec = clip.embed_text(prompt)
    print(f"text embed: {time.perf_counter() - t0:.3f}s  dim={getattr(vec, 'shape', '?')}")

    try:
        embedded = store.streams.color_image_embedded
        results = list(embedded.search(vec).order_by("ts", desc=True).limit(5))
    except Exception as e:
        print(f"search failed: {e}")
        print("  -> color_image_embedded stream missing or no embeddings written yet.")
        print("  -> Confirm the recorder logged 'continuous CLIP embed pipeline active'.")
        return
    print(f"top-5 matches (most recent first):")
    if not results:
        print("  (no matches at all — CLIP embed pipeline not writing? Check brightness/sharpness filters)")
        return
    for i, obs in enumerate(results):
        sim = getattr(obs, "similarity", None)
        pose = obs.pose
        print(f"  #{i}  ts={obs.ts:.3f}  sim={sim}  pose={pose}")

    most_recent = results[0]
    print(f"\npicking #0 for VLM (ts={most_recent.ts:.3f}, similarity={getattr(most_recent, 'similarity', None)})")

    hr("C. depth + intrinsics for that timestamp")
    try:
        depth_obs = store.streams.depth_image.at(most_recent.ts, tolerance=0.1).first()
        info_obs = store.streams.camera_info.last()
        print(f"depth ts={depth_obs.ts:.3f}  (Δ={depth_obs.ts - most_recent.ts:+.3f}s)")
        print(f"info ts ={info_obs.ts:.3f}")
    except Exception as e:
        print(f"depth/info lookup failed: {e}")
        return

    hr("D. Moondream detection on the matched frame")
    from dimos.models.vl.moondream import MoondreamVlModel

    vlm = MoondreamVlModel()
    vlm.start()
    t0 = time.perf_counter()
    try:
        dets_2d = vlm.query_detections(most_recent.data, prompt)
        print(f"VLM call: {time.perf_counter() - t0:.3f}s")
        print(f"detections: {len(dets_2d.detections)}")
        for i, d in enumerate(dets_2d.detections):
            bbox = getattr(d, "bbox", None)
            print(f"  #{i}  bbox={bbox}  name={getattr(d, 'name', '?')}")
    except Exception as e:
        print(f"VLM call failed: {e}")
        return

    if not dets_2d.detections:
        print("  -> VLM returned no detections for this prompt + this frame.")
        print("  -> Try a more visible prompt or move the object into clearer view.")
        return

    hr("E. depth stats in the detection bbox (why does projection fail?)")
    import numpy as np

    from dimos.perception.detection.type.detection3d.object import Object as DetObject

    try:
        depth_cv = depth_obs.data.to_opencv()
        print(f"depth array: shape={depth_cv.shape} dtype={depth_cv.dtype}")
        print(f"depth global: min={np.nanmin(depth_cv):.3f} max={np.nanmax(depth_cv):.3f} "
              f"mean={np.nanmean(depth_cv):.3f}")
        d0 = dets_2d.detections[0]
        x1, y1, x2, y2 = (int(v) for v in d0.bbox)
        h, w = depth_cv.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        roi = depth_cv[y1:y2, x1:x2]
        nonzero = roi[(roi > 0) & np.isfinite(roi)]
        print(f"bbox roi: ({x1},{y1})-({x2},{y2})  pixels={roi.size}")
        if nonzero.size:
            print(f"bbox valid-depth: count={nonzero.size} "
                  f"min={nonzero.min():.3f} max={nonzero.max():.3f} mean={nonzero.mean():.3f}")
            print("  -> If these look like 700-3000, depth is in MILLIMETERS (need depth_scale=1000)")
            print("  -> If these look like 0.7-3.0, depth is in METERS (depth_scale=1.0 is correct)")
        else:
            print("  -> NO valid depth in bbox region. Sensor holes / reflective / out-of-range.")
    except Exception as e:
        print(f"depth-stat failed: {e}")

    hr("E2. 3D projection — production params vs filters-disabled")

    def _project(label: str, **kw):
        try:
            objs = DetObject.from_2d_to_list(
                detections_2d=dets_2d,
                color_image=most_recent.data,
                depth_image=depth_obs.data,
                camera_info=info_obs.data,
                camera_transform=None,
                **kw,
            )
            print(f"[{label}] -> {len(objs)} objects")
            for o in objs:
                print(f"    {o.name} at ({o.center.x:.3f}, {o.center.y:.3f}, {o.center.z:.3f})")
        except Exception as e:
            print(f"[{label}] raised: {e}")

    # 1. exactly what find_objects uses today
    _project("production (max_dist=1.0, scale=1.0, trunc=10)",
             max_distance=1.0, use_aabb=True, max_obstacle_width=0.06)
    # 2. distance filter off — isolates the max_distance guard
    _project("max_distance=0 (filter off)",
             max_distance=0.0, use_aabb=True, max_obstacle_width=0.06)
    # 3. treat depth as millimeters — isolates the unit-scale bug
    _project("depth_scale=1000 (mm->m)",
             depth_scale=1000.0, depth_trunc=10.0, max_distance=0.0,
             use_aabb=True, max_obstacle_width=0.06)
    # 4. both: mm scale + no distance filter + generous trunc
    _project("mm scale + no dist filter + trunc=50",
             depth_scale=1000.0, depth_trunc=50.0, max_distance=0.0,
             use_aabb=True, max_obstacle_width=0.0)
    print()
    print("Whichever variant returns >0 objects tells us the exact fix:")
    print("  - 'max_distance=0' works  -> raise/disable max_distance in LazyPerceptionModuleConfig")
    print("  - 'depth_scale=1000' works -> recorder/realsense emits mm; need depth_scale=1000 in find_objects")
    print("  - none work               -> empty depth in bbox; sensor/exposure issue")

    hr("F. EXACT find_objects() code path (the real query LazyPerceptionModule runs)")
    print("Replicates lazy_perception.py: .search(vec).filter(sim>=min_similarity)")
    print(".order_by('ts',desc=True).first() -> VLM -> from_2d_to_list(config params)")
    print()
    MIN_SIM = 0.20  # LazyPerceptionModuleConfig.min_similarity default
    print(f"min_similarity = {MIN_SIM} (the filter find_objects applies)")
    try:
        filtered = (
            store.streams.color_image_embedded.search(vec)
            .filter(lambda o: (getattr(o, "similarity", 0) or 0) >= MIN_SIM)
            .order_by("ts", desc=True)
        )
        chosen = filtered.first()
        sim = getattr(chosen, "similarity", None)
        print(f"chosen obs: ts={chosen.ts:.3f} sim={sim}  (passed the >= {MIN_SIM} filter)")
    except Exception as e:
        print(f"NO observation passed the min_similarity filter: {e}")
        print(f"  -> Every embedded frame scored < {MIN_SIM} for prompt {prompt!r}.")
        print("  -> THIS is why find_objects returns nothing. Two options:")
        print("     (a) prompt/scene mismatch — try a prompt that matches what the camera sees")
        print("     (b) min_similarity too high for this CLIP model — lower it in spec.py:")
        print("         LazyPerceptionModuleConfig.min_similarity")
        print("  -> Look at section B sims: if the best is e.g. 0.21, a threshold of 0.20")
        print("     barely passes; 0.25+ would reject everything.")
        chosen = None

    if chosen is not None:
        try:
            d_obs = store.streams.depth_image.at(chosen.ts, tolerance=0.1).first()
            i_obs = store.streams.camera_info.last()
            vlm2 = MoondreamVlModel()
            vlm2.start()
            dets = vlm2.query_detections(chosen.data, prompt)
            print(f"VLM on chosen frame: {len(dets.detections)} detections")
            if dets.detections:
                # Mirror the FIXED lazy_perception path: mm->m + foreground band
                _depth_m = _depth_m_for_world(d_obs, detections=dets)
                objs = DetObject.from_2d_to_list(
                    detections_2d=dets,
                    color_image=chosen.data,
                    depth_image=_depth_m,
                    camera_info=i_obs.data,
                    camera_transform=None,
                    max_distance=1.0,
                    use_aabb=True,
                    max_obstacle_width=0.06,
                )
                print(f"find_objects WOULD return {len(objs)} object(s):")
                for o in objs:
                    print(f"  {o.name} at ({o.center.x:.3f}, {o.center.y:.3f}, {o.center.z:.3f})")
                if objs:
                    print()
                    print("  => find_objects() WORKS. If HumanCLI still says 'nothing',")
                    print("     the agent isn't CALLING find_objects (system-prompt issue),")
                    print("     OR it's calling get_scene_info (empty until find_objects runs).")
                else:
                    print()
                    print("  => find_objects() returns []. 3D projection still failing —")
                    print("     check section E depth stats: is depth coherent post-codec-fix?")
            else:
                print("  => VLM found nothing on the chosen frame. find_objects returns [].")
                print("     CLIP matched a frame, but Moondream can't see the object in it.")
                print("     Likely: CLIP matched on background context, not the object itself.")
        except Exception as e:
            print(f"find_objects replication failed at projection: {e}")

    hr("G. LOCALIZATION accuracy — why the pick position is off")
    print("Moondream gives a rectangular bbox (no mask). from_2d_to_list builds a")
    print("rectangular depth mask -> pointcloud = object + background behind it.")
    print("use_aabb=True -> center = midpoint of (object .. background) -> too deep.")
    print()
    try:
        d0 = dets_2d.detections[0]
        x1, y1, x2, y2 = (int(v) for v in d0.bbox)
        # mm->m converted depth (same as the fix)
        dcv = depth_obs.data.to_opencv()
        if depth_obs.data.format == ImageFormat.DEPTH16:
            dcv = dcv.astype(np.float32) / 1000.0
        h, w = dcv.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        roi = dcv[y1:y2, x1:x2]
        valid = roi[(roi > 0.05) & np.isfinite(roi)]
        if valid.size:
            pcts = np.percentile(valid, [5, 10, 25, 50, 75, 90, 95])
            print(f"bbox depth (meters) percentiles:")
            print(f"   5%={pcts[0]:.3f}  10%={pcts[1]:.3f}  25%={pcts[2]:.3f}")
            print(f"  50%={pcts[3]:.3f}  75%={pcts[4]:.3f}  90%={pcts[5]:.3f}  95%={pcts[6]:.3f}")
            raw_aabb_center = (valid.min() + valid.max()) / 2.0
            bounds = _nearest_cluster_bounds(valid)
            print()
            print(f"  RAW (no filter) AABB-center depth ≈ {raw_aabb_center:.3f} m  "
                  f"(spans {valid.min():.3f}–{valid.max():.3f})")
            if bounds is not None:
                near, far = bounds
                cluster = valid[(valid >= near) & (valid <= far)]
                cl_center = (cluster.min() + cluster.max()) / 2.0 if cluster.size else near
                print(f"  CLUSTER kept: [{near:.3f}, {far:.3f}] m  "
                      f"(extent {far - near:.3f} m, {cluster.size} pts)")
                print(f"  CLUSTER AABB-center depth ≈ {cl_center:.3f} m  <-- what find_objects now uses")
                print(f"  correction applied ≈ {abs(raw_aabb_center - cl_center):.3f} m")
                print()
                print("  The cluster extent ADAPTS — it is whatever the object's actual")
                print("  depth span is (tall object top-down = large extent kept; thin")
                print("  object side-on = small extent). No fixed band, no hardcoded depth.")
            else:
                print("  cluster filter skipped (too few points) — projection uses raw bbox")
            spread = pcts[6] - pcts[0]
            print()
            print(f"  (bbox 5%→95% depth spread = {spread:.2f} m; "
                  f"large spread = lots of background the cluster filter removes)")
        else:
            print("  no valid depth in bbox — different problem (sensor/exposure)")
    except Exception as e:
        print(f"localization analysis failed: {e}")

    hr("G2. world-frame position (what the AGENT actually receives to pick)")
    print("Section F used camera_transform=None (camera frame). The real find_objects")
    print("applies the recorded pose -> world/base frame. This is the number the arm")
    print("plans to. Compare it against where the object ACTUALLY is on your bench.")
    try:
        from dimos.manipulation.memory2 import LazyPerceptionModule

        m = LazyPerceptionModule.__new__(LazyPerceptionModule)
        ct = m._camera_transform_from_pose(most_recent.pose)
        print(f"recorded pose (7-tuple): {most_recent.pose}")
        print(f"reconstructed camera_transform: {ct}")
        if ct is not None and dets_2d.detections:
            world_objs = DetObject.from_2d_to_list(
                detections_2d=dets_2d,
                color_image=most_recent.data,
                depth_image=_depth_m_for_world(depth_obs, detections=dets_2d),
                camera_info=info_obs.data,
                camera_transform=ct,
                max_distance=0.0,           # disable filter so we SEE the raw world pos
                use_aabb=True,
                max_obstacle_width=0.06,
            )
            print(f"WORLD-frame objects (max_distance disabled to see raw position):")
            for o in world_objs:
                print(f"  {o.name} at world ({o.center.x:.3f}, {o.center.y:.3f}, {o.center.z:.3f})")
            print()
            print("  Compare these XYZ to the object's real position in the robot base")
            print("  frame. If consistently offset by a fixed vector -> hand-eye calibration.")
            print("  If Z is ~0.2-0.3m too far -> the bbox-background bias from section G.")
        else:
            print("  (no pose on the matched obs, or no detections — can't world-project)")
    except Exception as e:
        print(f"world-frame projection failed: {e}")

    print()
    print("READ THIS:")
    print("  Section B  -> CLIP semantic search proven (similarity numbers)")
    print("  Section D  -> Moondream VLM proven (bbox detections)")
    print("  Section E  -> depth integrity post-codec-fix (coherent values = fix worked)")
    print("  Section F  -> exact find_objects path (camera frame)")
    print("  Section G  -> localization error source: bbox-background bias vs calibration")
    print("  Section G2 -> the WORLD position the agent picks; compare to ground truth")


def _nearest_cluster_bounds(depths, bin_size=0.02, gap=0.10, min_points=50):
    """Mirror of lazy_perception._nearest_cluster_bounds for the diagnostic."""
    if depths.size < min_points:
        return None
    lo = float(np.percentile(depths, 1))
    hi = float(np.percentile(depths, 99))
    if hi <= lo:
        return None
    nbins = max(1, int(np.ceil((hi - lo) / bin_size)))
    hist, edges = np.histogram(depths, bins=nbins, range=(lo, hi))
    occ_thresh = max(3, int(0.005 * depths.size))
    occupied = hist >= occ_thresh
    if not occupied.any():
        return None
    first = int(np.argmax(occupied))
    gap_bins = max(1, int(np.ceil(gap / bin_size)))
    end, empty_run = first, 0
    for b in range(first, nbins):
        if occupied[b]:
            end, empty_run = b, 0
        else:
            empty_run += 1
            if empty_run >= gap_bins:
                break
    return float(edges[first]), float(edges[end + 1])


def _depth_m_for_world(depth_obs: Any, detections: Any = None):
    """mm->m converted depth + foreground CLUSTER filter — mirrors the fixed
    lazy_perception._detect_and_project_one path so the diagnostic reflects
    real find_objects behavior."""
    dcv = depth_obs.data.to_opencv()
    if depth_obs.data.format == ImageFormat.DEPTH16:
        dcv = dcv.astype(np.float32) / 1000.0
    elif dcv.dtype != np.float32:
        dcv = dcv.astype(np.float32)
    dcv = dcv.copy()
    if detections is not None:
        h, w = dcv.shape[:2]
        for det in detections.detections:
            try:
                x1, y1, x2, y2 = (int(v) for v in det.bbox)
            except (TypeError, ValueError):
                continue
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            roi = dcv[y1:y2, x1:x2]
            b = _nearest_cluster_bounds(roi[(roi > 0.05) & np.isfinite(roi)])
            if b is None:
                continue
            roi[(roi < b[0]) | (roi > b[1])] = 0.0
    return Image(
        data=dcv, format=ImageFormat.DEPTH,
        frame_id=depth_obs.data.frame_id, ts=depth_obs.data.ts,
    )


def main() -> None:
    prompt = sys.argv[1] if len(sys.argv) > 1 else "cup"
    db_path = sys.argv[2] if len(sys.argv) > 2 else "recording_xarm6.db"
    LOG.unlink(missing_ok=True)
    with LOG.open("w") as f, redirect_stdout(f):
        print(f"perception_diag — prompt={prompt!r} db={db_path} time={time.strftime('%Y-%m-%d %H:%M:%S')}")
        try:
            run(prompt, db_path)
        except Exception as e:
            import traceback

            print(f"FATAL: {e}")
            traceback.print_exc()
        print()
        print("=== END ===")
    print(f"wrote {LOG}")


if __name__ == "__main__":
    main()
