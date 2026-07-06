# Point-cloud streaming to the browser — thin/code the geometry (2026-07-03)

Follows the research note (`docs/point-cloud-streaming-research.md`). A raw `PointCloud2` is heavy
(20k pts × 16 B = **320 KB/frame ≈ 3.2 MB/s @ 10 Hz**), rides the bulk lane, and dies under loss on
WS/WebRTC (the 2026-07-03 transport matrix). This ships a server-side **cloud plane** that hands the
browser a browser-appropriate representation instead of the raw firehose, and measures it end-to-end.

## What was built

- **`gateway/cloud.py`** — a Bus consumer (like `media.py`) that, for every source `PointCloud2`,
  republishes derived variants **onto the bus** so they flow through every existing transport (WS /
  WT / WebRTC), lane, and the last-value cache — no new endpoint:
  - **`<topic>_ds`** — stride-decimated to `CLOUD_DS_MAX_POINTS` (default 2000). Still a **standard
    `PointCloud2`** (re-encoded via `lcm_encode`, `frame_id`/seq preserved) → the existing
    `@dimos/msgs` decode + `WorldView` render work **unchanged**. Guaranteed win, lossy decimation.
  - **`<topic>_draco`** — full-resolution, Draco-quantized (`DracoPy`, `quantization_bits=11`), on the
    custom wire type `draco.PointCloud2`, payload `[u32be seq][u32be nPoints][f64be ts][draco bytes]`.
    Keeps all points at reduced precision. Off (degrade to `_ds` only) if `DracoPy` is absent.
- **`packages/web/src/draco.ts` + `client.ts`** — a client branch on `draco.PointCloud2`: a cheap,
  dependency-free envelope parse (`decodeDracoEnvelope`) so the bench measures it (seq/size/latency)
  with **zero new deps**; `decodeDracoGeometry` lazily loads `draco3d` for rendering only (best-effort,
  returns null if absent, so a missing wasm decoder never breaks the data path).
- **`packages/web/src/bench.ts`** — `cloud`, `cloud-ds`, `cloud-draco` workload profiles.

## Measured — localhost, clean, 15 s/cell, live 10 Hz cloud publisher

| transport | variant | Hz | kB/s | vs raw | deliv% | p50 ms |
|-----------|---------|----|------|--------|--------|--------|
| **WS** | cloud (raw)   | 9.33 | 2917 | 1.0×  | 100 | 1.8 |
| **WS** | cloud-ds      | 9.33 |  293 | **10.0×** | 100 | 2.2 |
| **WS** | cloud-draco   | 9.27 |  477 | **6.1×**  | 100 | 9.8 |
| **WT** | cloud (raw)   | 9.33 | 2918 | 1.0×  | 100 | 3.5 |
| **WT** | cloud-ds      | 9.33 |  293 | **10.0×** | 100 | 1.9 |
| **WT** | cloud-draco   | 9.33 |  481 | **6.1×**  | 100 | 9.4 |

Raw wire sizes (probed): raw **320174 B/frame** → ds **32177** (10.0×) → draco **52780** (6.1×).

### Draco ratio depends on structure (why a codec, not just decimation)
`DracoPy` quant-11 encode of one 20k-pt cloud: **random 6.1× / smooth-surface (lidar-like) 7.8×**.
Real sparse/voxel-downsampled lidar compresses more still (spatial coherence); the bench cloud is
deliberately *random* (incompressible — so deflate can't cheat throughput), which **understates** Draco.

## The honest tradeoff (the story for Lesh)

- **Downsample** wins raw bytes (10×) but **drops 90% of points** — great for a teleop overview, wrong
  for mapping/costmap input.
- **Draco** keeps **all** points at reduced precision (~cm), 6–8×, but costs **~8 ms encode latency**
  at the gateway (software `DracoPy`; p50 9.8 vs 1.8 ms for raw/ds) — a CPU-for-bandwidth trade a
  robot with a hardware encoder wouldn't pay.
- Both cut the 3.2 MB/s raw cloud to **~0.3–0.5 MB/s** — the difference between "fits a cellular
  uplink" and "doesn't." The right pick is per-consumer (viewer → ds; mapping → draco).
- Transport still matters: the ratio is transport-independent, but a compressed cloud on a **WT/QUIC
  lane** is the combination that survives loss + keeps the control lane isolated (per the transport
  matrix). Compression and lane-choice are complementary, not either/or.

## Verified end-to-end
Publisher (`/load/cloud` @ 10 Hz) → gateway transcode (`_ds` + `_draco`) → browser. The existing
`WorldView` renders the cloud (height-colored points, canvas draw confirmed); all three variants are
discovered with correct types incl. the custom `draco.PointCloud2`. WS + WT full parity.

## Live 3-way visual comparison (2026-07-05, real dog lidar)
A **Clouds tab** (`app/src/panels/clouds/CloudCompare.tsx`, `?tab=clouds`) renders the SAME live
`PointCloud2` three ways side-by-side, sharing one auto-fit view, each captioned with live kB/s +
point count. Source is the dog sim (`--simulation mujoco run unitree-go2` → real fused `/lidar` @ 2 Hz;
the cloud plane auto-transcodes it — no gateway change). Measured on `/lidar` (headless, WT):

| cell | wire | points | vs raw |
|------|------|--------|--------|
| Raw (`/lidar`) | 267 kB/s | 8,336 | 1.0× |
| Downsampled (`/lidar_ds`) | 54 kB/s | 1,668 | **≈5×**, visibly sparse |
| Draco (`/lidar_draco`) | 36 kB/s | 8,335 | **≈7.4×**, full density ≈ raw |

The visual is the "why a codec, not just decimation" proof: downsample drops 80–90% of points (sparse);
**Draco keeps every point at reduced precision and is even smaller than the sparse downsample**, so it
looks like the raw scene at 7.4× less bandwidth. In-browser Draco decode is real (draco3d wasm,
`app/src/panels/clouds/dracoDecode.ts`, supplied `wasmBinary` so the emscripten glue never touches
node `fs`); production `vite build` emits the wasm as an asset and succeeds.

### 3D orbit viewer + frame-sync fixes (2026-07-06)
The Clouds tab is now a **shared 3D orbit view** (`app/src/panels/clouds/cloudGL.ts` — dependency-free
`gl.POINTS` + a hand-rolled mat4/orbit camera, no three.js): drag any cell → all three rotate together,
GPU renders ALL points (no 4000 cap). This fixes the top-down flatten that aliased a depth scan into a
grid. Two correctness fixes shipped with it:
- **Draco un-frozen.** The gateway derives the Draco envelope seq from `frame_id` (`_seq_of`), but a
  real lidar's `frame_id` is `"world"` → seq `0` every frame, so the old "decode only when seq changes"
  gate decoded once and froze (why Draco read 8,329 pts vs raw 7,702). Now the client decodes per **new
  blob** (Uint8Array identity) — live for any source.
- **Frame-synced by source `ts`.** Each cell buffers recent frames keyed by the source timestamp (raw/ds
  via `meta.srcTs`, draco via the envelope `ts`, which the gateway stamps identically); each frame the
  three cells render the newest `ts` common to all three (fallback to each-latest, never stall). Result:
  Raw and Draco now show the **same scan with equal point counts** (verified headless: 8,102 == 8,102).
Falls back to the 2D top-down render if WebGL is unavailable; headless verified via SwiftShader.

### WorldView lidar encoding toggle (2026-07-06)
WorldView's lidar layer has a `raw | ds | draco` toggle (`app/src/panels/WorldView.tsx`) — switch the
nav-map lidar between `/lidar`, `/lidar_ds`, `/lidar_draco`; the LayerChip kB/s reflects it live
(≈259 → 52 → 34 kB/s), and the Draco option decodes in-browser via the same path as CloudCompare.
WorldView stays the 2D nav map (its purpose); only the source encoding changes.

### three.js viewer + UX redesign (2026-07-06)
The Clouds tab moved from hand-rolled WebGL to **three.js** (`app/src/panels/clouds/cloudThree.ts`,
replaces `cloudGL.ts`): one shared `PerspectiveCamera` + damped `OrbitControls` drives all three cells
(drag any → all orbit), a `GridHelper` floor gives depth, and a small `ShaderMaterial` keeps the
blue→amber height ramp on crisp round points. three is **lazy-loaded/code-split** — `three.module.js`
(177 KB gzip) + `OrbitControls.js` load only when the tab opens; the main chunk is unchanged (146 KB
gzip). Falls back to the 2D `drawCloud` path if WebGL is unavailable.
The comparison was redesigned to the app's design system (was hardcoded hex + 11px inline styles):
titled cards with a **semantic dot** per encoding (raw = cyan baseline, downsampled = amber/lossy,
Draco = green/best), the `.metric` treatment for live **kB/s**, a **compression badge** (`↓4×` / `↓8×`),
a height legend, and a segmented `raw|ds|draco` control shared with WorldView (`.enc-toggle`). Deps:
`three ^0.169` + `@types/three` (three r169 ships no bundled types). Verified headless (SwiftShader):
3 cells render + orbit together, frame-synced counts (raw == draco), badges live.

### WorldView 3D scene (2026-07-06)
WorldView gained a **2D / 3D toggle** (`app/src/panels/worldThree.ts` + `WorldView.tsx`). The 2D
top-down nav map stays default (best for precise click-to-goal + follow-robot); 3D is an orbit scene
fusing every layer: lidar cloud (height ramp, honoring the raw/ds/draco toggle), laser scan, the
**costmap as a canvas-textured ground plane**, the path line, an **oriented robot marker** (a
`0.7×0.31×0.4 m` box from the Go2 URDF + heading cone — no browser mesh exists) with a trail, and a
goal ring. Camera follows the robot; **click-to-goal raycasts the z=0 plane** → `client.navigate`.
Reuses the shared point shader (`clouds/pointsShader.ts`) + the cloudThree camera/orbit/grid pattern;
three.js loads lazily **only when 3D is first opened**, staying code-split (main chunk +2.4 KB gzip
only). Verified headless (SwiftShader): all layers render, orbit works, 2D↔3D toggles.

## Gaps / pending
- **WebRTC (local)**: ICE fails on the Mac LAN (`webrtc-ice: no such remote` — the v4-mapping issue
  from the sidecar notes); ratios are transport-independent (identical on WS/WT) so this is a local
  connectivity gap, not a cloud-plane issue. WebRTC's bulk collapse under load is already characterized.
- **netem loss matrix**: needs a Linux VPS (macOS has no `tc`); contabo SSH was fail2ban-blocked.
  The compression win is proven clean; the "survives loss better because smaller" matrix is pending.
- **WT last-value replay**: over WebTransport a newly-subscribed slow/stalled topic shows empty until
  its next frame (WS replays the LVC on subscribe; the WT path doesn't). A *continuously* publishing
  source (mujoco @2 Hz, cloudpub @10 Hz) makes the Clouds tab work on any transport; a stalled replay
  only renders over WS. Adding LVC-replay to the WT path would remove this wart — noted, not built.

## Reproduce
```
# 1. publisher (any PointCloud2 on /load/cloud; or GO2Load start_bench(hz,bytes,"cloud"))
DIMOS_TRANSPORT=zenoh uv run python scratchpad/cloudpub.py     # standalone 10 Hz publisher
# 2. gateway+sidecar (with cloud.py) + app
deno task serve   ;   deno task app
# 3. headless bench
deno run -A scripts/bench-headless.ts \
  "http://localhost:5173/?gw=localhost:8080&transport=ws&profiles=cloud,cloud-ds,cloud-draco&net=clean&dur=15000&drive=0&run=1"
```

Live 3-way visual (real dog lidar):
```
# 1. a continuously-publishing structured source (mujoco opens no egl on mac → CGL):
DIMOS_TRANSPORT=zenoh uv run dimos --simulation mujoco --mujoco-headless run unitree-go2   # /lidar @2Hz
#   (or --replay run unitree-go2 for recorded MID360, but replay stalls → use WS; or cloudpub.py)
# 2. deno task serve  ;  deno task app
# 3. open the Clouds tab:
open "http://localhost:5173/?gw=localhost:8080&tab=clouds"
```
Env knobs (`gateway/cloud.py`): `CLOUD_DS_MAX_POINTS` (2000), `CLOUD_DRACO` (1), `CLOUD_QUANT_BITS` (11).
