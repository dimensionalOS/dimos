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

## Gaps / pending
- **WebRTC (local)**: ICE fails on the Mac LAN (`webrtc-ice: no such remote` — the v4-mapping issue
  from the sidecar notes); ratios are transport-independent (identical on WS/WT) so this is a local
  connectivity gap, not a cloud-plane issue. WebRTC's bulk collapse under load is already characterized.
- **netem loss matrix**: needs a Linux VPS (macOS has no `tc`); contabo SSH was fail2ban-blocked this
  session. The compression win is proven clean; the "survives loss better because smaller" matrix is
  the pending VPS step.
- **In-browser Draco render**: needs `draco3d` installed + Vite wasm wiring; today `_draco` is
  measured on the wire and `_ds` is what renders. Installing `draco3d` enables the geometry decode.

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
Env knobs (`gateway/cloud.py`): `CLOUD_DS_MAX_POINTS` (2000), `CLOUD_DRACO` (1), `CLOUD_QUANT_BITS` (11).
