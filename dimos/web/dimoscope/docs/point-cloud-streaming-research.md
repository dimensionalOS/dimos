# Point-cloud streaming to the browser — research reading list

Context for dimoscope: we can already ship a `PointCloud2` as compressed binary over a data lane
(WT/WS/WebRTC DataChannel), but the *heavy* geometry path is an open design question. Two options
keep coming up:

1. **Cloud-as-video** — project geometry into 2D frames and ride the hardware video encoder + WebRTC
   media plane (V-PCC). Cheap encode, but video compression corrupts geometry at depth
   discontinuities → viz-only, disqualifies SLAM/costmap input.
2. **Cloud-as-geometry** — a point-native codec (G-PCC / Draco) or a downsampled representation
   (splats / voxels / LOD) over a **data lane**, ideally a QUIC lane.

Our own transport benchmark (`docs/benchmarks.md`, `bench-results-2026-07-03.md`) points at option 2
over a QUIC lane: WebTransport wins bulk + is the only transport that survives loss + gives real
per-lane isolation, whereas WebRTC DataChannels share **one SCTP congestion window** and collapse
under loss/contention. The MoQ paper below reaches the same conclusion from the streaming side.

---

## Codec background — V-PCC vs G-PCC vs Draco (the "as-video vs as-geometry" fork)

- **[An overview of ongoing PCC standardization: V-PCC and G-PCC](https://mpeg-pcc.org/wp-content/uploads/2020/04/an_overview_of_ongoing_point_cloud_compression_standardization_activities_videobased_vpcc_and_geometrybased_gpcc.pdf)**
  (APSIPA, open PDF) — the canonical intro. V-PCC projects the cloud into 2D geometry+texture videos
  (HEVC/VVC); G-PCC reduces points directly. This is the decision point.
- **[An Introduction to Point Cloud Compression Standards](https://dl.acm.org/doi/10.1145/3599184.3599188)**
  (ACM GetMobile) — shorter, digestible survey.
- **[Standardization status of MPEG G-PCC Edition 2](https://ieeexplore.ieee.org/iel8/10566276/10566278/10566443.pdf)**
  (IEEE) — current state; G-PCC edition 1 published Mar 2023.

## Robot teleoperation streaming — the bullseye for dimoscope

- **[Seeing Through the Robot's Eyes: Adaptive Point Cloud Streaming for Immersive Teleoperation](https://link.springer.com/content/pdf/10.1007/978-3-032-03805-0_1.pdf)**
  (Springer, open PDF) — robot→operator cloud streaming with bandwidth adaptation. Closest paper to
  what dimoscope is doing.
- **[Toward Optimal Real-time Dynamic Point Cloud Streaming over Bandwidth-constrained Networks](https://dl.acm.org/doi/10.1145/3595916.3626429)**
  (ACM MM-Asia) — rate adaptation under a bandwidth cap; matches our netem-profile approach.
- **[LEAN-3D: Low-latency Hierarchical Point Cloud Codec for Mobile 3D Streaming](https://arxiv.org/pdf/2604.04737)**
  (arXiv) — the "downsample / hierarchical LOD at the gateway" path (splats / voxels).

## Transport-specific — ties directly to our WT-vs-WebRTC result

- **[Point Cloud Streaming with Latency-Driven Implicit Adaptation using MoQ](https://arxiv.org/html/2507.15673)**
  (arXiv, 2025) — **Media over QUIC**. The exact QUIC-lane direction our benchmark points to; read
  this one first for transport.
- **[Scalable MDC-Based Volumetric Video Delivery for Real-Time One-to-Many WebRTC Conferencing](https://www.spirit-project.eu/wp-content/uploads/sites/85/2024/03/Scalable-MDC-Based-Volumetric-Video-Delivery-for-Real-Time.pdf)**
  (ACM MMSys 2024, open PDF) — cloud *through* the WebRTC media plane, one-to-many with an SFU. The
  "stream the cloud through the media channel" idea, done for real (accepts video-codec geometry loss).
- **[Transcoding V-PCC Point Cloud Streams in Real-time](https://dl.acm.org/doi/10.1145/3682062)**
  (ACM TOMM) — practical V-PCC pipeline latency.

---

## Takeaway for the design

- The **MoQ paper + our benchmark** agree: geometry wants a **QUIC lane**, not an SCTP DataChannel.
- The **WebRTC-MDC paper** is the counter-case: video-plane volumetric works *if* you accept
  video-codec geometry loss + run an SFU (LiveKit/mediasoup). Fine for a viewer, not for SLAM input.
- The **V-PCC/G-PCC overview** is the codec fork; the cheap near-term dimoscope move is
  **downsample/voxelize/splat at the gateway** (LEAN-3D style, viz-only), with G-PCC/Draco as the
  structured-lossless upgrade path.

Related repo docs: `docs/benchmarks.md`, `bench-results-2026-07-03.md`.
