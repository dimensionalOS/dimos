# Microduck public launch plan

This is the planning record. See [public-launch.md](public-launch.md) for the
implemented architecture, measured load and current deployment status.

Prepared 2026-09-09. Planning only. Hardware, current occupancy, existing relay counters and AWS prices were read without modifying the running services. No public infrastructure was provisioned and no new load test was run.

## Recommended first release

One shared match, with six controllable ducks and additional read-only spectators. The user confirmed this scope and selected GitHub sign-in. Keep the simulation on Omarchy initially, publish the existing website on a public HTTPS domain, and add a public WebSocket transport. dimTELE is optional and is not a dependency for this release.

The user requires MuJoCo to remain responsible for all physics and DimOS to run the robot stack. Simulation, sensor capture, perception, policies, teleop, mapping and the human CLI therefore remain composed through DimOS on Omarchy. Cloudflare handles website infrastructure and transport. Three.js displays server state and cosmetic UI. Neither Cloudflare nor the browser becomes a second physics authority.

Preserve the six ducks, teams, locker-room spawns, smaller benchmark rooms, names, picker animation, follow camera, teleop, policies, human CLI, scoreboard behavior and current ball physics. Three.js continues drawing the world in each browser. MuJoCo remains authoritative on the server. Do not add kickoff rules, automated resets or kicks as part of hosting or perception.

Proposed first capacity target: six players plus 20 spectators. This is a load-test target and proposed admission limit, not a measured maximum. Enable that many connections only after the combined workload passes the launch checks.

Pending product decisions:

- Public hostname. `sim.tule.world` was proposed in the earlier hosting document, but no public route has been configured.
- Spectator authentication. GitHub sign-in is confirmed for players. Requiring it for spectators too is now the proposed default for reducing anonymous connection abuse; this scope is awaiting the user's answer. The landing page and previews can remain public.
- Camera visibility. Controller access is included. Spectator access to duck cameras is awaiting the user's answer, and should never expose private chat, maps or commands.

## Product definitions for implementation

| Definition | Decision or proposed first-release behavior |
| :--- | :--- |
| Identity | GitHub sign-in, confirmed. Use GitHub's immutable user ID internally; keep the duck's custom display name separate from account identity. |
| Entry flow | Public landing page and all six animated duck previews. Pick a duck, sign in with GitHub if needed, enter the duck name at connection, then atomically claim the slot. Recheck availability after sign-in and offer another duck or spectator mode if it was taken. |
| Roles | Player, spectator and operator. Players control only their claimed duck. Spectators receive permitted read-only streams. Operators can release seats, revoke sessions and block abusive accounts. |
| Seats | One active duck per GitHub account and one controller per duck. Keep the existing 60-second reconnect grace and generation isolation. Additional visitors can watch when the six seats are occupied. |
| Screens and states | Landing/picker, sign-in callback/error, connection/name prompt, cockpit, spectator view, and a small operator view. Explicit connecting, occupied, reconnecting, server-offline and capacity-reached states. |
| Persistence | Store account identity, preferred display name, sessions, account restrictions and usage totals. Preserve existing per-session robot knowledge isolation; returning with the same GitHub account does not automatically load a previous player's memory. |
| Anti-spam | Enforce account and connection limits, name validation, request limits and agent spending budgets on the server. GitHub authentication provides an identity to enforce against; it does not prove that a visitor is human or prevent multiple accounts. |
| Perception | YOLO runs on Omarchy as a DimOS perception component, using native MuJoCo head-camera RGB. Start with 2D ball detection and the current 3 Hz source rate. |
| Hosting | Proposed Cloudflare Workers, one match-scoped Durable Object relay, and D1 for account/session records. Omarchy initiates authenticated outbound WSS connections. |
| Operations | Separate staging and production configuration, versioned scene/model assets, monitoring, rollback and an admission limit established by the combined load test. |

GitHub sign-in needs an application registration owned by the chosen GitHub account or organization, a public hostname and exact callback URL. Request only identity access; repository permissions are unnecessary for this product. Exchange the authorization code on the server, use state and PKCE, revalidate the GitHub identity, and issue the site's own secure HttpOnly session cookie. Keep GitHub tokens out of browser storage and out of robot command messages. A framework auth integration can implement this flow without adding another identity provider. [GitHub authorization flow](https://docs.github.com/en/apps/oauth-apps/building-oauth-apps/authorizing-oauth-apps)

The minimal data contracts are `User`, `Session`, `DuckLease`, `WorldSnapshot`, `CameraFrame` and `BallDetections`. A lease binds the GitHub user ID, world ID, duck ID and runtime generation. A camera/detection pair binds that same duck and generation to a frame ID and capture timestamp. Define command acknowledgments, expiry and reconnection behavior before implementing the public relay.

## Hosting architecture

```text
Browser
  Three.js world, GitHub sign-in, cockpit and detector panel
       |
Cloudflare Worker
  Static assets, authentication endpoints and session validation
  D1: account/session records and account restrictions
       |
Cloudflare Durable Object, one per match
  Authenticated WSS relay, read-only world distribution, bounded queues
       |
Authenticated outbound WSS connection initiated by Omarchy
  DimOS transport adapter and authoritative duck ownership
  DimOS MuJoCo world and six locomotion policies
  Occupied ducks' sensors, maps, agents and human CLI
  Shared DimOS perception worker using native head-camera RGB
```

The proposed Cloudflare Worker serves the application and auth endpoints; a match-scoped Durable Object accepts the browser connections and Omarchy's authenticated publisher connection. Cloudflare documents this WebSocket server capability. D1 provides managed SQL storage for account/session records. Visitors need only their browser, and Omarchy needs outbound connectivity. Public router port forwarding and a Cloudflare Tunnel are not required for this proposed relay path. [Workers Static Assets](https://developers.cloudflare.com/workers/static-assets/), [Durable Object WebSockets](https://developers.cloudflare.com/durable-objects/best-practices/websockets/), [D1](https://developers.cloudflare.com/d1/)

Keep transport connections scoped to the match and separate private duck streams from the shared world feed. Authentication can issue a bounded identity grant, but Omarchy remains the final authority for duck claims and accepted commands. The relay must obtain an acknowledged claim and generation from Omarchy, never allocate seats from an independent cached roster. If Omarchy disconnects, show the world as offline/stale and accept no movement until fresh ownership and state are available. Do not save every pose or camera frame to the account database.

A self-managed CPU server running the same WSS relay remains a fallback if Cloudflare's latency or limits fail the prototype. The earlier EC2 relay proposal is therefore an alternative, not an additional required server. Measure relay placement, private-link latency and restricted-network behavior before selecting the final public path.

The existing gateway is deliberately restricted to loopback or Tailscale addresses and forwards WebTransport over UDP. Putting its web page behind an HTTPS proxy alone will not make the live simulation work publicly. Implement an optional WSS adapter that preserves the current protocol, lobby authorization and reconnect behavior. Keep the private gateway mode intact. Generic SDK/relay changes belong in the appropriate framework repository; Microduck deployment, admission and UI changes belong in this application.

Separate control traffic from camera and bulk map transfers, discard superseded state/video before transmission, and bound queued bytes. On reconnect, request fresh state and revalidate ownership; do not replay old movement commands. Keep the existing movement deadman behavior.

The public relay should distribute one upstream world feed to many viewers. This is additional implementation work: a plain reverse proxy forwarding one connection per visitor would still multiply Omarchy's upload use. Keep duck commands and private data scoped to the controller's current session and generation. Internal endpoints and MCP ports remain private; preserve the human CLI through the existing DimOS tool route with authenticated access.

Cloudflare Tunnel is an alternative for direct HTTPS/WSS ingress to an origin. It does not itself provide the shared relay or a transparent browser UDP path. For the initial 3 Hz camera panel, prototype bounded binary JPEG messages with matching detection metadata on a separate WSS media connection. Use an appropriate paid Developer Platform configuration and confirm its media use, traffic limits and account pricing before deployment; do not assume a free CDN/tunnel is an unrestricted video service. If smoother video becomes a requirement or TCP delivery fails the latency target, use Cloudflare Realtime SFU/WebRTC for camera tracks while retaining DimOS and the same ownership rules. That is a separate media adapter, not a change to physics. [Cloudflare routing](https://developers.cloudflare.com/tunnel/routing/), [service terms](https://www.cloudflare.com/service-specific-terms-application-services/), [Realtime](https://developers.cloudflare.com/realtime/)

Budget the Worker, Durable Object messages/active duration and storage separately. Platform connection ceilings do not establish our tested capacity; continuous world traffic also should not be costed as an idle, hibernating relay. No Cloudflare resources, subscriptions or identity applications have been created in this planning work. [Durable Object pricing](https://developers.cloudflare.com/durable-objects/platform/pricing/)

## Football detection feed

Use the duck's actual native RGB sensor image as the detector input. The current source is 640 by 360 pixels at a configured maximum of 3 frames per second per active duck. A cockpit subscription cap of 6 Hz does not increase that source rate. Six occupied ducks therefore offer up to 18 source images per second for detection.

Start with the existing DimOS detector integration and measure accuracy before choosing different weights. A GitHub API review of DimOS checkpoint `e0676c6` verified `Yolo2DDetector`, which imports Ultralytics, defaults to `yolo11n.pt` and selects CUDA when available. `Detection2DModule` consumes `color_image` and publishes typed `Detection2DArray` messages. The Microduck blueprint does not yet wire this detector. Verify compatibility with the application's exact deployed dependency pin during implementation. [DimOS YOLO adapter](https://github.com/dimensionalOS/dimos/blob/e0676c6/dimos/perception/detection/detectors/yolo.py), [DimOS detection module](https://github.com/dimensionalOS/dimos/blob/e0676c6/dimos/perception/detection/module2D.py)

Ultralytics provides model weights, image inference, boxes/class scores, training/evaluation tools and export to runtimes such as ONNX and TensorRT. In this architecture it is a library used by DimOS's perception component. Local inference does not require sending camera frames to an Ultralytics cloud service. It supplies the object-recognition implementation, while DimOS manages the robot pipeline, MuJoCo supplies physics and sensor rendering, and Three.js displays the website. [Prediction](https://docs.ultralytics.com/modes/predict/), [Training](https://docs.ultralytics.com/modes/train/), [Export](https://docs.ultralytics.com/modes/export/)

COCO provides the broader `sports ball` class, not a football-specific classifier. Test the pitch balls and original benchmark ball explicitly, including small, distant, occluded and moving appearances. Fine-tune on representative rendered images if the pretrained model misses them. [Ultralytics COCO documentation](https://docs.ultralytics.com/datasets/detect/coco/)

Load one model in a shared DimOS-managed worker. Feed it bounded per-duck queues that retain the newest frame, with fair scheduling between ducks. Detection must run outside the physics stepping loop and must not block locomotion, sensors or teleop. Stop processing inactive duck sessions and clear their results when a new player claims the slot. A spectator never creates another copy of inference.

The inspected YOLO adapter calls `track(..., persist=True)`. Do not interleave six independent camera streams through that persistent tracker: it would share temporal identity state across ducks. For the initial bounding-box feature, use stateless prediction behind the DimOS detector interface with serialized/batched inference. If tracking is added, maintain separate tracking state per duck and runtime generation while sharing model weights. Preserve DimOS message types and explicitly publish matched full-frame images; the inspected module's `detected_image_*` outputs are object crops, not the requested full-camera overlay. Optional 3D annotations must use measured depth and calibration, not assumed depth.

YOLO operates on image pixels, not inside a renderer. MuJoCo-rendered RGB is recommended because it already comes from the authoritative head camera and continues working with no browser connected. Browser-rendered Three.js RGB could also be analyzed, but it would make perception depend on the viewer's camera, rendering settings and connection, and would require uploading those frames to Omarchy. A separate server-side Three.js camera renderer is possible but would duplicate an existing sensor path. Keep Three.js for the world display and camera-panel overlay.

Publish the source frame together with its duck ID, generation, frame ID, timestamp, dimensions and detections. Draw boxes and confidence in the browser over that exact image, correcting for resizing and letterboxing. A box from the native camera must not be drawn on an independently rendered Three.js image or the following camera. Show `Ball detected`, `No ball detected`, or a stale/unavailable state as appropriate. `No ball detected` is not proof that no ball exists.

Keep the smooth Three.js world view alongside this sensor panel. Begin with the existing sensor rate. If a smoother detection view is wanted, separate RGB capture scheduling from expensive depth and lidar work and benchmark 5 to 10 Hz before raising the source rate. Detector speed alone does not establish camera throughput.

For validation, render segmentation can supply test labels, but deployed detection must consume image pixels rather than simulator ball coordinates. Hold out camera positions and include empty views, balls behind walls, white duck feet, field lines and multiple balls. Record precision, recall by visible ball size, false positives and end-to-end frame age. Do not promise detection accuracy before this evaluation.

Choose code and weight licenses before integration. Ultralytics offers AGPL-3.0 and an enterprise option; resolve their fit with the application's intended distribution instead of assuming that every YOLO implementation and checkpoint has the same license. [Ultralytics licensing options](https://www.ultralytics.com/license)

## Omarchy capacity evidence

Read-only inventory:

| Component | Omarchy |
| :--- | :--- |
| CPU | Intel Core i9-10900, 10 physical cores, 20 hardware threads |
| RAM | 64 GB installed, about 62 GiB reported by the OS |
| GPU | NVIDIA GeForce RTX 3070 Ti, 8,192 MiB VRAM |
| NVIDIA driver | 610.57.04 |
| Current occupancy | Zero players and zero viewers at inspection |
| Current world service memory | About 2.8 GiB, while unoccupied |

The earlier six-player validation advanced simulation time by 10.005 seconds during 10.000 seconds of wall time. This establishes real-time operation during that bounded test, not sustained public capacity with YOLO enabled. The low GPU utilization and memory snapshot above are idle observations and cannot establish remaining capacity under six active players. [Existing six-duck validation](football-club.md)

Each occupied duck launches its own agent/runtime pipeline, configured with four workers, plus mapping and sensor work. Spectators receive the shared world state and render locally, so player and spectator counts must be measured separately. Hosted language-model API use also needs its own request and spend budget.

Existing relay counters recorded 1,127,451,093 bytes over 183,117 world frames: an accumulated average of about 6,157 bytes per frame. At the configured 30 Hz this projects to 1.48 Mbps per receiving browser, before network overhead, assets, cameras, maps and chat. These are historical counters, not a fresh active-load measurement.

| World-only spectators | Projected world-state payload traffic |
| :--- | :--- |
| 20 | About 30 Mbps |
| 50 | About 74 Mbps |
| 100 | About 148 Mbps |

With edge fanout, this traffic leaves the public relay, while Omarchy sends approximately one shared copy upstream. A transparent proxy alone provides no such saving. Measure the actual uplink and private-link latency before selecting the final architecture and limits.

The current 128-participant retention cap and default 64 UDP-peer cap are separate implementation limits, not capacity benchmarks. A future WSS path needs explicit limits too. Test limits and reconnection headroom deliberately in staging rather than increasing production caps to claim scale.

## AWS equivalent and cost

There is no exact instance equivalent to this desktop CPU and GPU combination. `g6.4xlarge` is the first comparison candidate: similar RAM, fewer CPU threads, and a different GPU with more VRAM. Its L4 is intended for inference and graphics workloads, but this does not establish equivalent MuJoCo, rendering or YOLO performance. AWS lists G6 GPU memory as 24 GB in the product table and approximately 22 GiB in the instance specification. [AWS G6 specifications](https://aws.amazon.com/ec2/instance-types/g6/), [EC2 CPU/core specifications](https://docs.aws.amazon.com/ec2/latest/instancetypes/ac.html)

| Option | vCPUs | RAM | GPU | USD/hour | USD/month at 730 hours |
| :--- | :--- | :--- | :--- | :--- | :--- |
| g6.2xlarge | 8 | 32 GiB | One NVIDIA L4 | 0.9776 | 713.65 |
| g6.4xlarge | 16 | 64 GiB | One NVIDIA L4 | 1.3232 | 965.94 |
| g6.8xlarge | 32 | 128 GiB | One NVIDIA L4 | 2.0144 | 1,470.51 |

Prices were queried directly from the AWS Price List API on 2026-09-09: US East (N. Virginia), Linux, shared tenancy, on-demand, no preinstalled paid software. Returned rates were effective 2026-09-01. Totals exclude storage, public IPv4, data transfer, DNS, monitoring, model API use, taxes and any separate relay. These are reference-region prices; availability, quota, latency and price in the eventual deployment region need confirmation. [AWS Price List Query API](https://docs.aws.amazon.com/awsaccountbilling/latest/aboutv2/using-price-list-query-api.html), [EC2 pricing and additional charges](https://aws.amazon.com/ec2/pricing/on-demand/)

Start an eventual cloud benchmark with g6.4xlarge. Consider g6.2xlarge only if profiling shows that the occupied app fits its CPU and RAM budget. Consider g6.8xlarge only if CPU measurements justify it; its extra CPU/RAM do not add another GPU. Validate native EGL rendering, NVIDIA drivers, the same MuJoCo version, all six policies, camera throughput and detector performance before migrating.

For the first release, reusing Omarchy avoids the full GPU-instance bill. It introduces dependence on local power, internet upload and server uptime. The proposed Cloudflare relay has separate platform costs; the CPU-server fallback has separate compute and traffic costs. Quote the chosen path using measured traffic and the account's existing usage.

## Implementation order and launch checks

1. Establish the scene baseline. The pitch entrance repair is committed as `efc2c9b`, with 145 Python tests passing, including six ducks crossing in both directions. Production remains on `be43a3e`. Deploying that repair and restarting the shared world still needs the previously requested approval; automatic review rejected the restart because it would interrupt the live session. The planning request does not authorize that restart.
2. Add the detector on staging. Prove exact image/box synchronization, six-stream scheduling, accuracy on actual balls and graceful inference failure while physics continues. Do not change the physics engine or contact settings for this feature.
3. Add the public WSS path and world relay. Preserve all existing cockpit streams and permission boundaries. Test simultaneous claims, six active slots, rejected seventh claims, read-only viewers, multiple tabs, stale generations, reconnects and slow connections. The existing per-duck runtime startup and shutdown should remain authoritative.
4. Configure public access after choosing the domain, GitHub application owner and spectator permissions. Implement GitHub sign-in, the account/session store, one-duck-per-account admission, TLS, request limits, agent spend limits, useful busy/offline states and monitoring. Cache immutable scene assets. Verify that a visitor outside the tailnet can load and connect without installing anything.
5. Measure staging capacity with all six players walking and exercising normal policies, sensors, maps and the human CLI, with YOLO on. Add 10, 20, 25 and 50 world-only spectators in stages; stop when latency or resources degrade. If spectator cameras are enabled, test that as a separate profile. Include cold page loads, reconnect bursts and slow clients.
6. Use a 30-minute target-load soak plus external browser checks. Proposed acceptance targets: simulation time stays within 1 percent of wall time without accumulating lag; no stale movement is replayed; p95 input-to-visible-motion is below 200 ms on a suitable nearby test connection; p95 detection-frame age is below 750 ms at the initial 3 Hz; queues remain bounded; and measured CPU, GPU, RAM and upload retain roughly 25 percent headroom. Measure client frame time and field entrance walking as well. These are proposed acceptance targets, not results already obtained.
7. Publish with the largest tested admission limit below the first degraded workload. Keep a known-good deployment for rollback and alert on stopped simulation progress, sensor/inference failures and connection errors. A process merely being alive is not sufficient health evidence.

Raw inventory, counters, AWS SKUs, rate codes and calculated monthly costs are saved in [public-launch-evidence.json](public-launch-evidence.json).
