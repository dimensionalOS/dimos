# Zenoh Investigation

Research for [#1726](https://github.com/dimensionalOS/dimos/issues/1726).

## What is Zenoh

Zenoh is a communication protocol and implementation by ZettaScale (the people behind the original DDS/CycloneDDS). Designed for the cloud-to-edge-to-microcontroller continuum. Written in Rust, with bindings for Python, C, C++, and others.

Key difference from LCM/DDS: it unifies pub/sub, request/response, and distributed storage in one protocol, and works across unreliable/heterogeneous networks natively — not bolted on.

## Communication Modes

Three session modes:

- **Peer:** Direct connectivity via multicast scouting (`224.0.0.224:7446`). Similar to LCM's current model but with more features. Peers auto-discover each other and routers via multicast/gossip.
- **Router:** Zenoh routers (`zenohd`) mediate between subnets. Routers **do not auto-discover each other** — every router-to-router link must be explicitly configured via `connect.endpoints`/`listen.endpoints`. Once connected, they exchange full link-state topology and compute spanning trees automatically.
- **Client:** Applications connect to a router as clients. Simplest for constrained devices. Clients auto-discover routers via multicast scouting.

These can be mixed — robots peer locally, connect via router to cloud.

### Router discovery and topology

Routers join the multicast group (`224.0.0.224:7446`) and **respond** to scouting so clients/peers can find them, but they **never initiate connections from scouting**. This is by design — you control the router topology explicitly.

Auto-connect *can* be enabled (`scouting.multicast.autoconnect: { router: ["router"] }`) but is off by default. Once router links are established, routers maintain two link-state networks:
- **Router graph** — full link-state of all routers (always on)
- **Linkstate-peer graph** — peers operating in linkstate mode (optional)

Link-state carries **topology only** (who's connected to whom, with what weights) — NOT subscription state. Subscription declarations propagate separately via the spanning tree.

## Pub/Sub

- Key expressions are hierarchical paths (like MQTT topics): `/robot1/sensors/lidar`
- Wildcard matching: `*` (single level), `**` (multi-level)
- Discovery: multicast scouting locally, gossip protocol when multicast unavailable
- Late joiners can receive latest cached values via publication caches (zenoh-ext `AdvancedPublisher`) or storage nodes

### How subscription interest propagates through routers

The network has two subsystems with different behavior:

**Router backbone (fully informed):** Routers propagate subscription declarations along the spanning tree. When a client behind Router B subscribes to `robot1/pose`, Router B sends a `DeclareSubscriber` to its peer routers. Router A receives it and starts forwarding matching publications toward Router B. If nobody behind Router B subscribes, **no data flows** — this is interest-based routing.

**Client/peer edge (simple):** Since Zenoh 1.0.0, subscription declarations are **not** propagated to clients/peers. All publications from a client/peer go to their nearest router unconditionally — the router decides what to forward. This is a scalability optimization: edge devices don't get flooded with subscription state from the entire network.

**Consequence:** Writer-side filtering (don't publish if nobody's listening) only works at the edge if the publisher explicitly declares a `Publisher` object, which triggers an Interest exchange with the router. A bare `session.put()` always goes to the router.

**Wildcard gotcha:** A wildcard storage (`key_expr: "**"`) declares itself as a subscriber for everything. This causes the hosting router to pull ALL publications from every connected router. Be careful with broad storage key expressions on routers that bridge constrained links.

**Interest timeout:** `routing.interests.timeout` (default 10s) — when a node connects and sends Interests to discover existing subscribers, it waits this long for the router to reply. If it times out, discovery may be incomplete (potential message loss at startup).

## Request/Response (Queryables)

- `Session.get(key_expr)` sends a query, returns zero or more replies
- Any node can declare a `queryable` that handles queries for a key expression
- A queryable can send multiple replies (streaming) — maps well to LLM token streaming, chunked image transfer
- Query payload is arbitrary — you can encode whatever request format you want
- Queries can be cancelled (since 1.7.x) — relevant for interactive/long-running queries

## Shared Memory (Zero-Copy Local IPC)

Zenoh has built-in SHM transport for zero-copy communication between processes on the same machine:

- Automatic: messages above a threshold (default 3 KB) are placed in SHM instead of copied
- Default pool size: 16 MB, configurable
- POSIX shared memory protocol (`PosixShmProvider`)
- Available in Rust, C, C++, and Python (since 1.6.x)
- Automatic garbage collection and defragmentation
- Typed SHM buffers with dynamic resize

Relevant for us: local sensor pipelines between co-located processes can bypass network serialization entirely, similar to our existing `SHMTransport` but with Zenoh's routing/discovery on top.

## Liveliness (Presence Detection)

Built-in presence/health system tied to session lifetime:

- `session.liveliness().declare_token("robot/robot1")` — announce presence
- `session.liveliness().declare_subscriber("robot/**")` — get `Put` on appearance, `Delete` on disappearance
- `session.liveliness().get("robot/**")` — query currently live tokens
- Crashes/disconnects automatically trigger `Delete` — no heartbeat polling needed

Directly useful for robot fleet management, module health monitoring.

## Timestamps (Hybrid Logical Clocks)

Every value in Zenoh gets a HLC timestamp automatically (when passing through a router):

- Timestamp = NTP64 time + HLC UUID
- Theoretical resolution: ~3.5 nanoseconds
- Guarantees: unique timestamps, happened-before ordering preserved, no consensus required
- Used by the storage alignment protocol for eventual consistency

## Reliability on Unreliable Networks

- Minimum 5-byte wire overhead (3-byte data message + 2-byte frame header; multiple messages share the frame overhead)
- Works at OSI Layer 2 — supports non-IP networks: BLE, serial, LoRa, CANbus
- Two channel types: best-effort and reliable (ordered delivery)
- Three reliability strategies:
  1. **Hop-by-hop** (default) — single reliability state per app, reliable under stable topology
  2. **End-to-end** — dedicated reliability channel per producer-consumer pair, no loss even during topology changes, higher resource cost
  3. **First-router-to-last-router** — reliability between edge routers, offloading pressure from endpoints to infrastructure
- Designed for WiFi/GSM from the start, unlike DDS which was designed for reliable LAN

## Backpressure & Congestion Control

Three independently configurable concerns:

1. **Resending (receiver controls):** Declares whether missing messages should be resent. Hop-by-hop or end-to-end.
2. **Buffering (each node controls):** Each node decides how much memory to dedicate. Constrained devices use minimal, routers buffer more.
3. **Dropping (sender controls):** Congestion control policy — drop sample or block publication. Propagates through routing path.

## QoS

- **Priority levels:** `RealTime`, `InteractiveHigh`, `InteractiveLow`, `DataHigh`, `Data` (default), `DataLow`, `Background`
- **Reliability:** best-effort vs reliable per-channel
- **Congestion control:** drop vs block
- **Express mode:** Bypass batching for immediate transmission (latency vs throughput tradeoff)
- **Locality control:** `SessionLocal` / `Remote` — control whether messages stay local-only, go remote-only, or both
- **Downsampling:** Configurable per-key-expression rate limiting — useful for high-frequency sensor data on slow links
- All configurable per key-expression via config file

## Performance

Official benchmarks (2023, Zenoh team + NTU, peer mode, single machine):

| Metric          | 8-byte payload    | 8 KB payload       |
|-----------------|-------------------|---------------------|
| Messages/sec    | ~4M msg/s         | —                   |
| Throughput      | —                 | ~67 Gbps            |
| Latency         | ~10 us (single machine), ~16 us (multi-machine, 64B) | |

Comparisons (vary by payload size and conditions):

| vs       | Messages/sec (8B) | Throughput (8KB) | Latency        |
|----------|-------------------|------------------|----------------|
| MQTT     | ~130x faster      | ~27x higher      | significantly lower |
| DDS      | ~2x faster        | ~4x higher       | comparable (DDS can be lower single-machine) |

Source: [Zenoh benchmarks blog](https://zenoh.io/blog/2023-03-21-zenoh-vs-mqtt-kafka-dds/), [arXiv paper](https://arxiv.org/abs/2303.09419)

## Scope Control (Local vs Internet)

Controlled via deployment topology and access control:

- **Local:** Peer mode, multicast scouting, stays on LAN
- **Site-wide:** Routers connect subnets
- **Internet:** Routers with TCP/TLS/QUIC endpoints
- **Access control:** ACLs per key-expression restrict what data flows where and to whom

No magic — you design the router topology to match your network architecture.

## Traffic Control at Routers

There is no single "routing filter" config. Instead, five layered mechanisms combine to control what crosses which link. All configured per-router in JSON5.

### 1. Interest-Based Routing (automatic)

Routers only forward data if a downstream subscriber exists for that key expression. No subscriber on the far side of the LoRa link = no data flows. This is automatic but **not sufficient alone** — a single wildcard subscriber (`**`) on the far side would pull everything.

### 2. ACL (deny key expressions per interface/link)

The primary hard filter. Block specific key expressions from egressing on specific interfaces or link protocols:

```json5
{
  access_control: {
    enabled: true,
    default_permission: "allow",
    rules: [
      {
        id: "block_highbw_on_lora",
        messages: ["put", "delete"],
        flows: ["egress"],
        permission: "deny",
        key_exprs: ["robot1/sensors/lidar/**", "robot1/sensors/camera/**"]
      }
    ],
    subjects: [
      // match by interface name, link protocol, TLS cert CN, username, or Zenoh ID
      { id: "lora_link", interfaces: ["ttyLoRa0"], link_protocols: ["serial"] }
    ],
    policies: [
      { id: "lora_restrict", rules: ["block_highbw_on_lora"], subjects: ["lora_link"] }
    ]
  }
}
```

Limitation: not runtime-updatable, requires restart.

### 3. Multilink (route by priority to different physical links)

A single router can listen on multiple transports simultaneously. Each endpoint declares which priority range and reliability mode it carries. Zenoh automatically routes messages to the matching link:

```json5
{
  listen: {
    endpoints: [
      "tcp/0.0.0.0:7447?prio=1-5;rel=1",                     // priorities 1-5 over WiFi (reliable)
      "serial//dev/ttyLoRa0#baudrate=9600?prio=6-7;rel=0"     // priorities 6-7 over LoRa (best-effort)
    ]
  }
}
```

Publishers choose their priority:
```python
session.declare_publisher("robot1/pose", priority=Priority.INTERACTIVE_HIGH)    # → WiFi
session.declare_publisher("robot1/battery", priority=Priority.BACKGROUND)       # → LoRa
```

Priority values: 1=RealTime through 7=Background. Also supports DSCP marking for IP-level QoS (`tcp/...#dscp=0x08`).

### 4. Downsampling (rate-limit per key expression per interface)

Rate-limit what crosses a link, even if ACL allows it:

```json5
{
  downsampling: [
    {
      interfaces: ["ttyLoRa0"],
      link_protocols: ["serial"],
      flows: ["egress"],
      messages: ["put", "delete"],
      rules: [
        { key_expr: "robot1/pose", freq: 1.0 },        // 100 Hz → 1 Hz on LoRa
        { key_expr: "robot1/battery", freq: 0.1 },      // once per 10s
        { key_expr: "fleet/status/**", freq: 0.5 }
      ]
    }
  ]
}
```

### 5. Low-Pass Filter (message size limit per interface)

Drop messages exceeding a size threshold — prevents accidentally large payloads from saturating a constrained link:

```json5
{
  low_pass_filter: [
    {
      interfaces: ["ttyLoRa0"],
      link_protocols: ["serial"],
      flows: ["egress"],
      messages: ["put", "delete"],
      key_exprs: ["**"],
      size_limit: 256           // bytes
    }
  ]
}
```

### 6. Locality API (application-level)

Publishers/subscribers can declare whether they're local-only or remote-only:

```python
# Never leaves this process — internal debug data
session.declare_publisher("robot1/internal/debug", locality=Locality.SESSION_LOCAL)
```

### Combining them (example: robot over LoRa)

For a robot with lidar/video locally and poses/commands over LoRa, layer:
1. **ACL** blocks lidar/camera/pointcloud from egressing on the serial interface
2. **Multilink** routes high-priority control on WiFi, low-priority telemetry on LoRa
3. **Downsampling** reduces pose from 100 Hz to 1 Hz on the LoRa egress
4. **Low-pass filter** caps message size at 256 bytes on LoRa as a safety net
5. **Locality** keeps internal debug topics process-local
6. **Interest-based routing** automatically avoids forwarding anything nobody subscribed to

## Video / Teleop

- **gst-plugin-zenoh:** Third-party GStreamer plugin for video over Zenoh (not official eclipse-zenoh)
- Express mode for low-latency frames
- Plugin supports optional compression (zstd, lz4, gzip) as compile-time features
- NAT traversal via router relay (router with public IP mediates), not ICE/STUN hole-punching
- For production teleop: deploy a cloud-side Zenoh router as relay endpoint

Note: Zenoh itself has transport-level compression (enabled via `transport.unicast.compression.enabled` in config), but it's all-or-nothing for a session — you don't pick the algorithm. The GStreamer plugin's compression is application-level and per-stream.

## Router (`zenohd`)

The Zenoh router is a standalone daemon:

- Installed as binary (Debian packages, Docker images, `cargo install zenohd`)
- Default REST plugin on **port 8000** — maps HTTP verbs to Zenoh operations (GET=get, PUT=put, DELETE=remove, GET+SSE=subscribe)
- Plugin system: loads dynamic libraries (`libzenoh_plugin_<name>.so`)
- Configured via JSON5 config file or CLI arguments
- **Admin space:** Every node exposes internal state under the `@` key prefix, queryable via standard Zenoh or REST API

## Built-in Storage (Distributed KV)

Storage manager plugin watches key expressions and persists values. Five backends:

- **Memory:** In-process RAM, lost on restart. Caching.
- **Filesystem:** Each key maps to a file in a configured directory.
- **RocksDB:** Embedded LSM-tree database. Persistent, fast, handles large volumes.
- **InfluxDB:** Time-series database backend. Good for sensor telemetry history.
- **S3:** Amazon S3 / MinIO compatible. Blob storage for large artifacts (maps, point clouds).

Any node can run the storage plugin. Other nodes query transparently — `session.get("/map/global")` routes to whichever node stores it. Multiple nodes can store the same keys — alignment protocol (using HLC timestamps) syncs them (eventual consistency, not linearizable).

Example config:
```json5
{
  "plugins": {
    "storage_manager": {
      "storages": {
        "robot_map": {
          "key_expr": "/map/**",
          "volume": "rocksdb",
          "db": "/var/zenoh/map_db"
        },
        "sensor_cache": {
          "key_expr": "/sensors/**",
          "volume": "memory"
        }
      }
    }
  }
}
```

### Storage limitations

This is a **flat key-value store**, not a database. You can:
- Look up by exact key
- Scan by key prefix/range (lexicographic)
- Get latest value for late joiners

You cannot:
- Query by value content or field values
- Do spatial queries ("find within 5m of X")
- Do temporal range queries on arbitrary fields
- Secondary indexes, joins, aggregation

### RocksDB specifics

Embedded library (no server process). LSM-tree — optimized for fast writes. Keys sorted lexicographically, only primary key index. Bloom filters for fast point lookups. Column families for namespaces. TTL for auto-expiry. Compression per level (snappy, zstd, lz4).

For real spatial/temporal queries, you'd implement a queryable node that runs actual DB logic (PostGIS, SQLite R-tree, etc.) and returns results via Zenoh's reply mechanism. Zenoh transports the query/response, your code does the indexing.

## Access Control

ACL system with three components:

- **Rules:** Define permission (allow/deny), message types (put, delete, declare_subscriber, query, reply, liveliness_token, etc.), flows (ingress/egress), and key expressions
- **Subjects:** Match remote nodes by network interface, TLS cert common name, or username
- **Policies:** Link rules to subjects
- `default_permission`: global allow/deny fallback
- **Limitation:** Cannot be updated at runtime — requires restart

## What Zenoh doesn't have

- **No consensus/leader election** — no Raft, no distributed locking. Build on top or use a separate tool.
- **No rich queries** — KV store only, not a database
- **No ICE/STUN NAT traversal** — relay through routers only
- **No runtime ACL updates** — access control requires restart

## Routing in Router Meshes

When routers form a mesh (multiple paths exist between source and destination), routing works as follows:

### Spanning tree computation

Every router computes **N trees, one rooted at each router** in the network, using Bellman-Ford shortest paths on the weighted topology graph. Each tree stores, for every destination, which direct neighbor is the next hop.

- **Default link weight:** 100 (configurable via `routing.router.linkstate.transport_weights`)
- **Tiebreaking:** Deterministic hash jitter (up to 1%) added to weights so all routers agree on the same tree when weights are equal. This prevents loops.
- **If both sides of a link set weights, the greater value is used.** If only one side sets it, that value is used.

### Data forwarding

Messages carry the source router's NodeId. Each forwarding router looks up `trees[source].directions[subscriber]` — "in the tree rooted at the original publisher's router, what's my next hop toward this subscriber?"

This is **multicast along the source-rooted tree**: one copy per distinct outgoing face that has subscribers behind it. Intermediate routers branch as needed. Multiple subscribers behind the same next-hop collapse into one forwarded copy.

**No load balancing.** Strictly single-path per (source, destination) pair. No ECMP.

### Router election (deduplication)

When peers operate in linkstate mode and connect to multiple routers, they'd receive duplicate messages. `elect_router()` deterministically picks one router per key expression (via hash of key_expr + router ZID) to be responsible for forwarding to/from that peer. Different key expressions can elect different routers, spreading load.

### Failover

When a router-to-router link fails:
1. Transport layer detects disconnect
2. Router removes the edge from its graph, broadcasts updated link-state to remaining neighbors
3. **100ms debounce delay** (`TREES_COMPUTATION_DELAY_MS`), then trees are recomputed
4. All cached routes are invalidated; next message triggers fresh route computation

**During the ~100ms recomputation window, messages routed through the dead link are silently lost.** There is no buffering or automatic reroute during recomputation. The hop-by-hop reliability layer ensures delivery over working links but cannot reroute around a failed one. End-to-end reliability (dedicated per-producer-consumer channels) can recover from this via retransmission after the new route is established.

### Loop prevention

Guaranteed loop-free by three mechanisms:
1. Source-rooted trees are DAGs by construction (shortest-path tree from Bellman-Ford)
2. Deterministic edge weight jitter ensures all routers compute identical trees
3. Ingress/egress filters prevent forwarding back to the arrival face

## Protocol Bridges (Plugins)

- **zenoh-plugin-mqtt:** MQTT broker that routes to/from Zenoh — existing MQTT devices participate transparently
- **zenoh-plugin-ros2dds:** Bridges DDS-based ROS 2 topics into Zenoh without changing RMW — useful for mixed fleets
- **zenoh-plugin-webserver:** HTTP/WebSocket access to Zenoh key space

## Advanced Pub/Sub (zenoh-ext)

Extensions beyond basic pub/sub:

- **Publication cache:** Publisher retains N recent samples, replies to late-joiner queries automatically
- **Advanced subscriber:** Detects sample miss (gaps in sequence numbers), can recover from publication caches
- **Matching listeners:** Get notified when a subscriber/publisher for your key expression appears/disappears

## Python Support

- PyPI: `eclipse-zenoh`
- Latest version: 1.8.0 (March 2026)
- **Development status: Beta** — not marked as production-ready on PyPI
- Rust-backed bindings (not pure Python) — good performance
- Minimum CPython 3.9 (binary wheels use `cp39-abi3` tag)
- Binary wheels for Linux (x86_64, ARM, ARMv6/v7, i686), macOS, Windows
- SHM API available since 1.6.x

## vs LCM (our current default)

|                  | LCM                | Zenoh                                               |
|------------------|--------------------|-----------------------------------------------------|
| Transport        | UDP multicast only | UDP, TCP, TLS, QUIC, BLE, serial, WebSocket         |
| Scope            | Local LAN only     | Local to internet-scale                             |
| Reliability      | Best-effort only   | Configurable (best-effort, reliable, 3 strategies)  |
| Discovery        | Multicast          | Multicast + gossip fallback                         |
| Request/response | No (pub/sub only)  | Built-in queryables                                 |
| Storage          | No                 | Built-in KV with 5 backends                        |
| QoS              | None               | 7 priority levels, congestion control, express mode |
| Backpressure     | None               | Per-concern (resend/buffer/drop)                    |
| Local IPC        | UDP (no zero-copy) | Shared memory (zero-copy above 3 KB threshold)     |
| Presence         | None               | Liveliness tokens (automatic crash detection)       |
| Timestamps       | None               | HLC (hybrid logical clocks)                         |
| Cross-language   | Via LCM codegen    | Native bindings (Rust, C, C++, Python)              |

## vs DDS

|               | DDS                         | Zenoh                      |
|---------------|-----------------------------|----------------------------|
| Wireless      | Poor (multicast flooding)   | Designed for it            |
| Network scale | LAN (multicast constraints) | Internet-scale via routers |
| Discovery     | Complex, unreliable on WiFi | Robust scouting + gossip   |
| Modes         | Peer-to-peer only           | Peer, routed, client       |
| Throughput    | Good                        | ~2-4x DDS depending on payload |
| Latency       | Good (can beat Zenoh locally) | Comparable                |
| Complexity    | Heavy config                | Simpler API                |

## Integration path for dimos

Stub exists at `dimos/core/transport.py:323` — `class ZenohTransport(PubSubTransport[T]): ...`

The pubsub abstraction layer lives in `dimos/protocol/pubsub/`:
- `spec.py` — `PubSub[TopicT, MsgT]` base, `AllPubSub`, `DiscoveryPubSub`
- `encoders.py` — `PickleEncoderMixin`, `LCMEncoderMixin`, `JpegEncoderMixin`
- `impl/` — existing implementations: `lcmpubsub.py`, `shmpubsub.py`, `ddspubsub.py`, `rospubsub.py`, `redispubsub.py`, `memory.py`

Implementation steps:

1. Create `dimos/protocol/pubsub/impl/zenohpubsub.py` — implement `PubSub[TopicT, MsgT]` backed by Zenoh sessions
2. Reuse existing encoder mixins (e.g. `PickleEncoderMixin`) for payload serialization
3. Complete `ZenohTransport(PubSubTransport[T])` in `transport.py`
4. Implement RPC via `PubSubRPCMixin` + Zenoh queryables (fits naturally — queryables already support streaming replies)
5. Test: split system — local control loop on robot (peer mode), GPU module offloaded to server (routed via Zenoh router)

## Robotics adoption

- **ROS 2:** Official `rmw_zenoh` middleware — supported on Rolling, Kilted, Jazzy, Humble. Zenoh is **Tier 1 middleware** in Kilted Kaiju (May 2025) but **Fast DDS remains the default**
- **PX4:** Zenoh-Pico client runs on the flight controller, communicates via UART/TCP/UDP to a `zenohd` router on companion computer, bridged to ROS 2 topics via `rmw_zenoh`. Alternative to the uXRCE-DDS bridge, not a replacement. Note: ROS 2 Jazzy added message type hashes that break PX4 compat unless `CONFIG_ZENOH_KEY_TYPE_HASH=n`
- **ZettaScale** (creators) are the CycloneDDS team — same people, evolved design
