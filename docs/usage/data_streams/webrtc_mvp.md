---
title: "WebRTC Cross-Region MVP"
---

The WebRTC MVP measures the live path before memory2 or asynchronous storage
is added. It uses the existing DimOS broker protocol and Cloudflare Realtime
SFU integration.

Install the optional WebRTC dependencies from the source checkout:

```bash
python -m pip install -e ".[webrtc]"
```

Start with the credential-free local H.264 baseline:

```bash
dimos webrtc-mvp local --duration 5
```

From a source checkout that has not installed the `dimos` console script yet:

```bash
python -m dimos.protocol.pubsub.impl.webrtc.mvp_cli local --duration 5
```

The result is one JSON object containing:

- `signaling_connected_ms`: cold offer/answer and ICE connection time
- `media_to_first_frame_ms`: time from an established media path to the first frame
- `latency_ms_p50` and `latency_ms_p95`: one-way frame latency
- `sequence_gaps`: frames skipped between decoded timestamp headers
- RTP packets received and lost

The sender embeds a timestamp, sequence number, and CRC in each synthetic
frame. This measures the encoded H.264 path instead of only measuring a
DataChannel ping.

Run the publisher at one site, for example China:

```bash
export TELEOP_API_KEY=dtk_live_...
dimos webrtc-mvp publish \
  --broker-url https://cn-teleop.example.com \
  --fps 15
```

The first JSON line contains `session_id`. Keep the publisher running.

Run the receiver at the other site, for example the US:

```bash
export TELEOP_OPERATOR_TOKEN=...
dimos webrtc-mvp subscribe SESSION_ID \
  --broker-url https://us-teleop.example.com \
  --duration 30
```

The two broker hostnames must share session state and route to the same SFU
application. With one anycast broker hostname, pass the same URL at both sites;
the CDN routes each client to its closest edge.

Use environment variables for secrets so they do not appear in shell history
or the process command line. Synchronize both hosts with NTP before interpreting
one-way latency. Clock error is included directly in that measurement.

Suggested MVP gates:

- media-ready to first frame below 500 ms
- cross-region one-way P95 below 300 ms
- cold signaling below 3 seconds
- no sustained sequence gaps during a 30-second run

H.264 is deliberate for the first interoperability test. H.265/HEVC support
varies across WebRTC clients and should be enabled only after both endpoints
advertise compatible codec capabilities.

After the live path meets these gates, connect the publisher input to the Go2
camera stream used by `dimos --replay run unitree-go2`. Add memory2 recording
as a side subscriber so storage cannot delay the live media path.
