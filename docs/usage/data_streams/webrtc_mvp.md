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
- `media_to_first_frame_ms`: time from accepting the media offer to the first frame
- `latency_ms_p50` and `latency_ms_p95`: one-way frame latency
- `sequence_gaps`: frames skipped between decoded timestamp headers
- RTP packets received and lost

The sender embeds a timestamp, sequence number, and CRC in each synthetic
frame. This measures the encoded H.264 path instead of only measuring a
DataChannel ping. The publisher offers H.264 only; startup fails instead of
silently falling back when the local H.264 encoder is unavailable. The
subscriber also fails if no real video frame arrives within 20 seconds or if
none of the received frames contain valid timestamp metadata.

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

## Raw video upload/download test

The repository MVP stores the original video object without ZIP packaging. An
object is addressed by developer/organization owner, repository name, and its
SHA-256 object id.

Start a local server:

```bash
export DIMOS_REPLAY_REPOSITORY_TOKEN=local-test-token
dimos replay-repo serve \
  --root /tmp/dimos-replay-repository \
  --public-read
```

Upload a video or elementary H.264/H.265 stream:

```bash
dimos replay-repo upload go2-office.mp4 \
  --owner alice \
  --repo go2-debug
```

The JSON result contains the immutable `object_id`. Another machine can list
Alice's repository and download that exact object:

```bash
dimos replay-repo list --owner alice --repo go2-debug
dimos replay-repo download OBJECT_ID \
  --owner alice \
  --repo go2-debug \
  --output go2-office-copy.mp4
```

With `--public-read`, a browser can play or download videos at:

```text
http://SERVER:8765/r/alice/go2-debug
```

Set `--server-url` on all three client commands when the repository server is
remote. The client streams the raw object to disk and verifies its SHA-256
before making the destination visible. A non-loopback server refuses to start
without `DIMOS_REPLAY_REPOSITORY_TOKEN`. Public read is an explicit MVP option;
do not enable it for private recordings.

## Batch transfer

For a capture directory, use the batch commands. Uploads run concurrently, each
file is retried independently, and the manifest is written atomically only after
all objects have completed:

```bash
dimos replay-repo batch-upload ./capture \
  --owner alice \
  --repo go2-debug \
  --pattern "*.mp4" \
  --workers 4 \
  --manifest ./capture-manifest.json

dimos replay-repo batch-download ./capture-manifest.json \
  --output-dir ./restored \
  --workers 4
```

The manifest records the owner, repository, filename, object id, byte count,
content type, and SHA-256. A failed item does not get advertised as completed;
rerunning the same batch is safe because object ids are content-addressed.

The current server is a local filesystem reference backend. The stable boundary
for a production migration is the HTTP object contract plus the manifest. A
production backend should replace the blob files with S3/R2/MinIO, metadata with
a durable database, and direct downloads with signed CDN URLs. Multipart upload,
range reads, TLS, quotas, and audit logs are required before exposing it to the
public internet.
