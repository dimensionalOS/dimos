---
title: "Hosted Data and Cross-Region Replay MVP"
---

The hosted-data MVP groups raw-object transfer, memory2 snapshot publishing,
remote Go2 replay, China/US node recommendation, and WebRTC diagnostics behind
one `dimos data` command. The storage path keeps original files rather than ZIP
archives. The live WebRTC path uses the existing DimOS broker protocol and
Cloudflare Realtime SFU integration.

Install the optional WebRTC dependencies from the source checkout:

```bash
python -m pip install -e ".[webrtc]"
```

Start with the credential-free local H.264 baseline:

```bash
dimos data live local --duration 5
```

From a source checkout that has not installed the `dimos` console script yet:

```bash
python -m dimos.hosted_data.cli live local --duration 5
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
dimos data live publish \
  --broker-url https://cn-teleop.example.com \
  --fps 15
```

The first JSON line contains `session_id`. Keep the publisher running.

Run the receiver at the other site, for example the US:

```bash
export TELEOP_OPERATOR_TOKEN=...
dimos data live subscribe SESSION_ID \
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
dimos data serve \
  --root /tmp/dimos-replay-repository \
  --public-read
```

Upload a video or elementary H.264/H.265 stream:

```bash
dimos data upload go2-office.mp4 \
  --owner alice \
  --repo go2-debug
```

The JSON result contains the immutable `object_id`. Another machine can list
Alice's repository and download that exact object:

```bash
dimos data list --owner alice --repo go2-debug
dimos data download OBJECT_ID \
  --owner alice \
  --repo go2-debug \
  --output go2-office-copy.mp4
```

With `--public-read`, a browser can download videos and play files that support
progressive HTTP playback at:

```text
http://SERVER:8765/r/alice/go2-debug
```

Set `--server-url` on all three client commands when the repository server is
remote. The client streams the raw object to disk and verifies its SHA-256
before making the destination visible. A non-loopback server refuses to start
without `DIMOS_REPLAY_REPOSITORY_TOKEN`. Public read is an explicit MVP option;
do not enable it for private recordings.

The MVP does not yet implement HTTP `HEAD` or byte-range requests. Seeking and
first-frame performance therefore depend on the video's container layout;
robust browser streaming remains follow-up work.

## Batch transfer

For a capture directory, use the batch commands. Uploads run concurrently, each
file is retried independently, and the manifest is written atomically only after
all objects have completed:

```bash
dimos data batch-upload ./capture \
  --owner alice \
  --repo go2-debug \
  --pattern "*.mp4" \
  --workers 4 \
  --manifest ./capture-manifest.json

dimos data batch-download ./capture-manifest.json \
  --output-dir ./restored \
  --workers 4
```

The manifest records the owner, repository, filename, object id, byte count,
content type, and SHA-256. A failed item does not get advertised as completed;
rerunning the same batch is safe because object ids are content-addressed.

## S3, R2, and MinIO storage

Install the optional cloud-storage dependencies and select the S3-compatible
backend:

```bash
pip install "dimos[cloud-storage]"

export DIMOS_REPLAY_REPOSITORY_TOKEN=replace-me
export DIMOS_REPLAY_S3_BUCKET=dimos-replays
export AWS_ACCESS_KEY_ID=...
export AWS_SECRET_ACCESS_KEY=...

dimos data serve \
  --host 0.0.0.0 \
  --backend s3 \
  --s3-region us-east-1
```

Cloudflare R2 and MinIO use the same backend. Set
`DIMOS_REPLAY_S3_ENDPOINT_URL` to the provider endpoint. MinIO commonly also
needs `DIMOS_REPLAY_S3_ADDRESSING_STYLE=path`; R2 commonly uses region `auto`.
The normal AWS credential chain remains available when explicit credentials
are omitted.

Clients continue to use the same HTTP and manifest contract regardless of the
selected backend. Completed metadata is written after the immutable blob, so
an interrupted upload is never listed as complete. Downloads stream from
object storage without first materializing the entire object on server disk.

## Upload a memory2 dataset

Upload a live or closed memory2 SQLite recording through the same repository:

```bash
dimos data upload go2-office.db \
  --owner alice \
  --repo go2-debug \
  --server-url https://replays.example.com
```

The JSON response includes `replay_uri`, which is the value to pass into the
existing replay CLI.

The command does not copy the database file directly. It uses SQLite's backup
API to create a consistent standalone snapshot, including committed data still
in the WAL, and runs `PRAGMA integrity_check` before upload. The original `.db`
is sent without ZIP packaging.

Each upload publishes two immutable objects:

1. The replayable memory2 `.db` snapshot.
2. A `<dataset>.memory2.json` index containing the snapshot object id,
   SHA-256, size, SQLite user version, and each stream's item count and time
   range.

The index is uploaded only after the database object completes, so consumers
never discover an index that points at an unfinished upload. Use `--name` to
give the portable dataset a name different from its local filename.

## Replay a remote memory2 dataset on Go2

Point `DIMOS_REPLAY_DB` at the uploaded database object's content-addressed
reference, then use the existing replay command:

```bash
export DIMOS_REPLAY_DB='PASTE_REPLAY_URI_FROM_UPLOAD'
export DIMOS_REPLAY_SERVER_URL='https://replays.example.com'
export DIMOS_REPLAY_REPOSITORY_TOKEN=...
dimos --replay run unitree-go2
dimos --replay --replay-db "$DIMOS_REPLAY_DB" run unitree-go2-memory
```

The object is downloaded atomically, verified against the SHA-256 object id,
checked for the memory2 `_streams` registry, and cached below the DimOS cache
directory. Replaying the same object again uses the verified local cache. The
repository token is read from the environment and is not embedded in the URI,
so references can be shared without sharing credentials. The URI server must
match `DIMOS_REPLAY_SERVER_URL` or a URL in `DIMOS_REPLAY_NODES` before DimOS
will connect or forward the environment token. This prevents an untrusted
shared URI from redirecting credentials to another host.

## China/US node selection

Each server exposes an unauthenticated, metadata-only `/healthz` endpoint.
Configure its identity when starting the service:

```bash
dimos data serve \
  --host 0.0.0.0 \
  --node-name cn-beijing \
  --region china
```

Clients can configure both China and US candidates as JSON:

```bash
export DIMOS_REPLAY_NODES='[
  {"name":"cn-beijing","url":"https://cn-replays.example.com","region":"china"},
  {"name":"us-west","url":"https://us-replays.example.com","region":"us"}
]'

dimos data recommend
```

The probes run concurrently. Unhealthy nodes are excluded and the lowest
measured health-check latency is recommended. The result is keyed by the node
configuration and cached for one hour; `dimos data recommend --force` refreshes
it immediately. Upload, list, download, and batch transfer
automatically use this recommendation when neither `--server-url` nor
`DIMOS_REPLAY_SERVER_URL` is set.

## Production hardening

The server can terminate TLS directly, enforce byte limits, and generate links
through a CDN origin:

```bash
dimos data serve \
  --host 0.0.0.0 \
  --port 443 \
  --tls-certfile /etc/letsencrypt/live/replays.example.com/fullchain.pem \
  --tls-keyfile /etc/letsencrypt/live/replays.example.com/privkey.pem \
  --max-object-bytes 10737418240 \
  --max-repository-bytes 107374182400 \
  --cdn-base-url https://cdn.example.com
```

TLS requires both files and enforces TLS 1.2 or newer. Per-object limits reject
the request before reading its body; repository reservations include concurrent
uploads so parallel clients cannot overrun the quota. Content-addressed
reuploads do not consume quota twice. These reservations are process-local:
when a byte quota is enabled, use one writer node until reservations are backed
by a shared transactional store.

`/metrics` exposes Prometheus counters for requests, failures, transfers,
bytes, and recovered partial uploads. It follows normal read authorization;
send the bearer token unless `--public-read` is intentionally enabled.
`/healthz` remains public and contains no repository data so load balancers can
probe it. Successful uploads are written to the Python audit log with owner,
repository, object id, and byte count. Filesystem servers remove interrupted
`.part` files at startup, while S3-compatible servers continue to advertise
metadata only after the immutable object finishes.

Without process-local quotas, API nodes can be replicated behind the China/US
load balancers with the shared S3/R2/MinIO backend. The CDN should use the API
nodes as its origin and cache immutable object-id URLs. Shared quota
reservations, multipart/resumable upload, and provider-native signed CDN URLs
remain follow-up work for very large datasets.

## Current limitations

This PR is an MVP and progresses the hosted-data design rather than closing the
entire roadmap. The WebRTC commands currently measure a synthetic H.264 path;
they do not continuously ingest Go2 sensor streams into memory2. A memory2
upload is a blocking point-in-time SQLite snapshot, not a background or
incremental upload. China/US candidates must be supplied through
`DIMOS_REPLAY_NODES`; the hourly recommendation measures health and round-trip
latency but does not yet discover nodes or measure bandwidth and packet loss.
The remote replay resolver is connected to `dimos --replay run unitree-go2`,
not every replay consumer. Resumable multipart upload, HTTP Range/HEAD, signed
URLs, automatic CDN deployment, and multi-tenant ACLs remain follow-up work.
