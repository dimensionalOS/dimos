# Relay

The relay is the server both sides connect to. Robots connect as robot sessions and browsers as viewer sessions, both over WebTransport. The relay routes frames by channel id without reading their payloads, sends each channel to the viewers that asked for it, and tells the bridge which channels anybody wants. It also serves the cockpit and the SDK over plain HTTP. It stores nothing: everything is in memory, and the clients re-establish it after a restart.

It runs in two modes. Local: `dimos run --local-relay` spawns one on your machine, bound to loopback, with no authentication and an ephemeral self-signed certificate. Hosted: the same program with a real certificate and an auth file, reachable from anywhere. Robots dial out to it, so they need no open ports.

<details>
<summary>diagram source</summary>

```pikchr fold output=assets/relay_modes.svg
color = white
fill = none
boxrad = 5px
margin = 0.06in

text "local" bold
LB: box "browser" "same machine" wid 1.2in ht 0.65in with .nw at last text.sw + (0, -0.1in)
LR: box "Relay" "127.0.0.1 only" wid 1.2in ht 0.65in with .w at LB.e + (1.9in, 0)
LD: box "Bridge" "same machine" wid 1.2in ht 0.65in with .w at LR.e + (1.3in, 0)
arrow <-> from LB.e + (0, 0.14in) to LR.w + (0, 0.14in) "HTTP: page, /api/info" above
arrow <-> from LB.e - (0, 0.14in) to LR.w - (0, 0.14in) "QUIC: discovered port" below
arrow <-> from LD.w to LR.e "QUIC, loopback" above

text "hosted" bold with .nw at LB.sw + (0, -0.55in)
HB: box "browser" "anywhere" wid 1.2in ht 0.65in with .nw at last text.sw + (0, -0.1in)
HR: box "Relay" "real certificate," "auth file" wid 1.2in ht 0.65in with .w at HB.e + (1.9in, 0)
HD: box "Bridge" "behind NAT" wid 1.2in ht 0.65in with .w at HR.e + (1.3in, 0)
arrow <-> from HB.e + (0, 0.14in) to HR.w + (0, 0.14in) "HTTPS: page, /api/info" above
arrow <-> from HB.e - (0, 0.14in) to HR.w - (0, 0.14in) "QUIC: same port, viewer token" below
arrow <-> from HD.w to HR.e "QUIC, robot key" above
```

</details>

![Local and hosted relay side by side. Local: browser, relay and bridge on one machine, the page over HTTP on port 7780 and live traffic over QUIC on a discovered port. Hosted: the browser reaches the relay over HTTPS and QUIC on one port with a viewer token, and the bridge connects from behind NAT with its robot key.](assets/relay_modes.svg)

The arrows show traffic. The robot initiates its connection in both modes, so it needs no open ports.

## Local relay

Add `--local-relay` to a blueprint command (a blueprint with `cockpit(...)` and no `--relay-url` does the same by itself). dimOS starts the relay and opens the cockpit. In a checkout, it builds missing or outdated web bundles first. Installed wheels include the bundles. If Deno is missing from `PATH`, dimOS downloads the pinned version into the dimos cache. The relay listens on port 7780 (`--local-port`), and the page opens at `http://127.0.0.1:7780/` once the relay's ready line appears (`--no-open-browser` to skip that).

The local relay listens on 127.0.0.1 only and deliberately trusts local pages: `/api/info`, `/sdk.js` and served JavaScript modules answer with wildcard CORS, so a Vite dev server, a page on another local port, or a `file:` page can connect without configuration and without a token. Only pages on the same machine can use it, because browsers allow WebTransport only on secure pages and `http://<lan-ip>` is not one. Another machine needs the TLS and auth flags below.

Its certificate is self-signed and regenerated on every start. The browser pins the certificate hash from `/api/info`, so no trust store changes are needed. The Python bridge skips certificate verification for the local relay, and only for a loopback address.

`--serve-dir DIR` after the blueprint name makes the relay serve your own directory at `/` instead of the cockpit. `/api/*` and `/sdk.js` keep precedence, and path traversal and symlink escapes are refused. The [web SDK](/docs/web/web_sdk.md) page uses it.

## Running a relay by hand

Start the relay yourself to work on it without restarting the robot, or to share one relay between robots. Build the bundles once (and after changes under `web/`), then run it from `web/`:

```bash
cd web
deno install --frozen
deno task -r build
deno task dev --cockpit-dir cockpit/dist --sdk-dir sdk/dist
```

| Flag | Default | Meaning |
|---|---|---|
| `--port` | 7780 | HTTP port. With `--cert` it is the QUIC port too. |
| `--host` | 127.0.0.1 | Bind address. Anything else needs the TLS and auth flags, or `--unsafe-non-loopback`. |
| `--cockpit-dir` | none | The built cockpit, served at `/`. Without it, the relay answers `/api/*` only. |
| `--sdk-dir` | none | The built SDK, served at `/sdk.js`. |
| `--serve-dir` | none | Your own directory at `/` instead of the cockpit. Loopback only. |
| `--cert`, `--key` | none | PEM certificate and key, given together. |
| `--auth-file` | none | Robot keys and viewer tokens (below). |
| `--unsafe-non-loopback` | off | Bind a non-loopback host without TLS and auth, behind your own TLS and access control. |

The first line on stdout is the ready line:

```json
{"event":"ready","httpPort":7780,"wtUrl":"https://127.0.0.1:49940","certHash":"...","v":6}
```

`wtUrl` is the WebTransport endpoint. Its QUIC port is ephemeral and changes on every start, so clients discover it through `/api/info` and you never type it. Attach a robot with the relay's HTTP address:

```bash
uv run dimos --replay run unitree-go2 --relay-url http://localhost:7780
```

Then open `http://localhost:7780/` for the cockpit. Things to know:

- Restart the relay whenever you like. The bridge and the page reconnect on their own.
- A bridge killed without a clean close keeps its robot id registered until the relay's 30 s idle timeout. Restarting it inside that window waits the conflict out.
- `--serve-dir` belongs to the relay here (`deno task dev --serve-dir DIR`). `dimos run --serve-dir` is rejected together with `--relay-url`.
- A second robot on the same relay needs its own `--robot-id`. A synthetic one for testing: `uv run python -m dimos.web.relay_bridge.demo_smoke --url http://localhost:7780`.
- With several robots on the relay the cockpit lists them. Pick one to watch it, and `switch robot` in the header reopens the list.
- Another machine needs TLS and the auth file (below), and `--relay-ca` on the robot for a private CA.

## HTTP routes

| Route | What it serves | Access |
|---|---|---|
| `/` | the cockpit build, or `--serve-dir` | same origin |
| `/sdk.js` | the SDK bundle, never cached | wildcard CORS |
| `/api/info` | `{"wtUrl": ..., "certHash": ..., "v": 6}`: the WebTransport address, the certificate hash (absent with a real certificate), the protocol version | wildcard CORS, never authenticated |
| `/api/stats` | live counters per robot, channel and viewer (see [Traffic](#traffic)) | wildcard CORS without an auth file. With one, `Authorization: Bearer <viewer token>` and no CORS header |

Served JavaScript modules (`.js`, `.mjs`) get wildcard CORS as well, so a page on another origin can import them. The viewer token, not CORS, is the access boundary.

## TLS

`--cert fullchain.pem --key privkey.pem` turns on real TLS. HTTPS (TCP) and QUIC (UDP) share `--port`, clients verify the certificate normally, `/api/info` carries no hash, and the WebTransport address is derived from the host the client dialed. There is no reverse proxy option: an HTTP proxy cannot carry QUIC. The files are read at startup, so a renewal means a restart.

A robot that must trust a private CA (mkcert on a LAN) passes it with `--relay-ca`. Browsers are pickier: Firefox needs a preference flipped to run QUIC through a private root, and Chromium refuses it by default (its `--webtransport-developer-mode` switch turns the check off). The [hosting guide](/docs/web/relay_hosting.md) has both.

## Auth file

`--auth-file auth.json` turns authentication on. The file holds robot keys, each bound to one robot id, and viewer tokens. Secrets are 16 to 256 characters (`openssl rand -hex 32`) and no secret appears twice:

```json
{
  "robots": { "go2-lab": "<key>" },
  "viewers": { "paul": "<token>" }
}
```

A robot sends its key when it connects (`RELAY_KEY` in its environment or `.env`, bound to its `--robot-id`). A viewer sends its token: the cockpit asks for it and keeps it in local storage until `log out`, and a page of your own passes `connect({ url, token })`. A viewer token grants everything (view, teleop, publish) on every robot behind the relay, so hand tokens out like passwords.

A wrong or missing secret fails with `auth_failed`, and that is terminal: neither client retries. The relay compares secrets in constant time, never logs one, and reports a malformed file by entry name. Edits to the file need a restart. With auth on, `/api/stats` wants the viewer token as a bearer and the log names viewers (`viewer 1 authenticated as paul`).

## Binding beyond loopback

A `--host` other than loopback is accepted only with `--cert`, `--key` and `--auth-file` together, or with `--unsafe-non-loopback` when your own TLS and access control sit in front. `--serve-dir` is refused on any non-loopback host: a public relay serves only the built cockpit. `--auth-file` alone on loopback is fine.

## Traffic

What the relay does between the two sides, for reading the stats and the logs:

- Sessions. A robot registers with its id and manifest, and a second robot with the same id is rejected (`robot_id_conflict`). Viewers get the robot list, pick one with `watch`, receive its manifest, and subscribe to channels. QUIC sessions idle out after 30 s, with keepalives every 4 s.
- Subscriptions. The relay keeps the union of the viewers' subscriptions per robot and sends the bridge a snapshot on every change. Encoding on the robot starts when the first viewer subscribes to a channel and stops when the last one leaves.
- Forwarding. A `latest` channel sends every frame on its own stream. A stream older than 500 ms is reset, so a slow viewer gets only the newest frames. A `reliable` channel uses one ordered stream per viewer and channel, with a queue of 64 frames or 16 MiB per viewer and channel (a lone frame over the byte cap still queues, up to the 64 MiB frame limit). A viewer that lets the queue overflow is disconnected with `reliable channel overflow`.
- Teleop. One viewer per robot holds the lease. Its commands are forwarded with a generation number, a second viewer's arm request gets `teleop_held`, and commands from anyone but the holder are dropped. When the holder disconnects or switches robots, the relay releases the lease and sends the bridge `teleop_stop`.
- Publishing. A `pub` is checked in order: a watched robot, no duplicate request id, size (32 KiB), a declared `publish="shared"` reliable tx channel, pending limits (16 requests or 256 KiB per viewer, 64 or 1 MiB per robot), then two token buckets at the channel's `max_hz`, one per viewer and one per robot (`rate_limited`). Accepted values are sent on the reliable robot carrier, and the bridge's ack or nack is routed back to the one viewer that asked. A request without an answer after 10 s settles as `publish_timeout`.
- Stats. `/api/stats` reports the robots, the viewer count, `pub` counters (accepted, acked, timed out, late acks, pending, rejected by code), per robot the subscribed channels, the teleop holder, the carrier queue and per-channel input rates, and per viewer the watched robot and per-channel output counters. On a viewer's channel, `sent` is frames accepted by the transport, `dropped` is frames shed before the wire (latest channels only), `aborted` is latest streams reset while the viewer was not keeping up (the stalled-viewer signal, 0 when healthy), and `expired` is routine end-of-life resets (about equal to `sent` on a healthy latest channel).

## Hosting

The [hosting guide](/docs/web/relay_hosting.md) puts a relay on a VM with Docker, certbot and the auth file, with a systemd variant and a LAN recipe with mkcert. The image is [`docker/relay/Dockerfile`](/docker/relay/Dockerfile) and [`docker/relay/compose.yaml`](/docker/relay/compose.yaml) runs it.
