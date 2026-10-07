# Relay hosting

The relay on your own server, under your own name: robots dial out to it with a key, operators open it in a browser with a token, and the cockpit works from anywhere. This guide takes a fresh Ubuntu VM and a domain to `https://dimos-relay.example.com`. The [LAN section](#lan-without-a-domain) does the same on a LAN without a domain. Everything here is packaging: a container image ([`docker/relay/Dockerfile`](/docker/relay/Dockerfile)), a compose file ([`docker/relay/compose.yaml`](/docker/relay/compose.yaml)), and the steps around them. The flags themselves are explained on the [Relay](/docs/web/relay.md) page.

## What you get, and what you do not

- One relay process on one machine. The registry is in memory: after a restart every robot re-registers by itself within seconds and every open page reconnects.
- No accounts. The auth file holds robot keys, each bound to one robot id, and viewer tokens. A viewer token does everything (view, teleop, publish) on every robot behind the relay, so hand tokens out like passwords.
- The relay terminates TLS itself, on one port for HTTPS (TCP) and QUIC (UDP). No reverse proxy: an HTTP proxy cannot carry QUIC, and the relay advertises the WebTransport URL from the host the client dialed.
- The certificate and the auth file are read at startup. Changing either means a restart, which costs a few seconds of reconnection.

## DNS and firewall

Point an A record at the VM (`dimos-relay.example.com -> <public IP>`). Open 443 TCP, 443 UDP, and 80 TCP for certbot (nothing listens on 80 between renewals). The cloud provider's own firewall (security group) needs the same rules. UDP 443 is the one people forget.

```bash
sudo ufw allow 22/tcp && sudo ufw allow 80/tcp && sudo ufw allow 443/tcp && sudo ufw allow 443/udp
sudo ufw enable
```

Without UDP 443 there is no WebTransport and, by design, no fallback: the page loads and never connects.

## Certificate

Clone dimos onto the VM (the image is built from the checkout) and install Docker and certbot. Request the certificate with a deploy hook that installs the two PEM files into the relay's config directory, owned by uid 1993 (the image's `deno` user), and restarts the relay:

```bash
sudo apt install -y git certbot docker.io docker-compose-v2
sudo git clone https://github.com/dimensionalOS/dimos /srv/dimos
sudo mkdir /srv/dimos/docker/relay/config
sudo certbot certonly --standalone -d dimos-relay.example.com --deploy-hook \
  'install -o 1993 -g 1993 -m 600 "$RENEWED_LINEAGE"/fullchain.pem "$RENEWED_LINEAGE"/privkey.pem /srv/dimos/docker/relay/config/ && docker compose -f /srv/dimos/docker/relay/compose.yaml restart relay'
```

The hook runs now (so `config/` holds `fullchain.pem` and `privkey.pem`, and the restart is a no-op until the relay exists) and certbot saves it in the renewal configuration: every automatic renewal (certbot's timer, roughly every 60 days) installs the new files and restarts the relay, which reads certificates only at startup. `sudo certbot renew --dry-run` checks renewal but skips hooks, so rehearse the restart once by running the hook command by hand after the [Run](#run) step.

## Auth file

`config/auth.json` maps robot ids to keys and viewer names to tokens, each 16 to 256 characters and no secret used twice. Generate them with `openssl rand -hex 32`:

```json
{
  "robots": { "go2-lab": "<key>" },
  "viewers": { "paul": "<token>" }
}
```

```bash
sudo chown 1993:1993 /srv/dimos/docker/relay/config/auth.json
sudo chmod 600 /srv/dimos/docker/relay/config/auth.json
```

A robot id here has to match the robot's `--robot-id` (its hostname when not given). Adding or rotating a secret is an edit plus `docker compose restart relay`. The relay refuses a malformed file with a message that names the entry, never the secret, and it never logs secrets.

## Run

```bash
cd /srv/dimos/docker/relay
sudo docker compose up -d --build
curl https://dimos-relay.example.com/api/info
sudo docker compose logs -f relay
```

The first build takes a few minutes: Deno installs the web dependencies and builds the SDK and the cockpit inside the image. `/api/info` answers `{"wtUrl":"https://dimos-relay.example.com","v":...}`. The log opens with the ready line and then shows connections (`[relay] robot connected`, `[relay] viewer 1 authenticated as paul`), rejections with their reason (`[relay] robot go2-lab rejected: invalid robot key`), and `[relay] robot go2-lab disconnected` when one leaves. `restart: unless-stopped` brings the relay back after a reboot. A new dimos version is `git pull` and the same `up -d --build`.

## Robot

The robot needs the key bound to its id, from the environment or the `.env` file of the checkout it runs from (the `--relay-key` flag exists but shows in the process list). The robot-side flags are listed on the [Bridge](/docs/web/bridge.md#robot-side-options) page:

```bash
RELAY_KEY=<key> dimos run unitree-go2 --relay-url https://dimos-relay.example.com --robot-id go2-lab
```

It dials out, so it works behind NAT with no open ports. A wrong key is logged once as `auth_failed` and the bridge gives up rather than retrying: fix `RELAY_KEY` and start again.

## Operator

Open `https://dimos-relay.example.com/`. The [cockpit](/docs/web/cockpit.md) asks for a viewer token, keeps it in the browser's local storage, and "Log out" in the status bar forgets it. With several robots registered the page lists them. "switch robot" reopens the list. `/api/stats` (per-robot, per-channel, and per-viewer counters, with viewer names) takes the same token as a bearer:

```bash
curl -H "Authorization: Bearer <token>" https://dimos-relay.example.com/api/stats
```

## Video through Cloudflare

Over the internet the JPEG video path is the slow part: every frame crosses the robot's uplink and the relay at full size. With a Cloudflare Realtime SFU behind the relay, `jpeg.v1` video channels become H.264 WebRTC tracks instead. The robot encodes once, Cloudflare fans the track out from its nearest edge, and the relay only brokers the signaling. Everything else (odom, map, chat, teleop) stays on WebTransport through the relay as before.

1. In the Cloudflare dashboard create a Realtime SFU app (Realtime -> SFU -> Create) and note its app id and secret. Optionally create a TURN key (Realtime -> TURN) for robots and browsers on networks that block UDP.
2. Write `config/rtc.json` next to `auth.json`, with the same ownership and mode. Leave out the two TURN fields to run with STUN only:

```json
{
  "appId": "<app id>",
  "appSecret": "<app secret>",
  "turnKeyId": "<turn key id>",
  "turnToken": "<turn api token>"
}
```

3. Add the flag. In the compose file set `command:` to the image's file flags plus `--rtc-file /etc/relay/rtc.json` (the commented example in [`compose.yaml`](/docker/relay/compose.yaml#L14)), or append the flag to the systemd `ExecStart` in [Without Docker](#without-docker). Restart the relay.

`/api/info` now answers `"rtc": true` and the log says `WebRTC video through the Cloudflare Realtime SFU: on`. A robot needs the `webrtc` extra (`uv sync --extra webrtc`, or `pip install 'dimos[webrtc]'`, which brings aiortc). Without it the bridge logs once and keeps sending JPEG. `--rtc false` on the robot keeps JPEG on purpose (a comparison, or a robot without the CPU for H.264). A track channel sends nothing through the relay: `/api/stats` lists the SFU sessions and pulls under `rtc`, and the per-channel counters of `color_image` stay at zero. Cloudflare bills egress past 1 TB a month. The relay's own traffic drops to control messages.

Troubleshooting: `rtc_failed` in the cockpit status bar names the failing Cloudflare call (the relay log has the full line, for example `HTTP 401` for a wrong app secret). The robot log says `WebRTC video failed ... re-offering` while the relay never answers its offer, and again whenever its peer connection to Cloudflare drops (it offers again by itself). With no TURN key both ends must reach Cloudflare over UDP.

## Without Docker

The same relay under systemd: the pinned Deno, a checkout owned by a service user, the built bundles, and the right to bind 443.

```bash
sudo apt install -y git unzip
curl -fsSLo /tmp/deno.zip https://github.com/denoland/deno/releases/download/v2.6.10/deno-x86_64-unknown-linux-gnu.zip
sudo unzip -o -d /usr/local/bin /tmp/deno.zip
sudo useradd --system --create-home --home /srv/relay relay
sudo -u relay git clone https://github.com/dimensionalOS/dimos /srv/relay/dimos
sudo -u relay sh -c 'cd /srv/relay/dimos/web && deno install --frozen && deno task -r build'
sudo mkdir /etc/relay
```

The pin is `DENO_VERSION` in [`dimos/utils/deno.py`](/dimos/utils/deno.py#L32). An arm64 VM downloads `deno-aarch64-unknown-linux-gnu.zip` instead. Put `auth.json` in `/etc/relay` owned by `relay` with mode 600, and use `install -o relay -g relay -m 600 "$RENEWED_LINEAGE"/fullchain.pem "$RENEWED_LINEAGE"/privkey.pem /etc/relay/ && systemctl restart dimos-relay` as the certbot deploy hook. `/etc/systemd/system/dimos-relay.service`:

```ini
[Unit]
Description=dimos relay
After=network-online.target

[Service]
User=relay
WorkingDirectory=/srv/relay/dimos/web
ExecStart=/usr/local/bin/deno run --frozen --allow-net --allow-read=/srv/relay/dimos/web,/etc/relay relay/main.ts --host 0.0.0.0 --port 443 --cockpit-dir cockpit/dist --sdk-dir sdk/dist --cert /etc/relay/fullchain.pem --key /etc/relay/privkey.pem --auth-file /etc/relay/auth.json
AmbientCapabilities=CAP_NET_BIND_SERVICE
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

`sudo systemctl enable --now dimos-relay`, then `journalctl -u dimos-relay -f` for the log.

## LAN without a domain

On a LAN with no public name, sign your own certificate with mkcert. It creates a private CA and a certificate for the relay's address. The robot trusts the CA through `--relay-ca`. The auth file stays mandatory: the relay binds a non-loopback address only with certificate, key, and auth file together. The image and the compose file are the same as on a VM. Only the certificate step changes. `192.168.1.20` below is the relay machine's LAN address.

```bash
sudo apt install -y mkcert libnss3-tools
mkcert -install
mkdir -p docker/relay/config
mkcert -cert-file docker/relay/config/fullchain.pem -key-file docker/relay/config/privkey.pem 192.168.1.20
```

`libnss3-tools` is what lets `mkcert -install` add the CA to Firefox. Write `auth.json` as in [Auth file](#auth-file), give all three files to uid 1993 the same way, and start the relay as in [Run](#run).

The robot needs the CA bundle: its QUIC client trusts certifi's roots, not the system store.

```bash
RELAY_KEY=<key> uv run dimos --simulation run unitree-go2-agentic-cockpit --relay-url https://192.168.1.20 \
  --relay-ca "$(mkcert -CAROOT)/rootCA.pem" --robot-id go2-lab
```

Browsers are the catch. Firefox trusts the certificate for the page but drops QUIC through a root that is not built in. In `about:config` set `network.http.http3.disable_when_third_party_roots_found` to `false` and restart Firefox. Then open `https://192.168.1.20/` and enter the viewer token. Chromium refuses a private root for QUIC by default (`QUIC_CERT_ROOT_NOT_KNOWN`): the page loads over HTTPS and never connects. Its `--webtransport-developer-mode` switch turns that check off, which is fine for a development machine. Anyone else on Chromium needs a publicly trusted certificate, which a domain you own provides without any inbound port: point a name at the LAN address and use certbot's DNS challenge (`certbot certonly --manual --preferred-challenges dns -d relay.lan.example.com`).

Without Docker, the command from [Without Docker](#without-docker) works with the mkcert files in place of the certbot ones.

## Troubleshooting

- **`auth_failed`.** The relay's message says `missing` (the client sent no secret: `RELAY_KEY` not in the robot's environment, no token stored in the browser) or `invalid` (wrong secret, or a robot key used with another robot id). It is terminal: the bridge stops and the page shows the token form. Fix the secret and start the client again. The relay log names the rejected robot id or viewer.
- **`accept failed: aborted by peer: the application or application protocol caused the connection to be closed during the handshake`**, repeating every 8 s, with no `viewer N authenticated` line. A browser is retrying and closing the connection itself after the TLS check: with a private CA, Firefox without the `about:config` pref from [LAN without a domain](#lan-without-a-domain), or Chromium. The page shows "Waiting for a robot to register..." meanwhile, which also covers a page still connecting. The status bar shows the real state. The robot and `/api/stats` are unaffected.
- **Certificate errors.** Check the VM clock (`timedatectl`). The robot fails with `CERTIFICATE_VERIFY_FAILED` when the relay serves `cert.pem` instead of `fullchain.pem` (browsers fetch the missing intermediate, OpenSSL does not) or when `--relay-ca` points at the wrong CA.
- **UDP 443 blocked.** HTTPS works, the page loads and stays "reconnecting", the robot's startup attempts time out (`relay startup connection attempt N failed`) and it gives up. Check both firewalls for UDP. `sudo ss -ulnp | grep 443` on the VM shows the published UDP port.
- **Robot behind NAT.** Fine: it dials out, and the QUIC session sends keepalives every 4 s.
- **Relay behind NAT** (a home server). Forward TCP 443 and UDP 443 to it. `/api/info` advertises the name the client dialed, so nothing else changes.
- **Restarts.** `docker compose restart relay` is always safe: robots re-register within seconds, pages reconnect on their own. A robot killed without a clean close holds its id for up to 30 s (the QUIC idle timeout), and its restart waits that out.

## Checklist

What a hosted relay is verified with. Repeat it after changes to the relay, the image, or this guide.

- [ ] Fresh Ubuntu VM with a domain, DNS through Run top to bottom, then `curl https://.../api/info` from a laptop.
- [ ] Replay robot on another, NATed network (`dimos --replay run unitree-go2 --relay-url ...`).
- [ ] Laptop on a third network, in Chromium and in Firefox: wrong token rejected, right token connects, robot picker with two robots, live map and video, teleop drives the replay robot.
- [ ] `docker compose restart relay` with the page open: the page reconnects, the robot re-registers.
- [ ] The deploy hook by hand: the relay restarts with the installed files, and `sudo certbot renew --dry-run` passes.
- [ ] `/api/stats`: 401 without the token, 200 with it.
- [ ] The LAN recipe: the robot registers with `--relay-ca`, Firefox with the pref connects, Chromium without its developer switch loads the page and never connects.
- [ ] [Video through Cloudflare](#video-through-cloudflare) with `rtc.json` installed: the video panel plays a WebRTC track in Chromium and Firefox (`/api/stats` shows one `rtc.pulls` per viewer, zero `color_image` frames through the relay), and a robot without aiortc stays on JPEG.
