# 4. Web cockpit and relay

These checks need the `web` extra and Chromium or Firefox. They run in simulation or on replay data unless a check says otherwise. The local relay is a Deno server that `dimos` downloads on first use. See the [web docs](../../docs/web/index.md).

## 4.1 Cockpit

```bash
# The first run downloads Deno and builds the web bundles.
uv run dimos --simulation run unitree-go2-cockpit
```

* Open http://127.0.0.1:7780/ and inspect the video, 3D map and 2D map together. The robot pose and visible obstacles should agree with the simulated scene.
* Click a free spot on the 2D map. A path is drawn and the robot drives there. Click `cancel` and the robot stops.
* Click the teleop panel so it reads `armed`. Drive with WASD and strafe with Q/E. Space stops the robot.
* Hold W and close the tab. The robot stops within about a second.

Expected result: the views agree with the robot's motion, and navigation, keyboard controls and stopping on tab closure work.

## 4.2 Agentic cockpit

Needs the agent credentials, ffmpeg and a microphone.

```bash
uv run dimos --simulation run unitree-go2-agentic-cockpit
# In another terminal, once the page is open:
uv run dimos agent-send "say hello"
```

* In the Agent chat panel, type "move forward 1 meter" and click send. "thinking..." shows, then the reply, and the robot moves on the maps. Wait for each movement and final reply before sending the next command.
* Hold the talk button next to the input, say "please rotate 180 degrees", and release. The transcript shows the text and the robot turns.
* The `agent-send` text and its reply appear in the same transcript.

Expected result: typed, spoken and `agent-send` commands all reach the agent, and the robot acts on them.

## 4.3 WebRTC video

Needs a Cloudflare Realtime app id and secret in `rtc.json`, see [Relay hosting](../../docs/web/relay_hosting.md#video-through-cloudflare).

```bash
uv run dimos --replay run unitree-go2-cockpit --rtc-file rtc.json
# In another terminal:
curl http://127.0.0.1:7780/api/info
```

* Open http://127.0.0.1:7780/ in Chromium and in Firefox. The video panel plays in both.
* Switch the header from `panels` to `channels`. `color_image` is listed with encoding `video.webrtc.v1`.

Expected result: `/api/info` answers `"rtc": true` and the video plays over WebRTC in both browsers.

## 4.4 Hosted relay with TLS and auth

Needs Docker, mkcert with libnss3-tools and Firefox on the relay machine, following the LAN recipe in [Relay hosting](../../docs/web/relay_hosting.md). No robot is needed.

```bash
# <relay-ip> is this machine's LAN address. <key> and <token> are two outputs of: openssl rand -hex 32
mkcert -install
mkdir -p docker/relay/config && mkcert -cert-file docker/relay/config/fullchain.pem -key-file docker/relay/config/privkey.pem <relay-ip>
echo '{"robots": {"go2-lab": "<key>"}, "viewers": {"tester": "<token>"}}' > docker/relay/config/auth.json
sudo chown -R 1993:1993 docker/relay/config
sudo docker compose -f docker/relay/compose.yaml up -d --build
uv run dimos --replay --relay-url https://<relay-ip> run unitree-go2-cockpit --relay-ca "$(mkcert -CAROOT)/rootCA.pem" --robot-id go2-lab --relay-key <key>
curl -H "Authorization: Bearer <token>" https://<relay-ip>/api/stats
sudo docker compose -f docker/relay/compose.yaml restart relay
```

* In Firefox set `network.http.http3.disable_when_third_party_roots_found` to false in about:config and restart it. Open https://<relay-ip>/ and enter `<token>`.

Expected result: after the token the page shows the replay robot's video and maps, the stats call lists `go2-lab` under `robots`, and after the relay restart the page and the robot reconnect by themselves.

## 4.5 Hosted teleop

Needs a physical Go2, an API key from the [Dimensional console](https://console.dimensional.org/console/keys) and internet access from the robot machine. See [Remote teleop](../../docs/capabilities/teleoperation/hosted.md).

```bash
TRANSPORTS__BROKER__API_KEY=<api-key> uv run dimos run teleop-hosted-go2-transport teleop-recorder
# After Ctrl+C:
ls ~/.local/state/dimos/teleop_recordings/
```

* Go to https://console.dimensional.org/console/teleop, sign in and click Connect on the robot under Available Robots. Video and the metrics HUD appear.
* Drive with WASD. The robot stops as soon as you release the key.
* Click a point on the minimap. The robot navigates there.
* Press E-STOP. The robot stops and damps. Clear the E-STOP, send RecoveryStand, and drive again.

Expected result: every command acts on the robot, and after Ctrl+C the directory holds `recording_teleop_<ts>.db` and `report_<ts>.json`.
