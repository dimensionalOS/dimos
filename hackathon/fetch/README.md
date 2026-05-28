# Fetch

Hackathon submission from **Team Pivot** — Philip Seifi ([@seifip](https://github.com/seifip)), Wenjie Fu ([@Wenjix](https://github.com/Wenjix)), and GuoZi ([@GuoZhuoRan](https://github.com/GuoZhuoRan)).

**A Unitree Go2 robot dog that trades ice-cold Cokes for instant photos.**

## If You Only Have 90 Seconds

1. Watch the demo: https://www.youtube.com/watch?v=8hHYE1239wg
2. Full source: https://github.com/seifip/robodog-fetch
3. Run it yourself (zero hardware — phone or laptop browser camera):
   https://github.com/seifip/robodog-fetch#quickstart-run-it-yourself

## One Sentence

Fetch works a crowd like a tiny, soda-carrying street performer: a vision LLM
"reads the room" on every camera frame and decides where to move, what to say, and
when to snap the photo — all running as a single FastAPI + WebSocket server you can
try from a phone browser **before any robot is involved**.

```text
camera frame (+depth)
  -> vision LLM (OpenAI / Gemini)
  -> decision (state, cmd_vel, line, photo?)
  -> act (move Go2, speak, snap photo)
  ^------------------ ~1s scan loop ------------------v
```

## The opportunity

Fetch is an autonomous **brand ambassador** and **mobile vendor** — here for Coca-Cola
— that hands out product, creates a memorable branded moment, and walks away with the
guest's photo. The longer-term vision: fleets of autonomous robot-dog vendors that roam
the beach and **self-resupply** at beachside bars and vendors, or at dedicated
autonomous resupply stations.

## What Matters

- **DimOS is the runtime.** Fetch reuses the DimOS teleop web pattern (HTTPS phone
  UI, WebSocket camera frames, motion/speech/photo decisions) and drives a **real
  Unitree Go2 over DimOS's WebRTC stack** — with selectable connection modes
  (`auto` / `local_ap` / `local_sta`) so it reaches the dog on its local-AP network
  at `192.168.12.1` as well as standard Wi-Fi.
- **Fetch is the behavior layer.** The vision-LLM decision loop, persona, approach/
  trade/photo state machine, and voice all sit on top of DimOS primitives.
- **Real-time by design.** We benchmarked round-trip latency across vision and speech
  models (`scripts/latency_bench.py`) and run the fastest combo — **Gemini 2.5
  Flash-Lite** vision + **Cartesia Sonic** speech; camera frames are downscaled
  (≤640 px) before analysis, and the whole scan loop lands around one second.

## Why a beach?

Quadrupeds earn their keep on terrain wheels can't handle, so we built Fetch around
that. We chose **sand** for a form-factor reason: the Go2's camera sits low and looks
*up* at standing people, but on a beach people sit or lie on the sand — dropping into
the dog's natural eye-line and making the interaction feel natural. And it's feasible
today: quadrupeds already run on sand
([RaiBo](https://techxplore.com/news/2023-01-raibo-versatile-robo-dog-sandy-beach.html)
at 3 m/s) and [sand-walking foot
adaptations](https://www.popsci.com/technology/robot-moose/) cut foot sinkage ~46%.

## What We Built

- A vision-LLM that turns each frame into a structured decision: target, `cmd_vel`,
  spoken line, and photo-framing readiness.
- The full interaction flow: scan for a relaxed guest → obstacle-aware approach
  (turning to keep the subject in frame) → wave + personalized one-liner →
  "grab a Coke, pose" → snap the instant photo + dance.
- **Instant, Fetch-branded photo → the guest's phone.** Each capture is composited
  with the Fetch logo in a Polaroid-style branded photo view (with a print sound),
  saved locally and optionally mirrored to an iCloud or Google Drive folder
  (`FETCH_PHOTO_MIRROR_DIRS`) so the demo phone syncs it seconds after it's taken.
- A single-page phone UI (camera feed, previews, live decision display, audio
  routing, photo flow) backed by FastAPI + WebSocket.
- Runtime-switchable **voice**: one-way TTS across Cartesia / Gemini Live / OpenAI,
  plus opt-in **two-way Gemini Live** conversation that drives the dog through tool
  calls (`accept_offer`, `take_photo`, `celebrate`, `do_trick`, `stop_and_reset`).
- Safety/privacy guardrails: humor limited to visible, non-sensitive context;
  LiDAR/depth-enforced `<4 m` stop and obstacle avoidance.

## Under the Hood (for the technically curious)

| Piece | What it does |
| --- | --- |
| **Camera sources** | One loop, three inputs: phone browser camera (zero hardware), Record3D USB RGBD (real iPhone LiDAR depth), and a live Go2 over WebRTC. |
| **Vision policy** | `FetchPolicy.analyze_frame()` sends image + prompt to a provider-selectable vision LLM (OpenAI or Gemini) and normalizes the JSON into a decision dict; the live demo runs on **Gemini 2.5 Flash-Lite** for the lowest latency. |
| **Go2 transport** | DimOS Unitree WebRTC; `--robot-connection-method auto\|local_ap\|local_sta` (default `local_ap`) + `--robot-ip` select how to reach the dog. |
| **Voice** | Provider-switchable TTS at runtime (no restart) plus an optional persistent Gemini Live session with server-side VAD / barge-in. |
| **Photos** | Fetch-branded capture (logo composited via `<canvas>`) to `static/captures/`, optionally mirrored to iCloud/Drive folders via `FETCH_PHOTO_MIRROR_DIRS`. |
| **Tests** | **76 passing tests**, all providers mocked — no live API calls needed to review (policy, middleware routes, TTS, conversation tools, photo saving). |

## Reviewer Map

| Question | Open this |
| --- | --- |
| What is the demo? | https://www.youtube.com/watch?v=8hHYE1239wg |
| Where is the full source? | https://github.com/seifip/robodog-fetch |
| How do I run it (no hardware)? | https://github.com/seifip/robodog-fetch#quickstart-run-it-yourself |
| How does the decision loop work? | https://github.com/seifip/robodog-fetch#how-it-works-at-a-glance |
| What's the DimOS integration? | https://github.com/seifip/robodog-fetch#built-on-dimos |
| Where's the DimOS runtime? | https://github.com/dimensionalOS/dimos |

## How to Run

Zero-hardware path (phone or laptop browser camera), from the DimOS monorepo root:

```bash
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455
```

Open `https://127.0.0.1:8455/fetch` and tap **Record** to start the ~1-second scan
loop. To drive a real dog, add `--robot-ip 192.168.12.1 --robot-connection-method
local_ap`. The full quickstart (Record3D USB, live Go2, provider keys, and voice
modes) is in the project README.

## What's Next

- **Sense the trade.** The Go2 EDU's [foot-force sensors](https://www.unitree.com/go2/foot/)
  could detect a Coke lifted from the back via the change in total load — closing the
  loop without the camera's framing check.
- **Real sand.** Fit sand-walking foot adaptations for an outdoor beach deployment.

## Scope Boundary

This PR is a hackathon **submission pointer**: the full source, demo video, and
assets are hosted externally at https://github.com/seifip/robodog-fetch. It adds a
single markdown file under `hackathon/` and **does not vendor Fetch into DimOS or
modify any DimOS runtime code**. (Fetch is designed to live at
`dimos/experimental/fetch/` in the monorepo; that vendoring is intentionally out of
scope for this pointer.)

## Validation

- `pytest -q` in the project repo: **76 passed**, providers mocked (no real API
  calls) — covers policy normalization, middleware routes, TTS, conversation tools,
  and photo saving.
- This submission adds markdown only; no DimOS code is touched.
