# Fetch

Hackathon submission from **Team Pivot** — Philip Seifi ([@seifip](https://github.com/seifip)), Wenjie Fu ([@Wenjix](https://github.com/Wenjix)), and GuoZi ([@GuoZhuoRan](https://github.com/GuoZhuoRan)).

**A Unitree Go2 robot dog that trades ice-cold Cokes for instant photos.**

## If You Only Have 60 Seconds

1. Watch the demo: _publishing to YouTube — link coming shortly._
2. Read the project: https://github.com/seifip/robodog-fetch
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

## What Matters

- **DimOS is the runtime.** Fetch reuses the DimOS teleop web pattern — a FastAPI
  server serves an HTTPS phone UI, the phone streams camera frames over a WebSocket,
  and the server returns motion / speech / photo decisions. On the dog it drives
  DimOS Unitree WebRTC control + LiDAR.
- **Fetch is the behavior layer.** The vision-LLM decision loop, persona, approach/
  trade/photo state machine, and voice all sit on top of DimOS primitives.
- **Three camera sources, one loop:** phone browser camera (zero hardware), Record3D
  USB RGBD (real iPhone LiDAR depth), and a live Unitree Go2 over Wi-Fi.

## What We Built

- A vision-LLM that evaluates each frame and emits a structured decision: target,
  `cmd_vel`, spoken line, and photo-framing readiness.
- The full interaction flow: scan for a relaxed guest → obstacle-aware approach →
  wave + personalized one-liner → "grab a Coke, pose" → snap the instant photo + dance.
- A single-page phone UI (camera feed, previews, decision display, audio routing,
  photo flow) backed by a FastAPI + WebSocket server.
- Runtime-switchable **voice**: one-way TTS across Cartesia / Gemini Live / OpenAI,
  plus an opt-in **two-way Gemini Live** conversation that drives the dog through
  tool calls (`accept_offer`, `take_photo`, `celebrate`, `do_trick`, `stop_and_reset`).
- Safety/privacy guardrails: humor constrained to visible, non-sensitive context;
  LiDAR/depth-enforced `<4m` stop and obstacle avoidance.

## Reviewer Map

| Question | Open this |
| --- | --- |
| What is the demo? | _Publishing to YouTube — link coming shortly._ |
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
loop. The full quickstart (Record3D USB and live Go2 paths, provider keys, and the
voice modes) is in the project README.

## Scope Boundary

This PR is a hackathon **submission pointer**: the full source, demo video, and
assets are hosted externally at https://github.com/seifip/robodog-fetch. It adds a
single markdown file under `hackathon/` and **does not vendor Fetch into DimOS or
modify any DimOS runtime code**. (Fetch is designed to live at
`dimos/experimental/fetch/` in the monorepo; that vendoring is intentionally out of
scope for this submission.)

## Validation

- `pytest -q` in the project repo is green — all provider calls are mocked, so there
  are no real API calls (covers the 26 middleware tests plus the policy, conversation,
  and TTS suites).
- This submission adds markdown only; no DimOS code is touched.
