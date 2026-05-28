<div align="center">

### Team Perception

# 🦮 Goldie

### A phone-first guide-dog interface for the Unitree Go2

**Goldie** lets a low-vision or blind user point a real quadruped robot at a destination, hear it confirm out loud, and follow it there — using only their phone.

It's an iPhone-tuned PWA wired to an LLM-driven agent stack built on top of [DimOS](https://github.com/dimensionalOS/dimos), with real-time teleop fallback and a voice loop tuned end-to-end for accessibility.

### [Demo Video →](https://canva.link/kmcp30jdfr8ict0) · [Presentation →](https://canva.link/perception-goldie)

</div>

---

## The story

People who are blind or low-vision rely on guide dogs to navigate the world. Guide dogs work — but they're scarce, expensive, take years to train, and can't be summoned on demand. We wanted to see how close we could get to that experience with a quadruped robot, an LLM, and a phone.

The pitch: **you hold a button and say "find the bathroom." The dog confirms out loud, gets up, walks you there, and tells you when you've arrived.**

We started from the [DimOS](https://github.com/dimensionalOS/dimos) robotics SDK — which already handles WebRTC to a Unitree Go2, ROS-compatible transports, LCM message bus, MCP-driven LLM agents, and SLAM/navigation — and built two things on top:

1. **Goldie, the phone app** — voice-first, every agent reply spoken back, barge-in to interrupt, manual joystick fallback.
2. **DimOS extensions** — wiring the agent's replies to the phone, a direct-move skill for stairs, and macOS support so we could develop without a Linux box.

---

## What we built

### 1. `webapp/` — Goldie (the phone app)

<table>
  <tr>
    <td align="center"><img src="docs/screenshots/splash.png" width="220"/><br/><sub>Splash</sub></td>
    <td align="center"><img src="docs/screenshots/voice.png" width="220"/><br/><sub>Voice mode</sub></td>
    <td align="center"><img src="docs/screenshots/manual.png" width="220"/><br/><sub>Manual mode</sub></td>
  </tr>
</table>

A Next.js 16 PWA, written ground-up during the hackathon. Designed to be opened in Safari on iPhone (Add-to-Home-Screen ready). The whole UI is one page with two modes.

| | |
|---|---|
| **Voice mode** | Hold-to-speak → on-device STT → query sent to the agent → agent replies stream back over SSE → phone speaks them via OpenAI TTS. Includes **barge-in**: starting a new utterance cancels the in-flight reply. |
| **Manual mode** | Analog joystick → Socket.IO `move_command` Twist at 15 Hz directly to the dog, bypassing the LLM. For when you just want to drive. |
| **Quick actions** | Sit / Stand / Jump buttons that go through the agent so it narrates the action. |
| **Interrupt** | Cuts the agent off mid-task and silences speech instantly. |
| **Status feed** | Live SSE feed — tool/status lines shown dimmed, agent replies spoken aloud. |

### 2. `dimos/` — DimOS extensions

| Change | What it does |
|---|---|
| **Typed agent message envelopes** (`dimos/agents/web_human_input.py`) | `WebInput` subscribes to the agent's LCM `/agent` topic and forwards each message as `{kind: "ai"\|"tool"\|"system", text}` — so the phone knows what to *speak* vs what to just *show*. |
| **Direct `move` skill with stall recovery** (`dimos/robot/unitree/unitree_skill_container.py`) | The LLM can issue short velocity commands when the global planner can't find a path. Watches odometry, stops early on stalls, performs a reverse-recovery if blocked. |
| **macOS support fixes** | Fixed the full stack to run on Apple Silicon for development and demos. |

---

## Architecture

![Goldie architecture](./docs/goldie-architecture.png)

See [`webapp/TECHFLOW.md`](webapp/TECHFLOW.md) for a full end-to-end trace of every channel.

---

## Climbing stairs

One of our proudest moments was getting the Go2 to climb a real staircase under LLM control.

The existing DimOS navigation stack treats stairs as obstacles — the costmap-based planner won't route through them. To get past this we added a **direct `move` skill** to `UnitreeSkillContainer`: the LLM can issue short velocity commands (`x`, `y`, `yaw`, `duration`) for local maneuvering when the global planner gives up. The skill watches odometry chunk by chunk, stops early if the robot stalls, and performs a small reverse recovery if blocked.

We also updated the agent's system prompt with the right decision tree: try `relative_move` first; if no path is found, verify with `observe`, then call `move` with conservative velocity and duration. If `move` reports a stall, assess and reroute rather than keep pushing.

The result: the agent navigates normally on flat ground, then when it hits stairs it switches to direct velocity control, climbs them step by step, and resumes normal navigation at the top.

---

## Achievements

- ✅ **Full voice loop on iPhone** running against a live Go2
- ✅ **Typed agent message envelopes** so the phone speaks only AI replies and treats tool output as status
- ✅ **Stair climbing** under LLM control via direct-move skill with stall recovery
- ✅ **Real-time joystick teleop** as a manual fallback (Socket.IO Twist @ 15 Hz)
- ✅ **iOS PWA** — Add-to-Home-Screen, safe-area layout, retry-hardened HTTP client
- ✅ **macOS support** so the stack runs on Apple Silicon

---

## Challenges

**1. Networking split-brain.** The Go2 communicates over its own WiFi. To send commands you have to be on the dog's network — but the agent needs internet to call the OpenAI API. Setting up routing so both could work simultaneously burned more time than expected.

**2. Crashes and memory pressure.** SLAM, WebRTC, LCM, a live LLM agent, and Whisper STT all running in the same process on development hardware. We regularly hit memory limits causing silent freezes mid-session with no clean error — debugging was often just figuring out whether the dog, the network, or the process had died.

**3. Connection timeouts.** The WebRTC link to the dog would occasionally stall without dropping cleanly — commands appeared to send but never arrived, and the dog would stop responding mid-navigation with no indication on the backend.

---

## Quick start

### Run the webapp (no robot needed)

```bash
cd webapp
npm install
npm run dev      # http://localhost:3000
```

With no `.env.local`, Goldie runs against a built-in mock — the full UI works with no DimOS process running. For real OpenAI TTS:

```bash
OPENAI_API_KEY=sk-...
NEXT_PUBLIC_DIMOS_API=https://your-dimos-host
NEXT_PUBLIC_DIMOS_VIS=https://your-vis-host     # joystick
```

### Run against a real Go2

```bash
uv venv --python 3.12 && source .venv/bin/activate
uv pip install -e '.[base,unitree]'

uv run dimos --viewer none --robot-ip 192.168.12.1 run unitree-go2-agentic --disable security-module
```

Wait for the backend to come online:

```bash
uv run dimos status
```

Then set `NEXT_PUBLIC_DIMOS_API=http://localhost:5555` in `webapp/.env.local` and start the webapp.

---

## Tech stack

**Webapp:** Next.js 16 · React 19 · TypeScript · Tailwind v4 · Socket.IO · OpenAI `gpt-4o-mini-tts` · Vitest

**DimOS stack:** Python 3.12 · FastAPI · ReactiveX · LCM · LangChain + MCP · Whisper STT · Unitree WebRTC SDK

---

## Credits

Built at the MuShanghai DimOS Hackathon 2026 by **Team Perception**:
Joy Munn · Yichu Lau · Cecilia Zhang · Brecht Davos · Figo Saleh

Built on top of [DimOS](https://github.com/dimensionalOS/dimos).

> [!NOTE]
> The upstream DimOS README has been replaced by this submission doc. For the original project docs see [dimensionalOS/dimos](https://github.com/dimensionalOS/dimos).
