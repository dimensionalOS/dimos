# FRANK: Go2 social agent demo — feasibility spec

Status: proposal, 2026-09-03.

## The demo

1. A QR sticker on the Go2 opens a chat web app on the visitor's phone.
2. Onboarding: "Introduce yourself to FRANK." Visitor types a name and takes a selfie.
3. Chat UI. The visitor talks to the agent that drives the Go2.
4. FRANK recognizes people from the selfie dictionary and from what it has seen (current and historical).
5. When idle, FRANK picks a task from a list, for example "find someone you talked to 30 min ago and ask a follow-up," with per-person rate limits.

The agent is Pi, Claude Code, or Codex running on the laptop, driving the robot through the `dimos mcp call` CLI (mostly `move_to`). Nothing new runs on the robot. DimOS itself runs on the laptop and talks to the Go2 over WebRTC. A Bluetooth speaker sits on the robot, paired to the laptop, so speech is a laptop-side script that hits ElevenLabs and plays the result. We control the harness skill, so anything hard for the LLM can be made deterministic in scripts.

## Verdict

Feasible for a demo in roughly two weeks of focused work. Every robot-side primitive exists today. The new work all lives on the laptop: a small "FRANK server" (chat app, people directory, face matching, idle scheduler, agent driver) and a Claude Code skill with a few scripts (speak, scan, recent sightings). No new DimOS modules are required. The two real risks are navigation reliability in a crowded room and face recognition from the Go2's wide-angle camera at distance. Both have cheap fallbacks that still make a good demo.

## What exists in the repo

| Need | Exists | Where | Gap |
|---|---|---|---|
| External agent → robot skills | Yes | `McpServer`, streamable HTTP at `localhost:9990/mcp`, exposes every `@skill` | None. `claude mcp add --transport http dimos http://localhost:9990/mcp` |
| Camera frame to the agent | Yes | `observe()` skill returns `Image.agent_encode()` inline as MCP image content | None |
| Navigate to a world pose | Yes | `move_to(x, y, degrees, relative)`, `navigate_with_text`, `stop_navigation` | Needs a map; use `unitree_go2_spatial` stack |
| Approach / follow a person | Yes | `follow_person`, `stop_following` (EdgeTAM), `look_out_for(..., then=...)` | None |
| Speak | Yes, via laptop audio | Bluetooth speaker on the robot, paired to the laptop. `SpeakSkill` already plays through the laptop's default device | Replace with a `speak.py` script in the harness skill: ElevenLabs → play. No DimOS change. (Go2 Pro's own speaker is also reachable via `unitree_speak.py`, not needed) |
| Hear | Optional | Go2 WebRTC driver receives robot mic frames but nothing consumes them. `WhisperNode` STT exists | Skip for v1. If wanted, push-to-talk in the phone PWA is the reliable path |
| Wake the agent on a perception event | Yes | `look_out_for(query, then=prompt)` → `dispatch_continuation` | Only for the in-process `McpClient` agent. External agent needs its own wake path (below) |
| Memory: where/when was X seen | Yes | `dimos/memory` Stream API: `.tags(name=..)`, `.after(t)`, `.near(pose, r)`, `.last()`; SQLite; `Recorder` for Go2 | No people stream yet. Add a `person_sightings` stream |
| Memory: natural-language history | Yes | `TemporalMemory.query(question)` (VLM entity graph, `unitree-go2-temporal-memory` blueprint); `SpatialMemory.query_by_text/image` | Entity ids are VLM labels, not identities |
| Face recognition | No | Only appearance re-ID (`EmbeddingIDSystem`, torchreid) and CLIP | Add face embeddings in the FRANK server, or VLM comparison |
| Browser chat UI | Legacy only | `WebInput` + Svelte app on port 5555 publishes to `/human_input`; the new web SDK / cockpit has no chat channel | Build a small chat PWA in the FRANK server |
| Idle task scheduler | No | `security_module.py` patrol→follow state machine is the closest template | Build in the FRANK server |
| Pi agent | No | Only `docs/development/pi_pointcloud_multistep_eval_proposal.md` | Pi/Claude Code both have programmatic session APIs |

## Proposed architecture

```
phone (PWA over HTTPS)
   │ QR → https://frank.<tailnet>/
   ▼
FRANK server (laptop, FastAPI + sqlite)
   ├─ /onboard  name + selfie → people/{id}.jpg + face embedding
   ├─ /chat     per-person transcript, websocket
   ├─ idle scheduler → wake prompts into the agent session
   └─ agent driver: Claude Agent SDK / Pi session, resumable
              │
              ▼
agent session (Claude Code) + harness skill `frank/`
   ├─ SKILL.md      decision procedures (chat, find-now, follow-up)
   ├─ speak.py      ElevenLabs → Bluetooth speaker
   ├─ scan.py       spin + capture + identify → one line
   ├─ sightings.py  search recent recorded frames / memory db
   ├─ people.py     list / chat_history / send_chat / task_done (talks to FRANK server)
   └─ dimos mcp call move_to | observe | follow_person | stop_navigation
              │ WebRTC (existing)
              ▼
        Go2  +  Bluetooth speaker
```

The agent is one long-lived session on the laptop. Every input it receives is a prompt injected by the FRANK server: either a chat message ("Alice says: ...") or a wake ("Idle 6 min. Task: follow_up. Candidate: Alice, last seen 31 min ago at (2.1, -0.4), last topic: ..."). Robot control is the existing `dimos mcp call` CLI. Everything else is scripts in the skill folder, so the tool surface stays small and deterministic.

### DimOS blueprint

Run an existing Go2 stack with `McpServer` and a `Recorder`, or compose `unitree-go2-frank` = `unitree_go2_spatial` + `McpServer` + skills (`move_to`, `navigate_with_text`, `follow_person`, `observe`) + `TemporalMemory` + `Recorder`. No `McpClient`, since the external agent is the brain. This is composition only, no new module code.

### Harness skill scripts

- `speak.py "text"` → ElevenLabs with a fixed `voice_id`, streamed to the laptop's default output (the Bluetooth speaker). Cache by text hash so repeated phrases are instant. Under a second to first sound with streaming.
- `scan.py [--spin] [--seconds N]` → optional 360° in 60° steps via `move_to(degrees=60, relative=True)`, grabs frames via `observe` or straight from the camera stream, runs face matching against the gallery plus body re-ID, prints one line: best match, confidence, bearing, range.
- `sightings.py --person alice [--since 120s]` → searches the memory database's recorded frames for the person and prints the newest hit with the robot pose at that instant. Also `--last-seen` for the `person_sightings` stream.
- `people.py list | history alice | send alice "text" | done <task_id> <outcome>` → thin client for the FRANK server. `record_sighting` appends to the memory db with the current TF pose.

## Feature by feature

### QR → chat app

Straightforward. One gotcha: phone cameras need a secure context, so the URL must be HTTPS. Use `tailscale serve` or `cloudflared` in front of the FRANK server. Print the QR for that URL. The QR can carry a `?robot=frank` param if more than one robot is ever on the floor.

### Onboarding and the people dictionary

Name + selfie form. Store `people/{id}/selfie.jpg`, compute a face embedding at enrollment, store name, embedding, timestamps in sqlite. Show a consent line ("FRANK will remember your face and chats for today") and provide a "forget me" button. Wipe the directory at the end of the demo day.

### Chat

Websocket per phone. Messages go into a queue. The agent is single-threaded, so the FRANK server serializes: one prompt in flight at a time, batching messages that arrive while it thinks ("Alice: hi. Bob: what's your name?"). Agent replies via `send_chat(name, text)`, and can also `speak` for people standing next to it. Expect 5 to 15 s per turn with Claude Code; show a typing indicator.

### Recognizing people

Three layers, cheapest first:

1. **Face embedding** (recommended primary). insightface `buffalo_l` runs on CPU at a few frames per second and matches well against a single selfie. Works when the face is within roughly 1.5 to 2 m of the Go2 front camera. Farther than that, faces are too small in the wide-angle frame.
2. **Appearance re-ID** as a secondary signal. On first confirmed sighting of the day, store a torchreid body embedding (`dimos/models/embedding/treid.py`) keyed to the name. Clothing is stable for a demo day, so this extends recognition range.
3. **VLM comparison** as the fallback. The agent calls `observe()` and `identify()`; if confidence is low it can look at the gallery selfies itself. For a gallery under ten people this works surprisingly well and needs no new deps. It is slow and expensive, so it stays the fallback.
4. **Ask.** "Are you Alice?" is a valid and charming last resort. Make this a rule in the system prompt.

Current vs historical memory: every confirmed identification appends a `person_sightings` observation to the DimOS memory db with `tags={name, confidence}`, `ts`, and the robot pose from TF. Then `last_seen(name)` is a one-line Stream query, and TemporalMemory's `query("who was near the couch 20 minutes ago?")` gives the fuzzy natural-language complement. The FRANK server can open the same SQLite store the `Recorder` writes, so no new transport is needed.

### Proactive follow-ups (idle task list)

The agent is reactive, so the scheduler lives in the FRANK server and fires prompts. The rate limiting is code, not prompt:

```
tasks:
  - follow_up:   candidates = people with chat_age in [20 min, 3 h]
                 and follow_ups_today(person) < 2
                 and last_follow_up(person) > 45 min ago
  - greet_known: candidates = people seen in last 2 min with no chat in 30 min
  - explore:     no candidates → wander / look_out_for("a person")
idle_after: 5 min without chat or task
cooldown_after_task: 3 min
```

A wake prompt gives the agent one task, one candidate, the transcript snippet, and the last-seen pose. The agent's plan is then: `move_to(last_seen pose)`, `observe()` + `identify()`, if not found `look_out_for("a person", then="identify them")` for a bounded time, if found `speak(follow-up question)` and `send_chat` the same question, then `task_done`. If `task_done` reports not found, the scheduler backs off for that person. Every wake has a hard timeout in the FRANK server; on timeout it sends "abort, stop_navigation" and marks the task failed.

Driving the agent: Claude Code via the Agent SDK `query()` with a resumed session, or `claude -p --resume <id>`. Pi has an equivalent programmatic loop. Codex works but has the least mature session API, so I'd pick Claude Code or Pi.

### Audio

**Speaking**: a Bluetooth speaker mounted on the Go2 and paired to the laptop. `speak.py` calls ElevenLabs streaming TTS and plays it on the laptop's default device. Sub-second to first sound, and no robot involvement. Pair the speaker before the demo and pin it as the default sink so a reconnect doesn't route audio to the laptop's own speakers. Bluetooth range is fine for a room; if the robot wanders further, the Go2 Pro's built-in speaker via `unitree_speak.py` is a fallback we don't need for v1.

**Hearing**: skip for v1. A demo floor is loud and the chat UI is a reliable input channel. If voice is wanted later, a push-to-talk button in the phone PWA is more reliable than the robot mic.

### Finding a person right now

"Find Alice" as the harness would run it, cheapest signal first, with a hard three-minute budget:

1. **Free signals before moving.** If Alice messaged in the last minute, send "coming to find you, give me a wave." Run `sightings.py --person alice --since 120s` over the recorded frames: if she walked past 40 s ago that pose is near-live. Read TemporalMemory's roster for how many people are in view.
2. **Look from here.** One `observe` + identify. Then `scan.py --spin`: six 60° turns, one frame each. Twenty seconds, no navigation risk.
3. **A person with no face match is a lead, not a miss.** Beyond about 2 m the face is too small. `move_to` about 1.5 m in front of the detection, or `follow_person` for 5 s, then re-identify. Body re-ID from her first confirmed sighting today usually settles it.
4. **Drive to candidates, bounded.** Freshest sighting, then last-seen pose, then where TemporalMemory says most people are, then tagged locations. Scan at each. Cap at three candidates.
5. **Confirm.** Below threshold, `speak.py "Alice?"` and watch a few seconds for a chat reply, a turn toward the robot, or an improving match. On budget exhaustion: `stop_navigation`, say "I couldn't find Alice, has anyone seen her?", and `people.py done <task> not_found` so the scheduler backs off.

`look_out_for(..., then=...)` isn't useful to an external agent: its continuation wakes the in-process `McpClient`. Polling inside `scan.py` is the right shape.

## Work breakdown

| Item | Effort | Notes |
|---|---|---|
| `speak.py` (ElevenLabs → Bluetooth) | 0.5 day | Streaming, cache, pin default sink |
| `unitree-go2-frank` blueprint (composition only) | 0.5 day | Or run an existing stack with `McpServer` + `Recorder` |
| FRANK server: PWA (onboard + chat), sqlite, websocket | 2 days | FastAPI + one static page; HTTPS via tailscale |
| Face matcher (insightface) + torchreid fallback, `scan.py` | 1 to 2 days | Enrollment + identify + spin scan |
| `sightings.py` over memory db + `person_sightings` stream | 0.5 to 1 day | Stream API |
| `people.py` + SKILL.md procedures | 1 day | Chat, find-now, follow-up |
| Agent driver (Claude Agent SDK / Pi session) + system prompt | 1 day | Serialization, timeouts |
| Idle scheduler + rate limits | 1 day | Config-driven task list |
| Integration and floor testing | 3 days | Nav tuning, recognition distance, prompt iteration |

Total: about 11 to 12 working days for one person.

## Risks and mitigations

- **Navigation in a crowd.** `move_to` a stale pose into a room full of people will fail sometimes. Mitigate: bounded attempts, `look_out_for` rather than driving far, and let the agent say "I'm looking for Alice, has anyone seen her?" as an on-brand failure.
- **Recognition range.** Faces are small beyond 2 m in the Go2 camera. Mitigate: appearance re-ID after first confirmed sighting, and asking.
- **Agent latency.** 5 to 15 s per turn. Mitigate: typing indicator, `speak` filler ("let me think"), and keeping the toolset small.
- **Single agent, many phones.** Serialize and batch. Cap concurrent chats with a friendly "FRANK is busy" if the queue exceeds a few messages.
- **Robot network.** The Go2 WebRTC link, the laptop, and the phones must all be reachable. Tailscale on the laptop covers the phones; the robot stays on its own AP or LAN.
- **Bluetooth audio.** Reconnects can silently reroute audio to the laptop speakers. Pin the sink, and have `speak.py` fail loudly if the Bluetooth device isn't the active output.
- **Privacy.** Biometric data of the public. Consent text at onboarding, a forget-me button, purge at end of day, no cloud storage of selfies beyond the TTS/LLM calls needed.

## Open decisions

1. Claude Code or Pi as the agent. Both work; Claude Code's Agent SDK session resume is the path I know best.
2. Face embeddings (new dependency, deterministic) vs VLM-only (no deps, slower). Recommend face embeddings in the FRANK server so DimOS itself gains no new dependency.
3. Whether to upstream a chat channel into the web SDK / cockpit, or keep the chat app separate. Recommend separate for the demo.
4. Voice input in v1. Recommend no.
