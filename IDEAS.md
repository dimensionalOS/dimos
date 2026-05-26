# Build ideas & requirements

Two live directions, fully specced, plus a backlog of fallbacks. Pick A or B after the Day-1 hardware check;
keep the other as a fallback narrative.

---

## Direction A — Patrol + Reasoning agent (Agents track)

**Pitch.** The Go2 becomes an autonomous patrol that *thinks out loud*. It walks a set of waypoints; at each
stop a VLM describes the scene and compares it to a learned "normal" baseline in spatial memory. When something
is off — door open, unfamiliar person, misplaced object — the agent reasons about the anomaly, narrates it,
captures the frame, and logs an incident with location + timestamp. On getting stuck/losing localization, the
LLM diagnoses and replans.

**Why this can win.** Demos reliably (no luck required), shows DimOS's agent core + spatial memory, and is
built directly to the track's stated rubric: *plan, execute, fail, recover, keep going.*

### Requirements / build plan
- **Skills to write**
  - `patrol(route)` — drive through saved waypoints (wraps `Navigate`/waypoint nav)
  - `describe_scene()` — grab current frame → VLM → short structured description
  - `compare_to_baseline(scene)` — query spatial memory for the "normal" at this pose; return diffs
  - `log_incident(desc, frame, pose)` — persist to memory + a JSON/markdown report
  - `narrate(text)` — TTS out loud
  - `recover()` — read stuck/localization state, replan or back out
- **Agent**: LangGraph agent (Claude) with the above as tools + a system prompt encoding the patrol policy
- **Perception**: existing YOLO + VLM scene captioning; person/object tracking streams
- **Memory**: ChromaDB spatial RAG — seed a baseline pass first, then patrol against it
- **Demo asset**: Rerun 3D replay + overlaid narration = the 90s video

### Risks & mitigations
- *Relocalization drift* → keep the route short, use distinctive landmarks, pre-map the demo area Day 2 AM
- *VLM latency* → describe only at stops, cache, keep prompts tiny
- *Flaky anomaly detection* → stage 2–3 reliable, obvious anomalies for the demo

### 48h timeline
- **D1 eve**: env up on the assigned dog, teleop sanity, save 4–6 waypoints, baseline pass
- **D2 AM**: Skills + agent loop working end-to-end in the demo area
- **D2 PM**: anomaly reasoning + incident log + TTS; Rerun capture dialed in
- **D3 AM**: harden recover(), rehearse, shoot the 90s video before freeze (17:30)

---

## Direction B — Swarm Choreography (Open / Creative track)

**Pitch.** A pack of Go2s as one choreographed performer. **One** DimOS Blueprint deploys the same behavior
file across every dog; a conductor agent issues natural-language cues ("form a line", "wave down the row",
"scatter and regroup") that each robot interprets relative to its own pose + neighbors. Formations,
synchronized tricks, and call-and-response emerge from shared Skills, not hand-scripted timelines.

**Why this can win.** Highest audience wow at the Maker Faire → best Grand-Prize swing, and it's the most
**on-brand** idea: it directly demonstrates that DimOS's "one file → many robots" (`autoconnect`) actually
scales. Adding a dog is a config change.

### Requirements / build plan
- **Confirm in the lottery you can get ≥2 dogs** (swarm track shares a pool of 6) — this gates the whole idea
- **Shared Skills (same file, all dogs)**
  - `take_position(formation, index, n)` — compute target pose from formation + my slot
  - `sync_move(vector)` / `sync_trick(name)` — synchronized motion / WebRTC trick (`FrontFlip`, `Pounce`…)
  - `hold_offset(neighbor, dx, dy)` — maintain spacing (visual or pose-based)
- **Conductor**: one agent that broadcasts cues; per-dog agent maps cue → its Skill calls
- **Coordination**: shared clock / countdown for sync; start simple (line, circle, wave) before anything fancy
- **Demo asset**: wide shot of the pack + Rerun multi-robot scene

### Risks & mitigations
- *Lottery gives 1 dog* → fall back to Direction A (keep both scaffolded)
- *Coordination/timing is hard* → choreograph to a countdown, not live feedback, for the demo
- *Localization between dogs* → use generous spacing + relative offsets; rehearse the exact routine
- *Testing needs all dogs at once* → book swarm-pool time early; design so each dog works solo too

### 48h timeline
- **D1 eve**: confirm dog count; get **one** dog running the behavior file solo
- **D2 AM**: same file on 2 dogs; basic line + circle + countdown sync
- **D2 PM**: add a wave + one synchronized trick; lock the routine
- **D3 AM**: full rehearsal, scale to whatever dogs are available, shoot video before freeze

---

## Ranked idea pool

Beyond the two committed directions, here's the broader pool — ranked by demo impact vs. 48h risk.
Mine these for variations on A/B or as fallbacks.

### Tier 1 — high demo impact, achievable in 48h
- **Robot concierge** — voice/chat → Go2 fetches a person by description ("find the guy in the red jacket,
  bring him to booth 4"). Open-vocab detection (Detic) + nav + planning agent. Hits perception + agents +
  a clear story. *Overlaps heavily with Direction A's perception/agent loop — could be the A demo scenario.*
- **Multi-robot swarm with role split** — 2–3 Go2s under one orchestrator agent: one scouts, one guards, one
  fetches. Visual chaos on stage = crowd reaction; DimOS supports it natively. *This is a richer framing of
  Direction B — swap pure choreography for role-based coordination if that demos better.*
- **MCP-everything** ⭐ — expose the Go2 over MCP so anyone controls it live from Claude Desktop / Slack /
  WhatsApp during the demo. "Anyone in the room commands the robot from their phone" is a judge-magnet, and
  it's exactly where DimOS is pushing → strong hire signal. **Treat this as a multiplier, not a standalone:**
  bolt it onto A or B for ~half a day and the demo gets dramatically more interactive.

### Tier 2 — technically interesting, more risk
- **Spatial memory + semantic search** — patrol, build a semantic map, answer "where did you last see my
  laptop?" Plays to DimOS's semantic memory module. (Close cousin of the memory-companion fallback.)
- **Teach-by-demonstration** — walk the robot a route once, it replays autonomously with obstacle avoidance.
  Good narrative, moderate nav risk.
- **Streaming sidekick** — agent narrates what it sees and reacts to chat commands (Twitch / 小红书).
  Cheesy but extremely shareable; pairs naturally with MCP-everything.

### Tier 3 — avoid unless you have a specialist
- Custom RL policies / new locomotion gaits — won't finish in 48h
- Fine manipulation — Go2 has no arm by default
- Heavy training runs — no time

### Quick fallbacks (if both committed directions stall)
- **"Where did I leave my X" memory companion** · **ArUco self-docking** (Autonomy) ·
  **Voice tour-guide** (Maker Faire crowd-pleaser) · **Gesture / Simon-says game** (low risk, high charm)

### Cross-cutting multiplier: MCP-everything
Whichever direction wins, exposing the robot's Skills over MCP and demoing live phone control is the single
highest-leverage add-on for both **audience reaction** and **hire signal**. Budget ~half a day for it once
the core behavior works.

## Judging notes
- Build to a **90-second story**; rehearse it.
- It's a **3-day interview** — clean Skill code, graceful failure handling, and good Rerun usage are hiring signal.
- Use the **dog lottery** result to make the A-vs-B call immediately.
