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

- **Pay-the-robot / vending dog** 💰 — the Go2 delivers a snack/drink, the agent calls the **Alipay MCP**
  to charge for it, confirms payment landed, then hands it over. "Pay the robot dog in Alipay" is a killer
  90s demo *at an Alibaba/Ant venue specifically*. See the Alipay-payments multiplier below.

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

### Cross-cutting multiplier: Alipay payments via MCP 💰
DimOS agents already speak MCP, and **Alipay ships an official MCP server** — so you wire *both* into the same
agent: one MCP connection drives the Go2's Skills, the other lets the agent take real payments. The robot can
now **reason, move, AND transact.** Wildly on-brand at an Alibaba/Ant campus.

- **Repo:** [`alipay/global-alipayplus-mcp`](https://github.com/alipay/global-alipayplus-mcp) ·
  run with `uvx ant-intl-alipayplus-mcp` (Python 3.11+). There's also a **mainland-China** Alipay payment MCP
  (Ant's domestic "AI Pay") — likely the better fit for Shanghai; ask mentors which to use.
- **Tools exposed:** `create_payment`, `query_payment`, `cancel_payment`, `create_refund`,
  `customs_declare`, `query_customs_declare`.
- **Config (env):** `CLIENT_ID`, `MERCHANT_PRIVATE_KEY` (RSA), `ALIPAY_PUBLIC_KEY`, `PAYMENT_NOTIFY_URL`.
- **Demo loop:** agent calls `create_payment` → robot shows the QR / triggers AI Pay → `query_payment` polls
  until paid → robot completes the physical action (hands over item / unlocks / proceeds).
- ⚠️ **The one real blocker = credentials.** You need merchant `CLIENT_ID` + RSA keys, and there's no obvious
  public sandbox. **Action: at check-in, ask the Dimensional/Ant mentors for sandbox merchant creds** — at
  this venue they may literally hand them to you. Have a fallback (mock the payment tool) so the demo survives
  if creds don't materialize.

### Cross-cutting multiplier: agent-economy payments (x402 / MPP / ERC-8004) 🤖💸
A different philosophy from Alipay: instead of "robot charges a human in fiat," the Go2 becomes an **economic
agent** — it autonomously pays for services it consumes, or sells its own services, with an on-chain identity
+ reputation. This is *the* frontier-agent narrative and a strong hire signal for an agent-native company.

| Rail | What it is | Why it matters here | 48h reality |
|---|---|---|---|
| **x402** | Coinbase's HTTP-402 + on-chain **stablecoin** (USDC); EVM + Solana; **Python SDK**; ~2s settle. Now an x402 Foundation (Linux Foundation) standard. | Robot pays/charges per call. **Permissionless on testnet** (Base Sepolia) → *no merchant creds needed*, unlike Alipay. | ✅ Lowest-friction crypto rail; testnet USDC is free. Best starting point. |
| **MPP** | Stripe + Tempo "Machine Payments Protocol". HTTP-402 abstraction, **method-agnostic** (stablecoin / card / Lightning), **back-compatible with x402**. "Sessions" = pre-authorize a budget, stream micropayments. | One endpoint, many rails; Stripe **test mode**. Sessions fit a robot doing many tiny paid actions. | ✅ Doable via Stripe test mode; newer, smaller ecosystem. |
| **ERC-8004** | Ethereum **Trustless Agents** standard. Three on-chain registries: **Identity** (ERC-721), **Reputation**, **Validation**. Live on mainnet Jan 2026. | Not payments — the **trust layer**. Robot gets an on-chain identity + a reputation it earns by completing tasks. Pairs with x402/MPP. | ✅ **In our wheelhouse — prior 8004 experience.** Lean in: this is a differentiator, not a risk. |

**Demo concepts**
- **Pay-per-task robot (crypto twin of the vending dog)** — robot's delivery/patrol service sits behind an
  x402-paywalled endpoint; a user/agent pays testnet USDC → robot executes. Permissionless, no creds → *more
  reliable to ship in 48h than the Alipay version.*
- **Self-funding robot** — the agent pays per-call for a paid perception/VLM/API service via x402, or streams
  micropayments through an MPP session as it works. Shows genuine economic autonomy.
- **Trustless swarm economy** (ties to Direction B) — each Go2 has an ERC-8004 identity + reputation; robots
  pay each other for subtasks and rate the result. Most ambitious; best "wow + frontier" combo — **and ERC-8004
  is already familiar to us, so this is the one to swing for.**

**Scope discipline:** ship **one payment rail** end-to-end on **testnet** first (x402 is the easiest). ERC-8004
identity/reputation is *not* the long pole for us — prior experience makes it a head start, so wire it in early
as the spine of the demo. Don't try to do Alipay *and* crypto in 48h — pick the narrative (local-fiat vs.
frontier-agent-economy) and commit.

### Pick-a-payment cheat (if you do payments at all)
- **Want the venue/judge ecosystem pop + real consumer UX** → **Alipay MCP** (blocker: merchant creds — ask mentors).
- **Want zero-credential, ship-tonight, frontier story** → **x402 on Base Sepolia testnet** (Python SDK).
- **Want method-agnostic + budgeted streaming** → **MPP** (Stripe test mode).
- **Want a trust/identity layer on top** → **ERC-8004** (already in our wheelhouse — use it as the spine, not a stretch).

### TypeScript role (for a TS-first builder)
The DimOS **core is Python and unavoidable**: `@skill`s, the LangGraph agent, MCP server, nav, perception,
memory are all Python. **Don't rebuild DimOS in TS** (Tier-3 trap). But TS owns the high-value layers around it:
- **Demo UI / dashboard** — a TS/React web app over the LCM↔WebSocket bridge (`@dimos/lcm`, `@dimos/msgs`) +
  MCP, streaming the agent's reasoning + camera + map + incidents. This *is* the "reasoning on screen" WOW the
  playbook calls for, and it's TS-native.
- **Payments layer (γ)** — the Alipay/x402 gateway + public webhook (lives on the VPS) in Node/TS; ERC-8004 /
  on-chain glue via viem. All TS turf.
- **MCP control surface** — a TS app/bot driving the robot over MCP (`:9990`) = "control from your phone".

**Split:** keep Python to a thin skills layer (use `learn/patrol_dog_skills.py` as the template); own UI +
payments + MCP control in TS. Solo, you still write a handful of Python `@skill`s; with a teammate, one does
Python skills, you do everything TS.

## Judging notes
- Build to a **90-second story**; rehearse it.
- It's a **3-day interview** — clean Skill code, graceful failure handling, and good Rerun usage are hiring signal.
- Use the **dog lottery** result to make the A-vs-B call immediately.
