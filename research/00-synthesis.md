# Synthesis — master strategy from 5 research fronts

> Integrates `01`–`05`. Read this first; the others are the evidence base.
> Produced by 5 parallel Opus research agents, May 25 2026 (day before check-in).

---

## TL;DR

- **Hero build:** re-skin **Direction A** as a **commercial inspection / security / delivery agent** ("Patrol
  Dog / 巡逻汪") — the most credible *落地 (real-deployment)* story for a Go2, it mirrors Dimensional's own
  hotel/data-center/grid deployments, it's reliable on stage, and it's built almost entirely through DimOS's
  one high-leverage seam: the **`@skill` decorator**.
- **Venue multiplier:** one **Alipay "pay-the-dog"** beat (delivery hand-off → AI Pay charge → release item).
- **The fork (decide now):** the **crypto agent-economy angle (x402 / ERC-8004)** that we got excited about is,
  per the China research, a **legal + cultural liability for a live public demo in mainland China.** This is the
  single most important finding of the whole exercise. See "The central tension" below — it needs your call.
- **Direction B (swarm)** stays as the Grand-Prize / Maker-Faire swing **only if the lottery gives ≥2 dogs.**

---

## Where all five papers agree (build this regardless)

1. **The `@skill` decorator is the whole game** (paper 01). Adding an `@skill` method (typed params, mandatory
   docstring, `str` return) to a Module and registering it in `_common_agentic.py` makes it *both* an LLM tool
   *and* an external MCP tool instantly. ~80% of a winning build lives here. Don't rebuild the stack — extend it.
2. **Reliability > ambition; one hero behavior** (paper 03). "The team with the best demo wins, not the best
   code." Robots fail *visibly*; winners make one happy-path flawless and freeze scope at ~hour 30.
3. **Build with DimOS's grain = sponsor alignment** (paper 03). Judges are the hiring panel; reward projects
   that make their platform look inevitable. Show the agent's reasoning/skill-calls on screen — it's the WOW
   factor *and* proof you directed the AI.
4. **Stand on shoulders** (paper 02). `dimos-unitree` is the baseline; `go2_webrtc_connect`, `go2_ros2_sdk`,
   and the CMU `autonomy_stack_go2` are the strongest reusable pieces. WebRTC path (not low-level SDK) for a
   48h demo — it's what DimOS already uses and where the crowd-pleasing tricks live.
5. **落地 + cute + bilingual** (paper 05). Lead with the customer and the job, not the architecture. Make the
   dog cute and the demo one legible "wow" moment, shot vertical for RED/Douyin. Name-check Unitree/Qwen/Alipay.

## The convergent hero behavior

**"巡逻汪 / Patrol Dog" — an autonomous inspection & security agent.**
It patrols waypoints; at each stop a VLM (use **Qwen/RynnBrain** and *say so* — instant face) describes the
scene vs. a learned baseline in spatial memory; flags anomalies; narrates bilingually; logs incidents; and on
getting stuck, the LLM diagnoses + replans. This is Direction A, but framed as the commercial category Go2s
actually sell into (State Grid is spending ~$1B on exactly this). It is the strongest *落地* story available,
it's reliable, and it's pure `@skill` work.

**Build seam (paper 01):** new container of `@skill`s → `dimos/robot/unitree/go2/blueprints/agentic/_common_agentic.py`
→ system prompt → done. Demoable with zero hardware via `--replay` / `--simulation` (MuJoCo) while you wait for
the dog.

---

## The central tension — CRYPTO vs ALIPAY (needs your decision)

We have two payment narratives and they point in opposite directions:

| | **Alipay AI Pay** (paper 05 rec) | **x402 / ERC-8004** (paper 04 rec) |
|---|---|---|
| Legality in mainland CN | ✅ Legal, domestic, the home-team product | ❌ **PBoC banned crypto trading + stablecoins + NFTs/RWA (June 1 2025), AI-surveilled enforcement** |
| Cultural read at Ant campus | 🎯 On-brand home run | ⚠️ Reads as "doesn't understand the market" |
| Engineering friction | Needs merchant creds (no public sandbox) | ✅ Permissionless on Base Sepolia, ships tonight |
| Novelty | Expected | Frontier — *but OpenMind+Circle already did robot-pays-in-USDC (Feb 2026)* |
| Our team fit | Neutral | ✅ We have prior ERC-8004 experience |

**The hard constraint:** a live on-chain USDC transaction at a *public* event in mainland China is not just
culturally off — it's legally dubious. Paper 05's verdict is unambiguous: **Alipay for the live demo; take
crypto off the public stage.**

**But two nuances cut the other way:**
1. **Who actually judges?** The internal dry-run is to **Dimensional's own engineers** — an agent-native team
   (ex-Figure/DJI, backed by the HuggingFace CEO, half SF-based). *They* may genuinely love an x402/8004
   economic-agent demo. The "Chinese-VC / 落地" lens applies most to the **public Maker Faire** and any external
   judges — not necessarily to the people deciding the prizes.
2. **Our 8004 skill is a real asset** and the agent-economy story is a strong *hiring* signal for this company.

### Three ways to resolve it
- **(α) Alipay-primary (safest, paper 05's call):** Patrol/delivery + Alipay "pay-the-dog." Crypto off the
  table. Maximizes 落地 + cultural fit + legality. Lowest risk.
- **(β) Crypto-primary (highest hiring-signal-to-Dimensional, highest risk):** x402 + ERC-8004 economic agent.
  Frontier story, uses our 8004 depth — but legally/culturally exposed at a public mainland event.
- **(γ) Thread the needle (recommended):** **Live public demo = Alipay (or a clean mock).** Keep the
  **agent-economy + ERC-8004 architecture as a documented "global mode" in the repo**, and optionally show it
  *privately* to the Dimensional engineers (testnet, on a laptop, not the public floor). Payment layer built
  pluggable so the same delivery Skill settles via Alipay *or* x402. We get the 落地 win, legal safety, AND a
  way to flex 8004 to the people who hire — without putting an illegal-in-CN transaction on the public stage.

→ **This is the one decision that forks the build. Everything else is settled.**

---

## Recommended plan (assuming γ, adjust per your call)

**Primary:** 巡逻汪 / Patrol Dog (commercial inspection-security-delivery agent) on Direction A, via `@skill`s.
**Multiplier (venue):** Alipay "pay-the-dog" delivery beat.
**Architecture flex (repo + private):** pluggable payment layer; x402 + ERC-8004 "global agent-economy" mode.
**Swing (only if ≥2 dogs):** Direction B swarm choreography for the Maker Faire / Grand Prize.

### 48h sequencing (from paper 03 + 01)
- **Pre-event (tonight):** `uv pip install 'dimos[base,unitree]'`; clone dimos + dimos-unitree; run `--simulation`;
  load LLM key + credits; install Rerun; pre-write `@skill` templates; **at check-in, ask mentors for Alipay
  sandbox creds**; (if γ/β) fund two Base Sepolia burner wallets from faucets *now* (they rate-limit).
- **H0–4:** lock the ONE hero behavior; serialize robot commands behind a locked service (SDK isn't
  concurrency-safe, paper 03).
- **H4–24:** patrol + VLM-reasoning + narrate loop working end-to-end (sim first, then the real dog).
- **H24–30:** add the payment beat + (γ) the reputation/architecture flex; stream agent reasoning to a screen.
- **H30:** **freeze scope.** Harden only.
- **H30–48:** rehearse the live run 3–5×; **film the hero behavior the first time it works reliably** (live-demo
  insurance); cut the bilingual vertical 90s video — robot moving by 0:10, one clean cause→effect shot.

## De-risking (cross-paper)
- Serialize all robot commands; no training-on-the-day; pre-trained models only.
- Good mic + **text fallback** for any voice (ambient noise killed a real Go2 voice demo, paper 03).
- Pre-recorded backup video queued; rehearse the "let me show the clip" pivot.
- Charge a spare battery; test the actual demo floor (surface/lighting/wifi/noise).
- (γ/β) Local mock facilitator behind an env flag; execute the physical action on `verify=ok`, let settlement
  finalize in background; burner keys only, never a mainnet key on the venue laptop.
- Alipay: ship a **mock-Alipay fallback** so the narrative survives if creds don't materialize. If creds are
  impossible, **mock Alipay — do not pivot to crypto on the public floor.**

## Open questions to resolve at check-in
1. How many dogs does the lottery give us? (gates Direction B)
2. Can mentors provide **Alipay AI Pay sandbox merchant creds**?
3. Who exactly judges the prize — Dimensional engineers, Ant/Ali people, or external? (calibrates α/β/γ)
4. Is there a Qwen/RynnBrain endpoint available on-site to use (and name-check)?
