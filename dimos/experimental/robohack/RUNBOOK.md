# Runbook — laptop setup & on-site steps

> **Where the hack runs:** your **laptop, on the venue network.** The Go2 is reachable only over local
> WebRTC/ROS2 — the VPS can't see it. Use the VPS only for prep + (optionally) hosting the public **payment
> webhook** (Alipay `PAYMENT_NOTIFY_URL` / a hosted x402 endpoint need an internet-reachable URL).
>
> **Flyer update (this version):** the "swarm track / pool of 6 dogs" line is **gone** — looks like **1 dog
> per team** now. Treat **Direction B (swarm) as contingent on confirming multi-dog access at the lottery.**
> Default to the **Patrol Dog** build. Stack also now says **Linux** (was "Mac & Linux").

---

## ⚠️ Laptop pre-check (do this FIRST — it changes everything below)
- [ ] **OS = Linux?** Stack now says Linux. If Linux → native install. If **Mac/Windows → use DimOS's Docker
      path** (Docker is in the stack) with host networking so the container still reaches the dog on the LAN.
- [ ] **Python 3.10+**, **Docker**, **`uv`** installed
- [ ] **~20 GB free disk** (models + Docker images)
- [ ] **GPU?** A CUDA GPU speeds up the VLM/perception (moondream/Qwen). CPU works but is slow — if no GPU,
      plan to use a **hosted VLM** (Qwen/OpenAI/Claude) for scene reasoning instead of local models.

---

## Phase 0 — Tonight, on your laptop (highest-leverage prep)

Ordered by payoff. The single biggest win is #3: arrive already fluent.

1. [ ] Install: `uv pip install 'dimos[base,unitree]'` (cleanest per research). Clone both repos:
       `gh repo clone dimensionalOS/dimos` and `gh repo clone dimensionalOS/dimos-unitree`.
2. [ ] LLM key **with credits** in `default.env` (DimOS defaults to `gpt-4o`; Claude works too).
3. [ ] **Run the agentic stack with NO hardware** via `--simulation` (MuJoCo) or `--replay`. Send it a command
       (`dimos agent-send "…"`) and watch it call skills. *This is the whole game — learn the loop tonight.*
4. [ ] Install the **Rerun viewer**; practice the timeline scrub + click-to-teleop.
5. [ ] Read the two files that matter: `dimos/agents/annotation.py` (the **`@skill`** decorator) and
       `…/go2/blueprints/agentic/_common_agentic.py` (where skills register). Skim the CLI:
       `dimos run · agent-send · mcp call · status · log -f`.
6. [ ] **Pre-write a Patrol Dog `@skill` scaffold** (patrol / describe_scene / compare_to_baseline /
       log_incident / narrate / recover) so Day 1 is fill-in-the-blanks.
7. [ ] Verify **MCP external control**: run with `--mcp-host 0.0.0.0` and call a skill from an external MCP
       client (Claude Code/Slack). Confirms the "control from your phone" beat is real before you bank on it.
8. [ ] **Resume + 100-word pitch ready** (it's a hiring funnel — attach the resume to the application).
9. [ ] **(γ payments, off the public floor)** Fund **2 Base Sepolia burner wallets** (testnet ETH + USDC from
       faucets — they rate-limit, do it tonight). Note your ERC-8004 identity/reputation setup. Keep keys in
       env only; never a mainnet key on the venue laptop.
10. [ ] **(γ payments)** Stand up the FastAPI payment gateway skeleton on the **VPS** (public URL for Alipay
        `PAYMENT_NOTIFY_URL` / x402). Leave the robot-calling part stubbed until on-site.

## Phase 1 — At the office / check-in (Tue 16:00, first ~60 min)

Resolve the open variables immediately:
1. [ ] **Dog lottery → how many dogs?** This gates Direction B. If only 1 → Patrol Dog, full stop.
2. [ ] **Ask mentors, at check-in:**
       - Alipay AI Pay **sandbox merchant creds**? (gates the live "pay-the-dog" beat)
       - Is a **Qwen / RynnBrain** endpoint available on-site to use (and name-check)?
       - **Who judges the prizes** — Dimensional engineers vs. Ant/external? (calibrates how far to push the
         private x402/8004 flex)
3. [ ] **Hardware sanity check (do before anything fancy):** get on venue WiFi, get the dog's connection
       details, run **teleop**, and confirm camera + LiDAR show up in **Rerun**. If you can't drive the dog and
       see its sensors, nothing else matters — fix this first.
4. [ ] **17:00 DimOS workshop** — ask the exact questions: `@skill` registration gotchas, sim-to-real diffs,
       MCP external control, command serialization.

## Phase 2 — Hacking (Tue 17:30 →)

Follow the sequencing in **[research/00-synthesis.md](./research/00-synthesis.md)**. The non-negotiables
(from the winning-playbook research):
- **H0–4:** lock the ONE hero behavior; put all robot commands behind a **locked/serialized service** (the
  SDK isn't concurrency-safe).
- Build **sim-first**, then move to the real dog.
- **H30: freeze scope.** Harden only after that.
- **Film the hero behavior the first time it works reliably** — that clip is your live-demo insurance.
- Good external **mic + text fallback** for any voice (ambient noise kills voice demos).
- **Stream the agent's reasoning/skill-calls to a screen** — it's the WOW factor *and* proves you directed the AI.
- Cut the **bilingual vertical 90s video**: robot moving by 0:10, one clean cause→effect shot.

## What to physically bring
- Laptop + charger + USB-C; **external mic** (voice demos die on ambient noise); phone for **vertical video**;
  ethernet adapter (just in case); a small **prop** for the demo (a "package" to deliver, a marker, etc.);
  spare robot battery if offered. Power strip / multi-charger.

## The 3 things that decide your build (resolve at check-in)
1. **# of dogs** → Patrol Dog (1) vs. swarm swing (≥2).
2. **Alipay sandbox creds?** → real "pay-the-dog" beat vs. mock-Alipay fallback.
3. **Who judges?** → how hard to lean the private x402/ERC-8004 flex.
