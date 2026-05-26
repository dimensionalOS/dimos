# robohack — DIMENSIONAL (DimOS) Robot Hackathon

> **Event:** DIMENSIONAL Hackathon Shanghai · Alibaba T1 · **May 26–28, 2026** · 48h
> **Hardware:** Unitree Go2 (LiDAR + RGB + IMU) · **Stack:** [DimOS](https://github.com/dimensionalOS/dimos)
> **Tracks:** Autonomy & Navigation · Agents · Open/Creative
> **Prizes:** Grand Prize = a Go2 robot · Track winner = ¥3,500 · winners get a hiring fast-track

This repo is my prep + plan. Two directions are live; final call after Day-1 hardware reality check.

---

## 📚 Deep research (5 Opus agents → start with the synthesis)

| Paper | Angle |
|---|---|
| **[00 · Synthesis](./research/00-synthesis.md)** ⭐ | Master strategy — read this first |
| [01 · DimOS codebase map](./research/01-dimos-codebase-map.md) | Real API; the `@skill` seam; extension points |
| [02 · Go2 OSS landscape](./research/02-go2-ecosystem-landscape.md) | Reusable repos + proven "wow" demos |
| [03 · Hackathon winning playbook](./research/03-hackathon-winning-playbook.md) | What wins + 90s demo craft |
| [04 · Agent-economy payments](./research/04-agent-economy-payments.md) | x402 / MPP / ERC-8004 technical reality |
| [05 · Chinese judges & narrative](./research/05-chinese-judges-and-narrative.md) | 落地, Alipay vs crypto, virality |

**Headline from the research:** build **"巡逻汪 / Patrol Dog"** — Direction A re-skinned as a commercial
inspection/security/delivery agent (most credible *落地* story, pure `@skill` work, reliable on stage) + one
**Alipay "pay-the-dog"** beat. ⚠️ **The crypto angle (x402/ERC-8004) is a legal + cultural liability for a live
public demo in mainland China** — see the synthesis "central tension"; it's the one decision that forks the build.

---

## Key dates (GMT+8)

| When | What |
|---|---|
| Tue May 26 16:00 | Check-in · **dog lottery** · team setup |
| Tue May 26 17:30 | ⏱ Hacking begins |
| Wed May 27 12:00 | Mid-hack checkpoint |
| Thu May 28 17:30 | ⏱ **Code freeze** — submit GitHub + **90s demo video** |
| Thu May 28 19:00 | Internal dry-run to judges |
| Fri May 29 | Hardware Maker Faire (public demos) · 16:00 awards 🏆 |

## The two candidate builds

| | **A — Patrol + Reasoning** (Agents) | **B — Swarm Choreography** (Open) |
|---|---|---|
| One-liner | A Go2 that patrols and *thinks out loud* about anomalies | A pack of Go2s as one choreographed performer |
| Why it wins | Robust live demo, best hiring signal, hits the track's stated "plan→fail→recover" rubric | Highest audience wow → Grand-Prize swing; most on-brand for DimOS Blueprints |
| Main risk | Relocalization drift; VLM latency | Multi-robot coordination + getting ≥2 dogs in the lottery |
| DimOS pieces | Skills, LangGraph agent, VLM, spatial memory, waypoint nav, TTS | Blueprints + `autoconnect()`, shared Skills, WebRTC tricks |
| Demo shape | Walk → narrate → flag anomaly → log → recover | Cue → synchronized formation → trick → regroup |

→ Full detail in **[IDEAS.md](./IDEAS.md)**. Study reference in **[CHEATSHEET.md](./CHEATSHEET.md)**. Application blurbs in **[PITCH.md](./PITCH.md)**.

## Strategy in one paragraph

Judges are the Dimensional team; the deliverable is a **90-second video**; the event doubles as an interview.
So optimize for a **tight, robust, narratable behavior** — not maximal autonomy. The work lives in **Skills**
(RPC functions the LLM agent calls), not in rewriting the nav stack. Whichever direction we pick, the demo
must read in 90 seconds and degrade gracefully when the hardware misbehaves.

## Pre-event checklist (do before Tue 16:00)

- [ ] Laptop meets reqs: **Mac or Linux, Python 3.10+, Docker** (robot link is local-network WebRTC/ROS2)
- [ ] `uv pip install 'dimos[base,unitree,sim]'`; clone `dimos` + `dimos-unitree`; run the **sim** once
- [ ] LLM API key loaded **with credits** (Claude for tool-calling)
- [ ] **Rerun viewer** installed (debugger *and* demo-video source)
- [ ] Skim CLI: `dimos run`, `agent-send`, `mcp call`, `status`, `log -f`
- [ ] Read the Skill API; have a fill-in custom-Skill template ready
- [ ] Pitch blurb + resume submitted in the application
