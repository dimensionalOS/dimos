# MUROBO — DePIN Robot-as-a-Service (DimOS hackathon)

**MUROBO** is an "Uber for robots": connect a wallet, pick a task (room scan, security patrol, perception walk, inspection…), and the nearest available robot is dispatched to do it. Autonomous **Unitree Go2** robots run agents built on **DimOS**, while a distributed set of providers supply the intelligence (VLM reasoning, CLIP analysis, alert detection, 3D reconstruction). When a job finishes, payment is split between the platform and the providers whose intelligence powered it. Agents carry verified on-chain identities (ERC-8004) and earn reputation across jobs.

Flagship demo: **hire a robot to scan your rooms and get back a 3D Gaussian-splat virtual tour.**

**▶ Demo video:** https://youtu.be/sqk0xOEcGHI

## Repos
- **robohack** — the web app (Next.js marketplace + server / gateway / api / db) and the **murobo** design assets + demo video: https://github.com/grmkris/robohack
- **gs-pot** — Gaussian-splat 3D reconstruction logic: https://github.com/ameer-clara/gs-pot

## DimOS side (this fork)
The robot agent is composed from DimOS Skills + Blueprints added in this fork:
- `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_gemini*.py` — all-Gemini agentic Go2 blueprints
- `dimos/agents/skills/` — `gemini_speak_skill.py`, `local_speak_skill.py`, `room_manager.py`, `map_uploader.py`, `map_saver_skill.py`, `take_picture_skill.py`, `agent_command_endpoint.py`
- `dimos/models/vl/gemini.py`, `dimos/stream/audio/tts/node_gemini.py`, `scripts/export_recording.py`

---
_Note: the sibling `security_demo/` is pre-existing upstream DimOS code (#1619), unrelated to this project._
