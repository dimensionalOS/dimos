# The Hackathon Winning Playbook

*Reverse-engineering what wins 48-hour robot/agent hackathons — tailored for the DIMENSIONAL/DimOS Go2 event (Shanghai, May 26–28 2026).*

## TL;DR

The team that wins this event will not have the most ambitious robot. It will have **one flawless, legible behavior** that visibly uses DimOS's agent-native architecture, a **90-second video that opens with the robot already moving**, and a **pre-recorded backup** so the live dry-run can never sink them. Below is the evidence behind each of those claims.

---

## 1. Patterns in winning projects: scope, polish, narrative, novelty vs. reliability

The single most repeated finding across postmortems is that **winners build a demo, not a product**. John Kim's "How to Win Hackathons" states it bluntly: *"You're not building a product and you're not even building a prototype — you're building a demo… The team with the best code almost never wins. The team with the best demo does."* ([Push To Prod](https://getpushtoprod.substack.com/p/how-to-win-hackathons)) The corollary is **make the happy path perfect and let everything else be rough.**

At hardware hackathons specifically, the reliability/novelty trade tilts hard toward reliability — because hardware *visibly* fails. The freecodecamp "How I Won" writeup shows the same instinct: when complex Excel-parsing blocked them, they **downgraded to a plain text input** and won anyway, because the judges cared about the demonstrated business impact, not the parser ([freecodecamp](https://www.freecodecamp.org/news/how-i-won-the-hackathon/)).

Narrative wins ties. The freecodecamp team coached a *non-technical domain expert* to deliver the pitch in business language ("100 hours saved annually"), using SPIN-selling framing — and that authentic problem narrative is what the senior judges responded to, not architecture slides.

Counter-intuitively, at the **LeRobot Worldwide Hackathon (June 2025, 3,000+ participants)**, the participant writeup notes that **hardware/mechanical innovations impressed judges *more* than AI implementations** — a modular "centipede" multi-arm rig, a rail system, custom grippers (suction cups, chopstick holders) all stood out, while many software/ML submissions failed on infrastructure ([Kamath](https://kamathrobotics.com/lerobot-worldwide-hackathon)). Lesson: a *physically tangible, novel-looking* element reads instantly on camera and to roboticist judges.

## 2. The 90-second demo as a format

Devpost's and Hackathon.com's demo-video guides converge on a tight structure ([Devpost](https://info.devpost.com/blog/6-tips-for-making-a-hackathon-demo-video), [Hackathon.com](https://tips.hackathon.com/article/creating-the-best-demo-video-for-a-hackathon-what-to-know)):

- **Lead with the elevator pitch + a working shot in the first ~10 seconds.** Judges watch many videos back-to-back; if the robot isn't doing something by second 10, you've lost them.
- **Show the project functioning with real input** — not slides describing it. "Show what it does as much as possible."
- **Don't pack in too much.** One clear capability beats five half-shown ones.
- **Technical quality matters**: good mic, no fuzzy voiceover, stable framing. Use tools you already know (OBS, CapCut, Canva) — don't learn an editor at hour 46.
- **Write your own script**; reserve 2–3 hours before deadline for record/edit/upload.

A 90-second cut for a quadruped should read roughly: **0:00–0:10** hook + robot already moving; **0:10–0:25** the problem/one-sentence concept; **0:25–1:10** the single hero behavior end-to-end (voice/text command → agent reasoning shown on screen → Go2 executes physically); **1:10–1:30** the "why this matters for agent-native robotics" payoff + team.

**Common mistakes that kill a hardware demo on camera:** filming a cluttered floor where you can't tell what the robot did; audio that drowns out the spoken command (the WSO2 Go2 team had to swap to a Jabra speakerphone + wireless mic because *ambient noise broke speech recognition* — "Choreo" was heard as "Korea") ([WSO2](https://medium.com/wso2-ai-blog/how-we-gave-life-to-an-ai-agent-with-the-unitree-go-2-robot-f9c7afec0a77)); and editing that hides the cause→effect link between the command and the motion.

## 3. How elite/technical judges actually score

ETHGlobal's published criteria (a useful proxy for elite agent-hackathon judging) are **Practicality** (is it complete/functional enough to use today?), **Usability/DX**, and **WOW factor** ([ETHGlobal](https://ethglobal.com/events/openagents/prizes)). Note that "working today" outranks ambition. ETHGlobal explicitly warns that **projects that are entirely AI-generated without meaningful human direction may be disqualified** — judges "need to see the full picture of how you directed the AI." For a vibecoding-friendly stack like DimOS, that means *show your reasoning and your skill code*, not just the output.

General judging frameworks (TAIKAI, Mettl) echo: judges *don't expect production-ready* in 36–48h, but **a working demo that proves the concept** plus "clean architecture and thoughtful problem-solving" signal a team that could ship beyond the event ([TAIKAI](https://taikai.network/en/blog/hackathon-judging)).

**The sponsor-alignment effect is the strongest single lever here.** Push To Prod's central thesis is that hackathons are *marketing events*: sponsors on the judging panel "want to hear their tools highlighted and see their products used effectively." The ASI/Fetch.ai track at ETHGlobal NYC rewarded projects that composed the sponsor's *own* frameworks (uAgent, MeTTa reasoning) into novel systems ([ASI](https://superintelligence.io/ethglobal-nyc-winners/)). **For this event, that means building visibly *with* DimOS's grain**: use `Agents()` calling Unitree WebRTC/ROS2 action primitives, the skills abstraction, MCP-based skill discovery, and RxPY reactive streams — i.e., demonstrate the thing DimOS exists to prove: *that an LLM agent can be a first-class controller of a physical robot* ([dimos GitHub](https://github.com/dimensionalOS/dimos), [dimos-unitree](https://github.com/dimensionalOS/dimos-unitree)). A project that makes DimOS look like the future of robotics is a project the DIMENSIONAL judges want to win.

## 4. Hardware-hackathon failure modes (the stuff that leaves you with nothing at hour 47)

The "demo-day curse" is documented and real: robots that worked all weekend spin in circles the moment they're in front of judges ([Robocraze](https://robocraze.com/blogs/post/why-my-robot-only-works-on-demo-day)). Concrete failure modes pulled from the Go2/LeRobot postmortems:

- **Cloud/training dependencies that fall over under load.** LeRobot teams lost their ML submissions when the HuggingFace hub crashed from hackathon traffic and overnight Colab training jobs died ([Kamath](https://kamathrobotics.com/lerobot-worldwide-hackathon)). *De-risk:* don't bet the demo on training a policy or on a flaky hosted service; prefer pre-trained models / deterministic skills.
- **Concurrency and SDK sharp edges.** The Unitree Python SDK was "not inherently concurrency safe"; the WSO2 team had to wrap it in a Flask service with request locking to stop simultaneous movement commands from colliding ([WSO2](https://medium.com/wso2-ai-blog/how-we-gave-life-to-an-ai-agent-with-the-unitree-go-2-robot-f9c7afec0a77)). *De-risk:* serialize robot commands early.
- **Audio/sensor environment.** Ambient noise wrecked voice control; required hardware mic upgrades. *De-risk:* if using voice, bring a good mic; have a text-command fallback.
- **Power and physical robustness.** STEM demo-day writeups cite power instability and mechanical fragility as the top causes of on-stage failure ([Tech Edu Byte](https://www.techedubyte.com/hardware-project-failed-demo-day-lessons-stem-students/)). *De-risk:* charged spare battery, tested floor surface, calm debugging routine.
- **Shared-robot scheduling.** A grand-prize *is* a Go2 — implying few physical robots on site. Expect to fight for hardware time. *De-risk:* develop against simulation/recorded streams and reserve hardware slots for integration + filming, not exploration.

## 5. The hiring angle

This event fast-tracks winners into hiring, so the *signals* matter as much as the result. From recruiting-focused writeups, what gets people hired at hackathons: **observable team dynamics, prototyping speed under pressure, and clear communication** — companies "witness their team dynamics and their ability to prototype ideas in a tight timeline" ([Resumly](https://www.resumly.ai/blog/how-to-leverage-hackathon-visibility)). Push To Prod's role model is a clean three-person split: a **PM/presenter** who owns sponsor conversations and the pitch, a **deep diver** who disappears into the hardest problem, and a **generalist** who does integration.

Concretely, to read as hireable: keep a tidy public repo with a real README and architecture diagram (ETHGlobal judges want to see *how you directed the AI*); narrate trade-offs out loud during the dry-run ("we serialized commands because the SDK isn't concurrency-safe"); and **build rapport with the DIMENSIONAL engineers all weekend** — Push To Prod calls sponsor rapport "probably the most underrated thing you can do." The people deciding the hiring fast-track are the same people standing at the sponsor table.

## 6. Time management for 48 hours

Synthesizing across sources, a winning 48h allocation:

- **Hours 0–4 — scope + sponsor recon.** Talk to DIMENSIONAL engineers, confirm hardware availability, pick the *one* hero behavior. Choose something with a tangible physical payoff (LeRobot lesson) that exercises DimOS's agent→skill→Go2 path (sponsor-alignment lesson).
- **Hours 4–24 — build the happy path** to a first end-to-end success. Get command→agent→motion working once on real hardware ASAP; everything after is hardening.
- **Hour ~30 — FREEZE SCOPE.** No new features after the freeze; only reliability, polish, and the pitch. Most teams that have "nothing at hour 47" kept adding instead of hardening.
- **Hours 30–40 — record the demo video and the backup.** Devpost: plan the video at the *start* and reserve dedicated time at the end. **Capture the hero behavior on video the *first* time it works reliably** — that footage is your insurance.
- **Hours 40–48 — rehearse the live dry-run** (3–5 full run-throughs), prep the "let me show you the video" fallback line, charge spare batteries, finalize repo + slide.

## 7. A concrete winning playbook for THIS event

1. **Pick ONE hero behavior**, not a platform. e.g. "Go2, find the red box and guard it" — voice/text command → DimOS agent reasons → navigates → reports. Demoable in 30 seconds.
2. **Build with DimOS's grain.** Use `Agents()` + Unitree WebRTC/ROS2 action primitives + the skills layer + MCP skill discovery. Make DimOS look inevitable.
3. **Make the agent's reasoning visible.** Stream the agent's thoughts/skill calls to a screen during the demo — it's the WOW factor *and* proves you directed the AI (anti-disqualification).
4. **Serialize robot commands** behind a locked service from hour one (avoid the concurrency crash).
5. **Avoid training-on-the-day.** No betting the demo on a freshly trained policy or a flaky cloud service. Use pre-trained models / deterministic skills.
6. **Freeze scope at ~hour 30.** Harden, don't expand.
7. **Film the hero behavior the first time it works** and keep the clip as your live-demo insurance.
8. **90-sec video: robot moving by 0:10**, one capability shown end-to-end, clean audio, your own script.
9. **Have a pre-recorded backup** queued for the live dry-run; rehearse the "here's the video" pivot.
10. **Charge a spare battery; test the actual demo floor** (surface, lighting, wifi/noise).
11. **One person owns the pitch**; coach them to lead with the problem and the agent-native vision, not the stack.
12. **Build sponsor rapport daily** — the DIMENSIONAL team are your judges *and* your hiring panel.
13. **Tidy repo + README + architecture diagram**, committed before the deadline (hireability signal).
14. **Add one tangible physical element** if cheap (a prop, a mount, a visible task object) — physical novelty reads on camera and to roboticist judges (LeRobot lesson).
15. **Rehearse the full live run 3–5×** and keep a calm debugging routine for the inevitable hiccup.

---

### Sources
- [Push To Prod — How to Win Hackathons](https://getpushtoprod.substack.com/p/how-to-win-hackathons)
- [freecodecamp — How I Won my First Hackathon](https://www.freecodecamp.org/news/how-i-won-the-hackathon/)
- [Lightning.ai — How a Team Won the LeRobot Hackathon](https://lightning.ai/blog/lerobot-hackathon-winners)
- [Aditya Kamath — LeRobot Worldwide Hackathon writeup](https://kamathrobotics.com/lerobot-worldwide-hackathon)
- [WSO2 — How We Gave Life to an AI Agent with the Unitree Go2](https://medium.com/wso2-ai-blog/how-we-gave-life-to-an-ai-agent-with-the-unitree-go-2-robot-f9c7afec0a77)
- [Devpost — 6 Tips for a Winning Hackathon Demo Video](https://info.devpost.com/blog/6-tips-for-making-a-hackathon-demo-video)
- [Hackathon.com — Creating the Best Demo Video](https://tips.hackathon.com/article/creating-the-best-demo-video-for-a-hackathon-what-to-know)
- [ETHGlobal — OpenAgents prizes/judging criteria](https://ethglobal.com/events/openagents/prizes)
- [ASI — ETHGlobal NYC Winners](https://superintelligence.io/ethglobal-nyc-winners/)
- [TAIKAI — Hackathon Judging: 6 Criteria](https://taikai.network/en/blog/hackathon-judging)
- [Robocraze — Why My Robot Only Works on Demo Day](https://robocraze.com/blogs/post/why-my-robot-only-works-on-demo-day)
- [Tech Edu Byte — When My Hardware Project Failed on Demo Day](https://www.techedubyte.com/hardware-project-failed-demo-day-lessons-stem-students/)
- [Resumly — Leverage Hackathons for Hiring Visibility](https://www.resumly.ai/blog/how-to-leverage-hackathon-visibility)
- [DimOS GitHub](https://github.com/dimensionalOS/dimos) · [dimos-unitree](https://github.com/dimensionalOS/dimos-unitree)
