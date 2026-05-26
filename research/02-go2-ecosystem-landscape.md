# Go2 Open-Source Ecosystem Landscape

**Research brief for the DIMENSIONAL/DimOS 48-hour hackathon (Shanghai, May 26–28 2026).**
Platform: Unitree Go2 quadruped + DimOS (Python, agent-native, ROS2-based). Tracks: Autonomy & Navigation, Agents, Open/Creative.

Goal: stand on shoulders. This maps what's already been built on the Go2 (and Go1/B2/Spot) so the team can reuse code and lift proven "wow" demos. Star counts and licenses captured 2026-05-26 via `gh search`/repo views; verify before depending on anything.

---

## 1. The control layer: how you actually move the dog

There are **two fundamental control paths**, and which one you pick shapes the whole build:

- **WebRTC path** — talks the same protocol as the Unitree mobile app. **No jailbreak, no firmware mod, works over Wi-Fi.** Gives you high-level "sport" commands (stand, walk, and crucially the *tricks*: FrontFlip, Pounce, Dance, Hello…), plus video/audio/lidar streams. This is the fastest path to a demo and the one DimOS uses.
- **DDS/SDK path** (CycloneDDS over Ethernet) — the official `unitree_sdk2`. Lower-level joint control, deterministic, required for custom RL locomotion policies. Heavier setup. Note the unofficial ROS2 SDK warns joint-state feedback lags at ~1 Hz over WebRTC — use DDS if you need tight closed-loop control.

### Official Unitree SDKs

| Repo | Stars | License | What it gives you |
|---|---|---|---|
| [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | 1120 | BSD-3 | C++ SDK v2, the canonical low-level control (DDS). `sport_client`, `motion_switcher`. |
| [unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | 699 | BSD-3 | Python bindings of the above. **Use this if you go DDS + Python.** |
| [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | 708 | BSD-3 | Official ROS2 message/bridge layer. |
| [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) | 1391 | BSD-3 | URDF/meshes + Gazebo description (ROS1-era but URDFs reused everywhere). |
| [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) | 988 | BSD-3 | Official MuJoCo sim — great for offline dev when you don't have the dog. |
| [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide) | 602 | BSD-3 | Reference controller / teaching codebase. |

### WebRTC drivers (the trick API — highest demo-leverage)

| Repo | Stars | License | Notes |
|---|---|---|---|
| [legion1581/go2_webrtc_connect](https://github.com/legion1581/go2_webrtc_connect) | 327 | MIT | **The de-facto community WebRTC driver.** Python, no jailbreak. Supports Go2 firmware 1.0.19→1.1.15+ and G1. Connection modes: LocalAP (dog's own Wi-Fi), LocalSTA (same LAN by IP/serial), Remote (Unitree TURN + account). Sport commands + video/audio/lidar subscriptions. Handles AES-128 per-device keys on new firmware. Actively maintained. |
| [legion1581/unitree_webrtc_connect](https://github.com/legion1581/unitree_webrtc_connect) | 327 | MIT | Newer Go2+G1 unified driver from same author, updated 2026-05-26. |
| [tfoldi/go2-webrtc](https://github.com/tfoldi/go2-webrtc) | 181 | BSD-2 | The original reverse-engineered WebRTC API (JS + Python). **Best documentation of the trick command names**: Hello, Stretch, FrontFlip, BackFlip, Pounce, Dance1, Dance2, FrontJump, Handstand, MoonWalk, FingerHeart. |
| [legion1581/go2_python_sdk](https://github.com/legion1581/go2_python_sdk) | 61 | BSD-2 | DDS-based unofficial Python SDK alternative. |
| [phospho-app/go2_webrtc_connect](https://github.com/phospho-app/go2_webrtc_connect) | 9 | — | Maintained fork by phospho (LeRobot folks). |

> **Trick API cheat-sheet source:** [The RoboVerse wiki — Go2 app console commands](https://wiki.theroboverse.com/en/unitree-go2-app-console-commands) documents the numeric API IDs for every sport action. legion1581 runs TheRoboVerse community; treat it as the reference.

**Verdict for 48h:** if your demo is "agent triggers cool physical behaviors," `legion1581/go2_webrtc_connect` + `tfoldi`'s command map is the single highest-leverage combo. Tricks are free wow-factor and already wired.

---

## 2. ROS2 / autonomy / SLAM stacks

| Repo | Stars | License | What it gives you |
|---|---|---|---|
| [abizovnuralem/go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) | 929 | BSD-2 | **The most popular unofficial ROS2 SDK.** WebRTC *and* CycloneDDS paths, lidar point clouds, front camera, laserscan conversion, slam_toolbox, Nav2, Foxglove + RViz2, joystick teleop, COCO object detection, multi-robot. Docker. Ubuntu 22.04 / Humble. Launch file starts 11+ services. Caveat: 1 Hz joint feedback over WebRTC. **Best single starting point for a nav/perception build.** |
| [jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2) | 473 | (unstated) | **Full CMU autonomy stack** (Ji Zhang et al., the LOAM/CMU-exploration lineage). Goal-point nav while mapping, terrain traversability, collision avoidance, route planner with visibility graphs, waypoint following. Uses only Go2's built-in L1 lidar + IMU. Unity sim included. Needs IMU calibration + Ethernet/DDS setup; not fully plug-and-play but battle-tested. |
| [Zhefan-Xu/isaac-go2-ros2](https://github.com/Zhefan-Xu/isaac-go2-ros2) | 532 | — | Go2 sim platform on NVIDIA Isaac + ROS2 for nav / decision-making / autonomy testing. |
| [Unitree-Go2-Robot/go2_robot](https://github.com/Unitree-Go2-Robot/go2_robot) | 314 | Apache-2.0 | Clean, modular community ROS2 org: `go2_driver`, `go2_slam`, `go2_navigation`, `go2_description`, `go2_rviz` as separate packages. Good if you want a tidy, à-la-carte ROS2 base. |
| [h-naderi/unitree-go2-slam-nav2](https://github.com/h-naderi/unitree-go2-slam-nav2) | 161 | — | Focused SLAM + Nav2 integration. |
| [andy-zhuo-02/go2_ros2_toolbox](https://github.com/andy-zhuo-02/go2_ros2_toolbox) | 151 | MIT | SLAM + navigation toolbox. |
| [khaledgabr77/unitree_go2_ros2](https://github.com/khaledgabr77/unitree_go2_ros2) | 120 | — | Full ROS2 Jazzy integration via CHAMP controller. |
| [machines-in-motion/Go2Py](https://github.com/machines-in-motion/Go2Py) | 71 | MIT | Clean Python interface + sim env. Good for scripted control without full ROS2. |
| [unitreerobotics/point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar) | 493 | GPL-2.0 | Official Point-LIO LiDAR-inertial odometry for Unitree lidar — drop-in localization. (GPL — license-check before mixing.) |

**Verdict:** for Autonomy & Navigation, start from `abizovnuralem/go2_ros2_sdk` (perception + nav already wired, WebRTC-friendly) or `autonomy_stack_go2` (stronger planner, more setup). For localization add `point_lio_unilidar`.

---

## 3. Perception building blocks

Perception is mostly **reused from generic CV repos** dropped onto Go2 camera/lidar topics rather than Go2-specific projects:

- `go2_ros2_sdk` ships **COCO object detection** (YOLO-class) with bounding boxes out of the box — easiest reusable detector.
- [grasp-lyrl/unitree_go2w_agent_sdk](https://github.com/grasp-lyrl/unitree_go2w_agent_sdk) (44★, Go2**W**) — unified AI-agent SDK with **YOLOv8/v9** detection, perception+planning+control behind one API. Aimed exactly at embodied-AI assistants; closest to "agent-friendly perception" off the shelf.
- For open-vocabulary / pointing-at-objects demos, bring your own **GroundingDINO + SAM** or **YOLO-World** and subscribe to the camera topic — no Go2-specific glue exists, but it's a 1-file ROS2 node.
- Lidar → costmap/laserscan conversion is handled in `go2_ros2_sdk` and `FishPlusDragon/unitree-go2-slam-toolbox` (55★: PointCloud2→Laserscan, slam-toolbox).
- **Person-following** is not a strong standalone repo for Go2 — easiest path is YOLO person detection → bbox-center P-controller → WebRTC velocity command. Build it; don't search for it.

---

## 4. LLM / VLM / agent integration (the "Agents" track core)

This is where DimOS itself lives, so know the alternatives you can borrow from:

| Repo | Stars | License | What it does |
|---|---|---|---|
| [dimensionalOS/dimos-unitree](https://github.com/dimensionalOS/dimos-unitree) | — (org repo) | — | **The hackathon's own stack.** Complete Go2 vertical slice: WebRTC **and** ROS2 hardware paths; A* global + local planning; LangGraph-based agents; **OpenAI Agent, Claude Agent, Planning Agent, HuggingFace Local Agent**; MCP support; skills architecture (`AbstractRobotSkill`, `SkillLibrary`, `MyUnitreeSkills`); spatial perception + object tracking + spatial memory. Skills = RPC functions callable by the LLM. **This is your baseline — extend it, don't reinvent.** (Note: main repo was 404 on direct fetch — clone via the org or PyPI `dimos`.) |
| [lpigeon/unitree-go2-mcp-server](https://github.com/lpigeon/unitree-go2-mcp-server) | 78 | Apache-2.0 | **MCP server** that turns natural-language commands into ROS2 instructions for Go2. Plug a Claude/GPT client straight in. Cleanest "LLM → robot via MCP" reference. |
| [wso2-incubator/unitree-go2-realtime-agent](https://github.com/wso2-incubator/unitree-go2-realtime-agent) | 16 | Apache-2.0 | **OpenAI Realtime API voice agent** w/ push-to-talk TUI, runs on the Go2's onboard Jetson. Personas, modular tools. Great template for "talk to the dog." See [WSO2 write-up](https://medium.com/wso2-ai-blog/how-we-gave-life-to-an-ai-agent-with-the-unitree-go-2-robot-f9c7afec0a77). |
| [anabeldilab/LLM-WebRTC-Unitree-Go2](https://github.com/anabeldilab/LLM-WebRTC-Unitree-Go2) | 1 | Apache-2.0 | Llama-3 semantic planner emitting **safe structured JSON commands** over legion1581's WebRTC middleware. Tiny but a clean planner pattern. |
| [grasp-lyrl/unitree_go2w_agent_sdk](https://github.com/grasp-lyrl/unitree_go2w_agent_sdk) | 44 | — | Single-API perception+planning+control built for agent control (Go2W). |
| [kimsooyoung/a1_chatgpt_demo](https://github.com/kimsooyoung/a1_chatgpt_demo) | — | — | "ChatGPT for Robotics" on Unitree A1 — the original Microsoft-style pattern, portable to Go2. |
| [vw VectorRobotics/vector-os-nano](https://github.com/VectorRobotics/vector-os-nano) | 142 | Apache-2.0 | CMU cross-embodiment "robot OS" w/ NL control + sim-to-real; covers Go2. A DimOS-adjacent comparison point. |

**Reusable voice/TTS-STT plumbing:** the WSO2 agent gives you OpenAI Realtime (STT+LLM+TTS in one stream). For local/cheaper: Whisper (STT) + Piper/Coqui (TTS) wired to the same tool layer. None of this is Go2-specific glue — borrow patterns from the WSO2 repo.

---

## 5. Simulation, RL & locomotion (if you want custom motion / a sim-only demo)

| Repo | Stars | License | Notes |
|---|---|---|---|
| [unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) | 3288 | BSD-3 | Official RL training (IsaacGym) — most-starred Unitree repo. |
| [unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab) | 1043 | Apache-2.0 | RL on IsaacLab. |
| [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways) | 1373 | (other) | Famous MIT sim-to-real for **Go1** — the gold-standard agile-gait policy. |
| [Teddy-Liao/walk-these-ways-go2](https://github.com/Teddy-Liao/walk-these-ways-go2) | 591 | MIT | **WTW ported to Go2** — deploy proven agile locomotion directly. |
| [abizovnuralem/go2_omniverse](https://github.com/abizovnuralem/go2_omniverse) | 1010 | BSD-2 | Go2/G1 in NVIDIA Isaac Lab/Sim/Gym — photoreal sim. |
| [zeonsunlightyu/LocomotionWithNP3O](https://github.com/zeonsunlightyu/LocomotionWithNP3O) | 514 | — | N-P3O locomotion policy, IsaacGym-trained. |
| [elijah-waichong-chan/go2-convex-mpc](https://github.com/elijah-waichong-chan/go2-convex-mpc) | 104 | MIT | Convex-MPC locomotion in MuJoCo — readable controller. |
| [legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control) | 520 | Apache-2.0 | ROS2-Control sim2real for quadrupeds incl. Go2. |

**Reality check:** training a new RL policy in 48h is a trap. If you want custom gaits, *deploy* `walk-these-ways-go2` (already trained). The built-in sport-mode tricks (Section 1) cover 95% of demo needs with zero ML.

---

## 6. Comparable platforms (fallback ideas / portable code)

- [rai-opensource/spot_ros2](https://github.com/rai-opensource/spot_ros2) (282★) — Boston Dynamics Spot ROS2 driver; CHAMP-controller pattern is reusable.
- CHAMP quadruped controller is the common abstraction across Spot/Go2 community sims (`spot_gazebo_ros2`, `khaledgabr77/unitree_go2_ros2`).
- [MAVProxyUser/YushuTechUnitreeGo1](https://github.com/MAVProxyUser/YushuTechUnitreeGo1) (456★) — Go1 dev notes, useful for older-firmware tricks.

---

## 7. Viral / creative demos worth stealing

Sources: [Unitree UniStore launch](https://tech.news.am/eng/news/7169/), [Go2 review](https://blog.robozaps.com/b/unitree-go2-review), [SCMP on G1 frontflips](https://www.scmp.com/tech/tech-trends/article/3337320/).

- **Tricks-on-command** — Go2 ships handstand, bipedal stand, leaps, two dances, sit/stretch/shake, and *upside-down self-righting* ("Advanced AI Mode"). Unitree's **UniStore** now sells dance/martial-arts/animation skill packs. Wrapping these behind an LLM agent ("do a backflip when I say I'm happy") is the cheapest viral demo.
- **G1 synchronized frontflips/dance** went mega-viral (Musk-endorsed). The quadruped analogue: **choreographed multi-action routine triggered by voice/agent**, filmed cleanly.
- **Conversational companion** — WSO2's voice-agent-on-Jetson at a conference. Persona + voice + reactive tricks = shareable.
- **Weaponized/military clips** (Guardian, 2024) went viral for the wrong reasons — *avoid*; this is a hiring funnel with elite roboticists judging.

**What makes Go2 clips shareable:** a *physical reaction to something intelligible* — speech, a gesture, a recognized object, a person to follow. The "wow" is the loop closing in the real world, not the model. Film it in one clean 90-second take; tricks + a clear trigger read instantly on camera.

---

## 8. Ranked shortlists by build direction

**If building "Agents" (talk to / command the dog) — recommended, plays to DimOS:**
1. `dimensionalOS/dimos-unitree` — extend the provided skills/agents (baseline).
2. `legion1581/go2_webrtc_connect` — trick/motion execution layer.
3. `lpigeon/unitree-go2-mcp-server` — if you want MCP-clean NL→action.
4. `wso2-incubator/unitree-go2-realtime-agent` — drop-in voice (OpenAI Realtime) loop.
5. `tfoldi/go2-webrtc` — trick command-ID reference.

**If building "Autonomy & Navigation":**
1. `abizovnuralem/go2_ros2_sdk` — perception + Nav2 + SLAM, WebRTC-friendly.
2. `jizhang-cmu/autonomy_stack_go2` — stronger planner / exploration.
3. `unitreerobotics/point_lio_unilidar` — localization.
4. `Unitree-Go2-Robot/go2_robot` — clean modular ROS2 base.
5. `grasp-lyrl/unitree_go2w_agent_sdk` — agent-friendly perception API.

**If building "Open/Creative":**
1. `legion1581/go2_webrtc_connect` + UniStore skill packs — instant tricks.
2. `tfoldi/go2-webrtc` — full trick vocabulary (FrontFlip, MoonWalk, FingerHeart…).
3. `wso2-incubator/unitree-go2-realtime-agent` — companion/persona.
4. `Teddy-Liao/walk-these-ways-go2` — custom expressive gaits (if time).

---

## 9. Flags / pitfalls

- **`dimos-unitree` main repo 404'd on direct fetch** — confirm clone path via the org / PyPI `dimos` early; don't discover this at hour 40.
- **WebRTC joint feedback ~1 Hz** — fine for tricks/teleop, *not* for closed-loop balance/manipulation; use DDS path for those.
- **New firmware (Go2 ≥1.1.15) uses AES-128 per-device keys** — `legion1581` handles it; older forks may not. Check the dog's firmware first.
- **GPL repos** (`point_lio_unilidar`, `lockecole111/go2py`) — license-incompatible with permissive distribution; isolate.
- Many `go2_webrtc_connect` "forks" are 0★ stale mirrors — use the canonical `legion1581` repos.
- RL-from-scratch in 48h = trap; deploy pre-trained `walk-these-ways-go2` instead.
- Avoid weaponization aesthetics entirely (hiring-funnel optics).

---

### Citations
GitHub data via `gh search repos` / repo views, 2026-05-26. Plus: [DimOS org](https://github.com/dimensionalOS/dimos), [DeepWiki: dimos-unitree](https://deepwiki.com/dimensionalOS/dimos-unitree), [The RoboVerse wiki](https://wiki.theroboverse.com/en/unitree-go2-app-console-commands), [WSO2 AI blog](https://medium.com/wso2-ai-blog/how-we-gave-life-to-an-ai-agent-with-the-unitree-go-2-robot-f9c7afec0a77), [UniStore launch](https://tech.news.am/eng/news/7169/), [Go2 review](https://blog.robozaps.com/b/unitree-go2-review), [SCMP G1 frontflip](https://www.scmp.com/tech/tech-trends/article/3337320/).
