# Synthesized Question Backlog: m20-rosnav-migration

## Synthesis Summary
- Models used: Opus 4.6, GPT 5.4, Gemini 3 Pro
- Raw questions from 9 analyses: 470
- After deduplication: 147
- P0: 28 | P1: 42 | P2: 48 | P3: 29

## Cross-Model Analysis

### User Advocate Convergence

**All 3 flagged (high confidence):**
- Operator readiness signal: all three models ask what single indicator tells the user the robot is ready. This was the strongest convergence across all analyses.
- Motion/performance expectations: all three flag that users will expect the new stack to produce noticeably better (or at least equivalent) navigation behavior, and will perceive no improvement as regression.
- Multi-floor expectation mismatch: all three warn that users will hear "multi-floor" and assume full autonomous floor-to-floor operation now, not in a later phase.
- Teleop fallback: all three ask whether manual control remains available and unchanged when autonomy fails.
- Recovery after failure/reboot: all three emphasize that users judge upgrades by recovery speed and clarity, not internal architecture improvements.
- Map/destination continuity: all three ask whether saved maps, waypoints, and destinations carry over.
- Docker/container crash behavior: all three flag the split architecture creating a scenario where the nav brain dies but the robot body keeps moving.

**Only 1 model flagged (worth attention):**
- Opus uniquely raised DDS topic collision from other ROS2 systems on the same network, per-robot calibration needs across a fleet, and incremental floor mapping.
- GPT uniquely raised the question of what prior system users are comparing this to (other AMRs, elevator-capable robots, app-style navigation), and overconfidence after initial success leading to riskier missions.
- Gemini uniquely raised battery drain from increased compute, fan noise, individual lidar feed access for diagnosing physical damage, and the lidar boot-before-network race condition from the user's perspective.

**Disagreements:**
- Gemini was notably more concerned with hardware-adjacent UX (noise, battery, boot speed) while Opus and GPT focused more on workflow and trust dynamics. No direct contradictions, but different emphasis.

### Product Designer Convergence

**All 3 flagged (high confidence):**
- Primary status hierarchy: what is the single most important piece of information at a glance (robot state, mission status, or system health)?
- Multi-floor UI representation: how to visually represent floor transitions and floor context.
- Recovery/error flow design: all three emphasize that recovery paths need as much design attention as happy paths.
- Goal-setting interaction: how does the user tell the robot where to go (map tap, named locations, coordinates)?
- Cancel/e-stop placement and interaction: safety-critical button must be omnipresent.
- States and transitions visual language: distinguishing idle, navigating, planning, stuck, recovering, degraded.
- Boot/startup progress communication: showing users what is happening during the multi-step startup sequence.

**Only 1 model flagged (worth attention):**
- Opus uniquely raised map editing interaction (correcting phantom walls from lidar reflections), patrol route setup/saving, and auto-zoom behavior.
- GPT uniquely asked about audit trails for state transitions, deployment-stage labeling (Phase 1 vs. full mode), and ownership context (who started this run).
- Gemini uniquely raised host-vs-container color coding, RPC call visualization, height cost display on costmaps, and the Foxglove vs. standalone web app question.

**Disagreements:**
- GPT emphasized role-based views (operator vs. supervisor vs. site manager) more strongly than the other two. Gemini was more concerned with the technical diagnostic layer (container status, RPC health). Opus struck a middle ground. No direct contradictions.

### Domain Expert Convergence

**All 3 flagged (high confidence):**
- Definition of "multi-floor transition" in the specific factory context (elevator vs. ramp vs. manual carry).
- Who the actual end-user is and their skill level.
- Whether multi-floor is a daily operational need or a demonstrative pilot feature.
- What the factory pilot means in business terms and what success criteria apply.
- How competing AMR platforms handle multi-floor navigation (benchmark expectations).
- What is explicitly NOT being solved (scope boundaries).
- Factory environment dynamism: layout changes, temporary obstacles, and map freshness.
- Wrong-floor relocalization and perceptual aliasing risks.
- Operator intervention rate as a key success metric.
- Recovery time after floor transitions.

**Only 1 model flagged (worth attention):**
- Opus uniquely raised geometric degeneracy in factory SLAM, clock drift between three boards, lidar cross-talk in multi-robot deployments, the Livox-to-RoboSense scan pattern difference as a potential research risk, and Python 3.8-to-3.10 as potentially unnecessary.
- GPT uniquely asked about the minimum outcome that justifies deployment, what business consequence occurs when the robot cannot resume, and whether relocalization-with-human-approval is acceptable.
- Gemini uniquely raised elevator-as-Faraday-cage (WiFi loss), payload changes between floors affecting dynamics, factory fire alarm/evacuation behavior, and field engineer satisfaction as a success metric.

**Disagreements:**
- Opus treats arise_slam porting as potentially a research risk rather than an engineering task. GPT and Gemini treat it more as a standard engineering challenge. This is a material disagreement about Phase 2 timeline risk.

---

## P0 — Must Answer

These questions were flagged by all 3 models or address critical impact issues that could block the pilot.

**1. What single indicator tells the operator the robot is ready for its first mission?**
With multiple components booting (host Python, Docker container, FASTLIO2, lidar), operators need one unambiguous "green light" before dispatching the robot.
*Raised by: Opus, GPT, Gemini*

**2. What does "multi-floor transition" actually mean in the pilot factory — elevator, ramp, manual carry, or a combination?**
The answer determines whether this is a discrete map-switching problem or a continuous 3D traversal problem, fundamentally changing the relocalization architecture and user workflow.
*Raised by: Opus, GPT, Gemini*

**3. Does the migration require a period of downtime where the robot cannot navigate at all, and if so, how long?**
Users operating the M20 daily need to know if the migration requires downtime and how long they lose a working robot.
*Raised by: Opus, GPT, Gemini*

**4. Will users expect noticeably better navigation from day one, and does Phase 1 actually deliver user-visible value or is it purely preparatory infrastructure?**
If Phase 1 produces a system functionally equivalent to or temporarily worse than the current setup, users will perceive regression. The disruption must be justified by tangible improvement.
*Raised by: Opus, GPT, Gemini*

**5. Will the multi-floor elevator transitions happen automatically, or does the user need to intervene at each floor — and how is this communicated?**
Users will hear "multi-floor navigation" and assume full autonomous floor-to-floor operation. Misaligned expectations during phased rollout create adoption blockers.
*Raised by: Opus, GPT, Gemini*

**6. Can users still teleoperate the robot when autonomous navigation is not working, and is the fallback identical to today?**
Fallback to manual control is a safety-critical expectation. Any change to latency, interface, or availability of teleop breaks operator trust.
*Raised by: Opus, GPT, Gemini*

**7. What happens if the Docker container running the nav stack crashes mid-mission — does the robot stop safely or continue driving blindly?**
The split architecture creates a scenario where the navigation brain dies while the robot body may still be in motion. Operators must see immediate, unmistakable failure indication.
*Raised by: Opus, GPT, Gemini*

**8. Will existing saved maps, waypoints, and mission definitions carry over from the current system?**
Users who spent hours mapping a facility will be deeply frustrated if they must redo all that work. Renaming or remapping creates immediate confusion.
*Raised by: Opus, GPT, Gemini*

**9. Who is the primary end-user during the factory pilot — robotics engineer, factory floor technician, or site manager — and what is their skill level?**
The skill level of the actual user determines information density, terminology, control safety, error message clarity, and how robust defaults must be.
*Raised by: Opus, GPT, Gemini*

**10. What job is the robot actually doing during the pilot — patrol, inspection, delivery, or cross-floor handoff?**
"Successful navigation" means different things in each workflow. Navigation is a means to an end; the business value depends on what the robot does when it arrives.
*Raised by: Opus, GPT, Gemini*

**11. What spaces are in scope and what spaces are explicitly out of bounds for the pilot?**
Restricted areas usually matter more operationally than nominal map coverage. Boundaries prevent the pilot from turning into a platform rewrite.
*Raised by: Opus, GPT, Gemini*

**12. What is explicitly NOT being solved in Phases 1-3, and what adjacent capabilities will users assume are included once "multi-floor navigation" is promised?**
Unstated assumptions about scope (elevator API integration, fleet management, dynamic obstacle prediction, load/unload automation) are the primary source of pilot failures.
*Raised by: Opus, GPT, Gemini*

**13. How do established factory AMR platforms (MiR, OTTO, Geek+, HIK Vision) handle multi-floor navigation, and what benchmark does the pilot customer expect?**
the pilot customer likely benchmarks against competitors with mature multi-floor products. Our approach must meet or exceed their operational workflow simplicity.
*Raised by: Opus, GPT, Gemini*

**14. What percentage of in-scope missions must complete without human intervention to call the pilot successful?**
Autonomy needs a clear bar. Without a defined threshold before the pilot, there is risk of moving goalposts or ambiguous outcomes.
*Raised by: Opus, GPT, Gemini*

**15. After a floor transition or manual relocation, how quickly must the robot become operational again?**
Recovery time is user-visible value. Industry leaders achieve sub-5-second relocalization; if arise_slam takes 30+ seconds, the system may be unusable where elevators have timeout behaviors.
*Raised by: Opus, GPT, Gemini*

**16. How many operator interventions per shift are acceptable, and which recovery actions must be doable by non-expert site staff?**
This directly measures babysitting load. Pilot scalability depends on who can unblock the robot, not just whether it can be unblocked.
*Raised by: Opus, GPT, Gemini*

**17. What happens if staff manually move the robot between floors outside the intended workflow (kidnapped robot scenario)?**
When the robot boots up after being carried, it has no odometry history to hint at its new floor. This is common in real pilots.
*Raised by: Opus, GPT, Gemini*

**18. How should the system behave when the factory floor layout has changed significantly since the map was made (pallets moved, walls added, temporary structures)?**
Factories are rarely static. The robot needs an operational behavior (stop and ask for help, replan, etc.) rather than blindly forcing through.
*Raised by: Opus, GPT, Gemini*

**19. How will the user know if FASTLIO2 SLAM has lost localization — and what is the distinction between degraded localization (high uncertainty) and total loss?**
Users expect clear, immediate feedback when the robot is "lost." Silent degradation where it drives into walls is the worst outcome. Treating localization as binary (working/not) misses the critical middle ground.
*Raised by: Opus, GPT, Gemini*

**20. What are all the top-level user-visible system states, and what distinct visual treatment does each require?**
Booting, initializing, mapping, idle, navigating, replanning, stuck, error, recovery, shutting down — wireframes should be built around state models, not just happy-path screens.
*Raised by: Opus, GPT, Gemini*

**21. What is the happy path for powering on the M20, waiting for ready, setting a goal, navigating, and arriving — and what does each step look like?**
The end-to-end flow defines the baseline experience. Every deviation is an error state that needs handling.
*Raised by: Opus, GPT, Gemini*

**22. What is the recovery flow when the robot becomes uncertain about its position or the path is blocked?**
This is a trust-critical moment where users need clear choices, not jargon. Recovery paths must be as designed and rehearsed as happy paths.
*Raised by: Opus, GPT, Gemini*

**23. How does pause, resume, and cancel work mid-mission — including the immediate visual feedback for emergency stop?**
Emergency stops must be faster and more discoverable than any other interaction. The user must instantly see that their stop command was registered.
*Raised by: Opus, GPT, Gemini*

**24. How should multi-floor context be represented visually — tabs, floor picker, 3D building view, or stacked cards?**
The visual metaphor for floor switching affects how quickly operators understand the robot's location. Even if multi-floor comes in Phase 2, the information model should not need a redesign.
*Raised by: Opus, GPT, Gemini*

**25. What is the "factory pilot" in business terms — a demo, a shadow run, or a production-adjacent deployment — and what evidence triggers Phase 2-to-3 progression?**
A demo, shadow run, and production pilot require very different rigor. Staged programs need explicit go/no-go gates.
*Raised by: Opus, GPT, Gemini*

**26. Is the real problem floor-to-floor operation, or that recovery currently needs expert intervention?**
Symptom and root cause point to different scope decisions. Reducing babysitting is often a stronger value proposition than adding features.
*Raised by: Opus, GPT, Gemini*

**27. What uptime across shifts, days, or weeks is required before the customer considers the system stable?**
Demos and production pilots use different stability bars. Factory logistics systems typically target 200+ hours MTBF and sub-5-minute MTTR.
*Raised by: Opus, GPT, Gemini*

**28. What site-specific safety, compliance, or regulatory requirements govern autonomous movement in the pilot factory?**
China has specific standards for AGVs/AMRs (GB/T 30029, GB/T 20721). An operationally disallowed workflow is still a failure regardless of technical capability.
*Raised by: Opus, GPT, Gemini*

---

## P1 — Should Answer

These questions were flagged by 2 models or address moderate-impact issues.

**29. Will the robot's top speed, acceleration profile, or responsiveness change after migration?**
Operators who have built muscle memory around current motion behavior will be confused or alarmed if the robot suddenly moves differently.
*Raised by: Opus, Gemini*

**30. Will the web visualization (WebVis/map/costmap/path display) look and behave the same?**
Users rely on visual feedback to trust the robot's decisions. Changes to the UI aesthetics without warning erode confidence.
*Raised by: Opus, Gemini*

**31. Can users roll back to the old system if the new one has problems in the field?**
Users expect a safety net. If rolling back requires SSH and 20 terminal commands, the shift is ruined.
*Raised by: Opus, GPT*

**32. Does the robot boot faster or slower with the new architecture, and what is the boot-to-ready time?**
Factory workers hate waiting. Booting host Python plus a Docker container may change startup times compared to the current setup.
*Raised by: Opus, Gemini*

**33. What is the process for a new operator joining mid-pilot to get up to speed without the original deployer present?**
Knowledge trapped in one person's head is a deployment risk. The system should be self-documenting enough for handoffs.
*Raised by: Opus, GPT*

**34. What does the user do when the robot gets stuck or behaves unexpectedly — and are recovery actions available without SSH or developer knowledge?**
Recovery must be obvious and doable without terminal access. If recovery requires knowing the right internal engineer, inclusive operations fail.
*Raised by: Opus, GPT*

**35. How does the user verify that the robot's localization is correct before starting a mission?**
Users need a quick sanity check — "does the robot know where it is?" — before trusting it to navigate autonomously.
*Raised by: Opus, Gemini*

**36. What happens if the user sends a navigation goal while the robot is already navigating to a different goal?**
Users will do this constantly. Does the new goal replace the old one, queue behind it, or get rejected? The behavior must be predictable.
*Raised by: Opus, GPT*

**37. What happens if the user power-cycles or force-reboots the robot mid-navigation — does the system recover cleanly?**
Factory workers pull the plug when things go wrong. The system must not corrupt the map or permanently lose its global position.
*Raised by: Opus, Gemini*

**38. What happens if someone manually pushes or relocates the robot while SLAM is running?**
SLAM systems lose localization when the robot is displaced without IMU-consistent motion. Users won't know this.
*Raised by: Opus, GPT*

**39. Can a user who does not know ROS, Docker, or Python operate this system day-to-day?**
If daily operation requires developer skills, the addressable user base shrinks to the engineering team only. Operators are not always the people who set the system up.
*Raised by: Opus, GPT*

**40. Are status messages and error reports available in Chinese (or languages other than English)?**
The pilot customer is a Chinese company. Factory operators may not read English error messages. Mixed-language teams are common in industrial sites.
*Raised by: Opus, GPT*

**41. Is there an audible or physical indicator when the robot changes state (starts moving, stops, encounters error)?**
In a noisy factory, visual-only feedback on a distant screen is insufficient. Workers expect specific audible beeps or light signals.
*Raised by: Opus, Gemini*

**42. What is the single most important piece of information at a glance — and should the UI separate mission status from system health?**
Operators need to distinguish "the task is blocked" from "the robot is unhealthy." The topmost status area and visual hierarchy flow from this choice.
*Raised by: Opus, GPT*

**43. How should the system communicate the difference between host process health and nav container health?**
The rosnav pattern splits the system across two runtimes. Operators must understand which half has failed, or they troubleshoot the wrong component.
*Raised by: Opus, Gemini*

**44. Should lidar health (dual RSAIRY merge status, partial failure) be a persistent indicator or only surfaced on failure?**
If one lidar gets smashed but the merged topic still publishes from the surviving one, the user might unknowingly operate a half-blind robot.
*Raised by: Opus, Gemini*

**45. What map layers should be visible by default: costmap, voxel grid, point cloud, planned path, or traveled path?**
Showing everything overwhelms; showing too little hides problems. The default determines how quickly operators assess the situation.
*Raised by: Opus, Gemini*

**46. How does an operator start autonomous navigation — single "go" button, map tap, coordinate entry, or named-location picker?**
The input method determines the skill level required. pilot factory workers need something simpler than typing coordinates.
*Raised by: Opus, GPT*

**47. How does the operator trigger a container restart or system recovery without SSH access?**
DockerModule has restart policies, but operators need a manual override that is safe and easy to find through the UI.
*Raised by: Opus, Gemini*

**48. What feedback does the operator receive between sending a goal and the robot's first movement?**
The pipeline (goal request through planning through path through velocity command) introduces latency. Silence during planning feels like failure.
*Raised by: Opus, Gemini*

**49. What is the interaction for switching between teleop and autonomous mode, and how is dual-control prevented?**
The M20 has Regular Mode (UDP) and Navigation Mode (NAV_CMD). Switching must be clear and prevent accidental conflicting commands.
*Raised by: Opus, Gemini*

**50. What is the boot sequence from the operator's perspective — how long does each phase take, and what do they see?**
NOS boot, uv/Python start, dimos host, Docker start, FASTLIO2 init, lidar lock, ready — each phase has different duration and failure mode. Users need a progress indicator, not a spinner.
*Raised by: Opus, Gemini*

**51. Error flow: NOS runs out of RAM (only ~2GB usable). What does the operator see before the OOM killer strikes?**
OOM kills are silent and confusing. The system should warn before catastrophic failure. Users need to know how to gracefully stop non-essential modules.
*Raised by: Opus, Gemini*

**52. What is the flow for a shift handoff — how does one operator transfer the robot's state and context to the next?**
Factory operations run 24/7. The robot's mission state, destination, and active errors must persist seamlessly across human handovers.
*Raised by: Opus, GPT*

**53. Where does the navigation monitoring view live — standalone web app, existing command center, Foxglove panel, or mobile interface?**
The pilot factory floor may not have a desktop nearby. The platform choice determines what interactions are feasible and what design system applies.
*Raised by: Opus, Gemini*

**54. Should the map view be 2D (top-down costmap), 3D (point cloud), or toggleable — and what is the default?**
2D is simpler for goal setting; 3D reveals terrain issues. The default affects cognitive load.
*Raised by: Opus, Gemini*

**55. Where should the emergency stop / cancel goal button be placed — and does it need to be omnipresent across all screen states?**
E-stop is safety-critical. It must never be occluded by modals, scroll position, or screen transitions.
*Raised by: Opus, Gemini*

**56. How should the UI show transition states like "starting," "becoming ready," or "recovering" versus a hard failure?**
Users make very different decisions for temporary uncertainty versus hard failure. Time thresholds shape trust and escalation behavior.
*Raised by: Opus, GPT*

**57. How should the Docker container auto-restart be communicated — crash notification, restart progress, and remaining restart attempts?**
A silent restart hides systemic issues. The operator needs to know it crashed, it is restarting, and how many restarts remain before the policy gives up.
*Raised by: Opus, Gemini*

**58. Which prior navigation failures or deployment pain points were most painful for operators?**
The brief should target remembered field pain, not abstract capability gaps. Reducing babysitting is often more valuable than adding features.
*Raised by: Opus, GPT*

**59. What recovery workflow do operators already consider "normal," and does replacing an accepted manual step actually add value?**
Replacing a low-friction manual step may not be the highest-value improvement compared to fixing high-pain failure modes.
*Raised by: Opus, GPT*

**60. How often do pilot missions actually require floor transitions, and would highly reliable single-floor autonomy already unlock the pilot?**
Rare transitions may not justify early complexity. This tests whether multi-floor is essential or aspirational for Phase 1.
*Raised by: Opus, GPT*

**61. Are we solving for fewer failed missions, fewer human touches, or faster site commissioning — and which metric drives expansion decisions?**
The success metric changes materially depending on the answer. Task-level benefit (coverage, throughput, response time) must be explicit.
*Raised by: Opus, GPT*

**62. What should happen during peak pedestrian or forklift traffic periods?**
A solution that only works off-shift may fail in reality. Dynamic obstacle handling is not optional in factory settings.
*Raised by: Opus, GPT*

**63. How should the robot recover after emergency stops, fire drills, or manual shutdowns mid-mission?**
Abnormal recovery is heavily judged in pilots. E-stop while halfway in/out of an elevator could break the relocalization state machine.
*Raised by: Opus, Gemini*

**64. What environmental conditions have historically broken robots at this site (dust, steam, reflective surfaces, condensation)?**
Industrial manufacturing involves thermal processes. Moving between climate zones can cause rapid condensation on sensors, temporarily blinding the lidar.
*Raised by: Opus, GPT*

**65. What promises have already been made to pilot stakeholders about autonomy level or timeline?**
Hidden commitments create scope conflict later. The success bar must be aligned before Phase 1 begins.
*Raised by: Opus, GPT*

**66. What safety indicators must improve or at least not regress during the pilot?**
No factory pilot survives if safety perception worsens. Safety validation methodology matters in a industrial manufacturing facility.
*Raised by: Opus, GPT*

**67. What operator-confidence signals will count as success beyond hard metrics?**
Qualitative trust often decides rollout before hard ROI is mature. This includes social acceptance of the robot's signaling.
*Raised by: Opus, GPT*

**68. Is the real problem that lio_perception cannot relocalize, or that the monolithic architecture cannot be iterated on quickly enough — and are these being conflated?**
The brief conflates architectural ambitions (developer velocity) with business capabilities (relocalization). These could be addressed independently, and conflating them doubles the risk surface.
*Raised by: Opus, GPT*

**69. Does the three-phase plan address the risk that Phase 2 (arise_slam porting from Livox to RoboSense) may be a research problem rather than an engineering task?**
Livox and mechanical lidars produce fundamentally different point distributions. If Phase 2 is on the critical path and turns out to be research, the entire pilot timeline is at risk.
*Raised by: Opus, Gemini*

**70. Is replacing the entire perception-to-navigation pipeline simultaneously the right approach, or should components be replaced incrementally?**
Replacing lio_perception, the planner, mapping, AND the architecture at once means we cannot isolate which change caused a regression. Incremental replacement reduces risk.
*Raised by: Opus, GPT*

---

## P2 — Good to Have

These questions were flagged by 1 model but are design-relevant.

**71. Will battery consumption change with the new architecture (host Python + Docker vs. single container)?**
Running complex SLAM and Docker on constrained hardware can increase compute load. Users expect battery life to remain constant or improve.
*Raised by: Gemini*

**72. Will the robot be louder due to increased compute load (fans spinning faster)?**
In quiet factory zones, increased noise pollution from the robot is a noticeable annoyance.
*Raised by: Gemini*

**73. Can the user still view individual lidar feeds to diagnose physical damage or dirt on a specific sensor?**
The brief mentions merging dual lidars into a single topic. Advanced operators often need individual feeds for hardware diagnostics.
*Raised by: Gemini*

**74. Does the new system handle dynamic obstacles (people, forklifts) as well or better than the current one?**
In a factory setting, dynamic obstacle avoidance is not optional. Users assume it works at least as well.
*Raised by: Opus*

**75. Will the system work the same on all M20 units, or does each robot need individual calibration after migration?**
Fleet operators assume a deployment is a deployment. Per-robot tuning is an unwelcome surprise.
*Raised by: Opus*

**76. Is the goal-setting interface (how users tell the robot where to go) changing at all?**
Users expect the same workflow for sending goals. Any change to the interaction model needs explicit communication.
*Raised by: Opus*

**77. What does the user experience when they arrive at the factory site — what is the first-time setup flow?**
First impressions set expectations. A painful initial setup poisons the entire pilot.
*Raised by: Opus*

**78. How does the user build or load a map for the first time, and does FASTLIO2 require different mapping procedures?**
Mapping is usually the most confusing step for non-expert users. If the new SLAM stack requires a complex "loop closure" driving pattern that the old system didn't, users will fail.
*Raised by: Opus, Gemini*

**79. What does the user do at end of shift — must they shut down gracefully, or can they just power off?**
Abrupt power-off is reality in factory environments. The system must tolerate it without corruption.
*Raised by: Opus*

**80. How does the user update the system when a new version is available, and can it be done without SSH?**
If updates require SSH, git pulls, and docker builds, only a developer can do it — that is a deployment bottleneck.
*Raised by: Opus, Gemini*

**81. What information does the user need to report a problem to the support team?**
If users cannot easily capture logs or diagnostic info, every support interaction starts with "can you SSH in?"
*Raised by: Opus*

**82. What happens if the user starts the system in a featureless area (empty warehouse, long corridor)?**
SLAM degeneracy is a real problem. The system should warn users rather than silently producing bad localization.
*Raised by: Opus*

**83. What happens if the user deploys in an environment with glass walls, mirrors, or highly reflective surfaces?**
Lidar-based systems fail in these conditions. Users in modern factories with glass offices will encounter this.
*Raised by: Opus, Gemini*

**84. What happens if a user sends the robot to a floor that has not been mapped yet?**
Clear error messaging is critical — "floor not mapped" is actionable; a silent failure or crash is not.
*Raised by: Opus*

**85. What if WiFi between NOS and the user's monitoring device drops during a mission?**
The robot should continue autonomously. The user expects to reconnect and see the robot is still working. Elevators are often Faraday cages.
*Raised by: Opus, Gemini*

**86. What if multiple users send conflicting goals to the same robot simultaneously?**
In a factory with multiple operators, command authority must be clear and predictable.
*Raised by: Opus, GPT*

**87. Can a colorblind user interpret the web visualization (costmap colors, status indicators)?**
Red/green status indicators and heatmap color schemes are common accessibility failures.
*Raised by: Opus*

**88. Are log messages and diagnostics written in plain language, or do they require domain expertise?**
"FASTLIO2 degenerate scan detected" means nothing to a factory technician — "Robot lost: move to open area" does.
*Raised by: Opus*

**89. Is there documentation or training material for non-technical factory staff?**
Assuming users will read code or architecture docs is a mistake. They need visual guides and checklists.
*Raised by: Opus, Gemini*

**90. What assumptions are we making about network infrastructure at the deployment site (WiFi reliability, multicast support, IT policies)?**
Not every factory has reliable WiFi, low-latency networking, or IT staff who will whitelist multicast traffic.
*Raised by: Opus*

**91. What happens for users in environments with strict IT security policies (no Docker Hub access, no outbound internet)?**
Air-gapped or restricted networks are common in manufacturing. The deployment pipeline must account for this.
*Raised by: Opus*

**92. How should the system present the distinction between FASTLIO2 SLAM pose and the robot's raw odometry?**
When these diverge, the robot is "confused." Showing two competing positions without context creates confusion.
*Raised by: Opus*

**93. Should Phase 2 multi-floor relocalization expose "current floor" as a first-class concept in the UI, or treat it as metadata?**
If the robot transitions between floors, misidentifying the current floor could send the robot into a wall.
*Raised by: Opus, Gemini*

**94. How should container lifecycle state (pulling image, starting, ready, crashed, restarting) be presented relative to mission state?**
DockerModule introduces startup latency and failure modes invisible in the monolithic setup. These must be legible without SSH.
*Raised by: Opus*

**95. What information density is appropriate for a field engineer at the pilot site versus a remote developer debugging?**
Non-developer operators may be overwhelmed by the data a developer needs. This may require role-based views.
*Raised by: Opus, GPT*

**96. Should velocity commands be shown as raw numbers, visual vectors, or abstracted into "speed: fast/slow/stopped"?**
Raw values are meaningless to non-roboticists; overly abstracted values hide diagnostic info.
*Raised by: Opus*

**97. How should the system communicate "degraded mode" — navigation working but no camera, or SLAM running but costmap stale?**
Partial failures are the most common real-world scenario. The UI must not treat everything as binary healthy/broken.
*Raised by: Opus*

**98. What should the "no map yet" state look like when FASTLIO2 is building its first SLAM map?**
Cold start is real. If the UI shows a blank map with no explanation, operators will assume the system is broken.
*Raised by: Opus*

**99. How should goal/waypoint information be structured — single destination, ordered queue, or flexible set?**
The factory pilot likely involves multi-stop routes. The UI must support showing where the robot is in a sequence.
*Raised by: Opus*

**100. What inputs are required vs. optional when setting a navigation goal, and what are sensible defaults?**
Position is required, but what about orientation, speed limit, obstacle avoidance aggressiveness? Optional parameters should have sensible defaults.
*Raised by: Opus*

**101. How does the operator mark an area as "do not enter" or adjust the costmap in real time?**
Factory floors change — spills, parked forklifts. If there is no operator input, the robot will plan through hazards.
*Raised by: Opus*

**102. How should the operator provide feedback when the robot takes a bad path — is there a "that was wrong" signal?**
Without a lightweight feedback mechanism, operators will not bother to report path quality issues.
*Raised by: Opus*

**103. How does an operator set up a repeatable patrol route (naming, saving, loading, editing)?**
Multi-stop navigation with saved routes is a core the pilot customer use case. The interaction must support route management.
*Raised by: Opus*

**104. What is the flow for the initial SLAM map building session in a new factory section?**
First-time mapping is a distinct workflow from steady-state navigation. It may require manual driving, landmark annotation, or map validation.
*Raised by: Opus, Gemini*

**105. Error flow: lidar topic goes stale (/lidar_points stops publishing). What happens in the UI?**
The entrypoint has lidar health recovery logic, but the operator needs to know whether the system is self-healing or needs manual restart.
*Raised by: Opus, Gemini*

**106. What happens when the robot reaches a goal — does it stop, await instructions, proceed to next waypoint, or return home?**
Post-goal behavior must match pilot factory workflow. Wrong default behavior means the operator must intervene at every stop.
*Raised by: Opus, GPT*

**107. What visual pattern should indicate planned path versus actual traveled path?**
Showing both reveals planning quality. Different visual treatments (color, opacity, line style) communicate different things.
*Raised by: Opus*

**108. Should the interface support dark mode for factory environments with variable lighting?**
A bright white UI in a dim warehouse causes eye strain and reduces situational awareness.
*Raised by: Opus*

**109. What is the appropriate map zoom behavior — auto-zoom to surroundings, full map, or planned path extent?**
Auto-zoom that changes unexpectedly is disorienting; fixed zoom that cannot show the goal is frustrating.
*Raised by: Opus*

**110. How does the system represent "waiting for lidar" versus "lidar healthy but no SLAM pose yet"?**
These are different boot stages with different expected durations and different failure remedies.
*Raised by: Opus, Gemini*

**111. What state represents "robot is physically stuck but system is healthy" (wedged against obstacle)?**
The nav stack may keep trying to move, consuming battery, while the operator sees a "navigating" state that is technically true but practically wrong.
*Raised by: Opus*

**112. What state represents "goal unreachable" — is it an error state or a special idle state with actionable info?**
The operator needs different options for "no path exists" (choose a different goal) versus "path exists but is blocked" (wait or reroute).
*Raised by: Opus*

**113. What does the "shutdown" flow look like — does the operator stop navigation first, then container, then dimos, or is there a single action?**
Improper shutdown ordering could corrupt the SLAM map or leave Docker in a bad state. The flow must enforce safe ordering.
*Raised by: Opus, Gemini*

**114. How does the system handle the transition when NOS loses network connectivity to AOS?**
DDS topics from AOS would stop flowing. The system must transition to a safe state and communicate the cause.
*Raised by: Opus*

**115. What is the operational definition of "map" — a pre-built static reference, a live-updated SLAM map, or a hybrid?**
The relationship between the live map and stored reference maps needs to be defined to avoid confusion about what "loading a floor's map" means.
*Raised by: Opus*

**116. What is "base autonomy" in the CMU nav stack versus what dimos provides, and where is the boundary of responsibility?**
Overlapping responsibilities between dimos and the CMU stack (obstacle avoidance, recovery behaviors, speed adaptation, safety stops) will cause conflicting commands.
*Raised by: Opus*

**117. What is the relationship between the container-side FAR planner costmap and the host-side dimos CostMapper — which is authoritative?**
If both produce costmaps, they must be reconciled. Which one governs collision avoidance?
*Raised by: Opus*

**118. How do the M20's proprietary drdds and standard ROS2 DDS interoperate?**
They may use different DDS implementations with different discovery and QoS behaviors, causing subtle communication issues.
*Raised by: Opus*

---

## P3 — Parking Lot

Technical questions that may be important later but are not required for scoping.

**119. What if a separate ROS2 application is running on the same network — could DDS discovery cause topic collisions?**
*Raised by: Opus*

**120. What if the Docker container runs out of memory on NOS (only ~2GB usable)?**
*Raised by: Opus, Gemini*

**121. What if the user accidentally triggers navigation while the robot is on a charging dock or tethered?**
*Raised by: Opus*

**122. What if the user wants to add a new floor to the map after initial deployment — is incremental mapping supported?**
*Raised by: Opus*

**123. Can a user with limited mobility operate the teleop fallback controls?**
*Raised by: Opus*

**124. What if the operator's monitoring device has a small screen (phone) versus a large screen (tablet)?**
The web visualization must be responsive.
*Raised by: Opus*

**125. Are we assuming the user has physical access to the NOS board for troubleshooting?**
*Raised by: Opus*

**126. Are we assuming a specific level of familiarity with M20 hardware basics?**
*Raised by: Opus*

**127. How should the system ask for missing information (e.g., SSH credentials for NOS) without feeling blocking?**
*Raised by: GPT, Gemini*

**128. What level of permissions should different roles have (operator gets "pause," supervisor gets "reconfigure")?**
*Raised by: GPT*

**129. What feedback should appear after success: silent completion, toast, summary card, or next-step suggestion?**
*Raised by: GPT*

**130. Should the interface show deployment-stage context like "Phase 1 pilot" versus "full multi-floor mode"?**
*Raised by: GPT*

**131. Do users need ownership context such as "who started this run" or "who last acknowledged an issue"?**
*Raised by: GPT*

**132. What audit or confirmation trail is needed when key state transitions happen?**
*Raised by: GPT*

**133. How should the system request the SSH sudo password if manual NOS intervention is required through the UI?**
*Raised by: Gemini*

**134. How should the system visually group mapping modules (VoxelGridMapper) versus navigation modules (AStarPlanner)?**
*Raised by: Gemini*

**135. In a service conflict (e.g., height_map_nav vs dimos-mac-bridge), what info does the user need to resolve it?**
*Raised by: Gemini*

**136. What interactions are available when viewing the 3D VoxelBlockGrid map (pan, zoom, rotate, filter by height)?**
*Raised by: Gemini*

**137. How should system resource usage (NOS host RAM/CPU) be displayed — front-and-center or under a diagnostic tab?**
*Raised by: Gemini*

**138. How should active RPC calls between host and container be visualized, if at all?**
*Raised by: Gemini*

**139. How should secondary telemetry (like the 512MB rerun memory limit) be displayed without cluttering the main view?**
*Raised by: Gemini*

**140. What color coding should separate host-level processes from container-level processes?**
*Raised by: Gemini*

**141. What iconography best represents dual lidar merging status versus single pointcloud output?**
*Raised by: Gemini*

**142. Should the robot's current pose be strictly centered on screen, or can the user pan the map independently?**
*Raised by: Gemini*

**143. How should height cost (terrain slope gradient) be displayed on the global costmap?**
*Raised by: Gemini*

**144. How does the system handle clock drift and time synchronization between the three boards (AOS, GOS, NOS) and the Docker container?**
*Raised by: Opus*

**145. How does the system handle electromagnetic interference in a industrial manufacturing facility?**
*Raised by: Opus*

**146. What happens when two M20 robots operate on the same floor — can their lidar scans interfere with each other?**
*Raised by: Opus*

**147. What if the payload characteristics change significantly between floors, affecting braking distance and dynamics?**
*Raised by: Gemini*

---

## Quality Check
- Raw questions from 9 analyses: **470**
- After deduplication (merging similar): **147**
- P0 (28) + P1 (42) + P2 (48) + P3 (29) = **147** (matches unique count)
- Global sequential numbering: 1 through 147
- All models attributed per question
