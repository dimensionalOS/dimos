# M20 ROSNav Migration Spec — Review Assessment

**Reviewed:** 2026-03-13
**Spec version:** Validated (2026-03-13)
**Reviewer:** Engineering assessment (automated)

---

## Assessment 1: Completeness Check Against 147 Scope Questions

### Coverage Summary

| Priority | Total | Addressed | Partially | Missing | Deferred | Coverage |
|----------|-------|-----------|-----------|---------|----------|----------|
| P0 (1-28) | 28 | 22 | 4 | 0 | 2 | 93% |
| P1 (29-70) | 42 | 18 | 10 | 6 | 8 | 67% |
| P2 (71-118) | 48 | 14 | 11 | 19 | 4 | 52% |
| P3 (119-147) | 29 | 8 | 5 | 16 | 0 | 45% |
| **Total** | **147** | **62** | **30** | **41** | **14** | **63%** |

### P0 Questions — Detailed Status

| # | Question | Status | Notes |
|---|----------|--------|-------|
| 1 | Ready indicator | Partially | Spec describes boot sequence and health monitoring but does NOT define a single composite "READY" indicator. Error Handling section covers failure states but not the readiness aggregation UX. |
| 2 | Multi-floor transition meaning | Addressed | Key Decisions Table: ramps AND staircases, continuous traversal. |
| 3 | Migration downtime | Addressed | Section 1.1 describes one-time setup. deploy.sh changes in 1.7 imply managed transition. |
| 4 | Phase 1 user-visible value | Addressed | Key Decisions Table: currently ZERO autonomy, Phase 1 delivers first-ever autonomous nav. |
| 5 | Automatic vs manual transitions | Addressed | Continuous traversal, no user intervention. |
| 6 | Teleop fallback | Partially | M20Connection (1.4) keeps UDP protocol for direct commands. However, the spec does not explicitly state that teleop remains identical to today or describe the teleop/autonomy mode-switching flow. |
| 7 | Container crash safety | Addressed | Error Handling > Container Crash: robot stops (500ms watchdog), host reports "Navigation Unavailable." |
| 8 | Map/waypoint carryover | Addressed | Key Decisions Table: no migration, start fresh with FASTLIO2. |
| 9 | Primary end-user | Addressed | Key Decisions Table: Houmanoids engineering team. |
| 10 | Robot job during pilot | Addressed | Key Decisions Table: patrol + inspection. |
| 11 | In-scope spaces | Partially | pilot factory mentioned but no explicit enumeration of which floors/areas are in-scope for the pilot vs. out-of-bounds. Phase 3 validation is "lab/office" not the pilot customer. |
| 12 | Explicit exclusions | Addressed | Out of Scope table + Key Decisions Table Q12 provide comprehensive exclusion list. |
| 13 | AMR competitor benchmarking | Deferred | Deferred table: Houdini epic. |
| 14 | Mission success threshold | Addressed | Zero-intervention target, iterative. |
| 15 | Floor transition recovery time | Partially | Phase 2 validation criteria mention <10s relocalization, but no recovery time SLA for actual floor transitions during Phase 3. |
| 16 | Operator interventions per shift | Addressed | Zero target, iterative. |
| 17 | Kidnapped robot scenario | Addressed | Phase 2 relocalization covers this. Phase 1 limitation acknowledged (FASTLIO2 cannot relocalize). |
| 18 | Behavior when layout changed | Addressed | Key Decisions Table: dynamic replanning via CMU stack. |
| 19 | SLAM loss indication | Addressed | Error Handling > SLAM Divergence: "Localization Degraded" vs "Localization Lost" states. |
| 20 | System states visual treatment | Addressed | Error Handling section defines states: Navigating, Navigation Unavailable, Localization Degraded, Localization Lost. But no comprehensive state model diagram or exhaustive list. |
| 21 | Happy path end-to-end | Addressed | Data Flow section (Steps 1-7) describes the full pipeline from lidar to goal setting. |
| 22 | Recovery flow when lost | Addressed | SLAM Divergence recovery: stop, teleop to feature-rich area, SLAM reconverges. Phase 2 adds relocalization. |
| 23 | Pause/resume/cancel + e-stop | Addressed | ROSNav bridge has `/cancel_goal`, `/soft_stop`, `/joy` outputs. Hardware e-stop mentioned (Q28). |
| 24 | Multi-floor visual representation | Deferred | Deferred table: Phase 2+. |
| 25 | Factory pilot business terms | Addressed | Key Decisions Table: demo / proof of concept. |
| 26 | Real problem: multi-floor or babysitting | Addressed | Implicitly: Phase 1 delivers first autonomy (reducing babysitting from 100% to near-zero). Multi-floor is Phase 3. |
| 27 | Uptime SLA | Deferred | Deferred table: post-demo. |
| 28 | Safety/compliance | Addressed | Standard e-stop, hardware HES button. |

### P1 Questions — Detailed Status

| # | Question | Status | Notes |
|---|----------|--------|-------|
| 29 | Speed/acceleration changes | Missing | No comparison of velocity limits between old and new stack. M20ROSNavConfig has no max speed config. |
| 30 | Web visualization unchanged | Addressed | WebsocketVisModule and RerunBridge listed as "use as-is" / "no changes needed." |
| 31 | Rollback capability | Addressed | Out of Scope table: "deploy.sh can redeploy old image if needed." |
| 32 | Boot-to-ready time | Missing | No timing estimates for the new host+container boot sequence. |
| 33 | New operator onboarding | Deferred | Not in spec. Partially deferred (Q89 training materials deferred). |
| 34 | Recovery without SSH | Missing | Spec recovery actions (SLAM loss, container crash) all assume engineering team with SSH. No UI-accessible recovery buttons. |
| 35 | Verify localization correct | Partially | dimos viewer shows SLAM output but no explicit "localization confidence check" before mission start. |
| 36 | Goal while navigating | Partially | ROSNav bridge has `/cancel_goal` + `/goal_pose` but spec does not state the policy for conflicting goals (replace, queue, reject). |
| 37 | Power-cycle recovery | Addressed | Phase 1: restart from 0,0,0. Phase 2: relocalization. Container restart policy handles Docker side. |
| 38 | Manual push during SLAM | Addressed | Error Handling > SLAM Divergence covers displacement scenarios. |
| 39 | Non-ROS/Docker operator | Addressed | End user is engineering team (Q9). Non-technical operation explicitly deferred to Houdini. |
| 40 | Chinese language | Deferred | Deferred table: Houdini epic. |
| 41 | Audible/physical state indicators | Missing | No mention of sounds, lights, or physical indicators for state changes. |
| 42 | Most important info at glance | Partially | States defined in Error Handling but no explicit UI information hierarchy decision. |
| 43 | Host vs container health distinction | Addressed | Error Handling > Container Crash distinguishes host and container states. deploy.sh status shows both. |
| 44 | Lidar health persistent indicator | Addressed | Error Handling > Lidar Failure: monitor publish rate, thresholds defined (10Hz normal, 5Hz degraded, 0 failure). |
| 45 | Default map layers | Missing | No specification of which layers show by default in viewer or command center. |
| 46 | Goal-setting interaction | Addressed | Data Flow Step 7: "Dimos Viewer / Command Center -> goal_req (LCM)." Uses existing UI. |
| 47 | Container restart without SSH | Missing | docker_restart_policy is automatic. No UI button for manual container restart. |
| 48 | Feedback between goal and movement | Partially | Data flow shows pipeline but no latency expectations or intermediate "planning..." feedback state. |
| 49 | Teleop/autonomy mode switch | Partially | M20Connection has dual paths (UDP teleop vs NAV_CMD nav) but spec does not define the switching UX or dual-control prevention. |
| 50 | Boot sequence operator view | Missing | No phase-by-phase boot timing or progress indicator specification. |
| 51 | OOM operator warning | Addressed | Error Handling > OOM on NOS: warning at >80% usage, container memory-limited. |
| 52 | Shift handoff | Deferred | Deferred table: post-demo. |
| 53 | Monitoring view location | Addressed | Key Decisions Table: dimos viewer (Rerun fork). |
| 54 | Map view 2D/3D default | Partially | Both VoxelGridMapper (3D) and CostMapper (2D OccupancyGrid) exist. No stated default. |
| 55 | E-stop button placement | Partially | Hardware HES button mentioned. Software cancel via ROSNav. No UI placement spec. |
| 56 | Transition state visual | Partially | States listed in Error Handling but no visual design for transition states vs hard failure. |
| 57 | Auto-restart communication | Partially | docker_restart_policy: "on-failure:3" specified. After 3 failures, "permanent failure" reported. No UX for restart progress or remaining attempts. |
| 58 | Prior navigation pain points | Missing | No retrospective analysis of current system pain points. |
| 59 | Current "normal" recovery workflows | Missing | No documentation of existing accepted manual steps. |
| 60 | Floor transition frequency | Addressed | Implicitly: Phase 1 is single-floor only. Multi-floor is Phase 3. Single-floor autonomy unlocks pilot value. |
| 61 | Primary success metric | Addressed | Zero-intervention autonomous operation (Q14, Q16). |
| 62 | Peak traffic behavior | Addressed | Q18/Error Handling: dynamic replanning via CMU base_autonomy + FAR planner for obstacle avoidance. |
| 63 | Recovery after e-stop/fire drill | Partially | SLAM reconvergence described but no explicit e-stop recovery protocol. |
| 64 | Environmental conditions | Addressed | Key Decisions Table: dust + reflective surfaces = tuning, not blockers. |
| 65 | Promises to stakeholders | Deferred | Deferred table: Houdini epic. |
| 66 | Safety indicator regression | Missing | No safety baseline defined. No regression test for safety metrics. |
| 67 | Operator confidence signals | Missing | No qualitative success criteria defined. |
| 68 | Architecture vs relocalization conflation | Addressed | Phased approach explicitly separates architecture (Phase 1) from relocalization (Phase 2). |
| 69 | arise_slam porting risk | Addressed | Phase 2 risk acknowledgment section + Key Decisions Table. 2-week spike recommended. |
| 70 | Incremental vs simultaneous replacement | Partially | Three-phase approach is incremental. But Phase 1 replaces perception + architecture simultaneously. |

### P2 Questions — Detailed Status

| # | Question | Status | Notes |
|---|----------|--------|-------|
| 71 | Battery consumption change | Missing | No power/thermal analysis. |
| 72 | Noise from compute | Missing | Not mentioned. |
| 73 | Individual lidar feeds | Partially | Error Handling > Lidar Failure mentions single-lidar degradation. No UI access to individual feeds. |
| 74 | Dynamic obstacle handling | Addressed | CMU base_autonomy handles local obstacle avoidance. |
| 75 | Per-robot calibration | Missing | Single M20 assumed. No fleet calibration discussion. |
| 76 | Goal-setting interface changes | Addressed | Existing UI preserved ("no changes needed" for WebsocketVisModule). |
| 77 | First-time setup flow | Addressed | Section 1.1 deploy.sh setup subcommand. |
| 78 | FASTLIO2 mapping procedure | Partially | FASTLIO2 described but no user-facing mapping workflow (drive pattern, validation, save). |
| 79 | End of shift power-off | Partially | Container restart policy and SLAM state discussed. No explicit graceful vs abrupt shutdown guidance. |
| 80 | System update mechanism | Deferred | Deferred table: post-demo (deploy.sh manual deployment). |
| 81 | Problem reporting info | Missing | No log bundle or diagnostic capture mechanism described. |
| 82 | Featureless area behavior | Addressed | Error Handling > SLAM Divergence: featureless environments listed as cause. |
| 83 | Glass/reflective surfaces | Addressed | Q64 answer + SLAM Divergence error handling. |
| 84 | Goal on unmapped floor | Partially | Phase 3 map layer architecture described but no explicit error message for unmapped floor. |
| 85 | WiFi loss during mission | Addressed | Error Handling > Network Loss: robot continues autonomously, operator reconnects. |
| 86 | Multiple users conflicting goals | Deferred | Deferred table: Houdini epic. |
| 87 | Colorblind accessibility | Missing | Not mentioned. |
| 88 | Plain language logs | Missing | Not mentioned. |
| 89 | Training materials | Deferred | Deferred table: post-demo. |
| 90 | Network infrastructure assumptions | Partially | DDS, LCM multicast, host networking required. No explicit WiFi/IT policy requirements. |
| 91 | Air-gapped deployment | Missing | deploy.sh pulls from ghcr.io. No offline deployment path. |
| 92 | SLAM pose vs raw odom distinction | Missing | Not mentioned as a UI concern. |
| 93 | Floor as first-class UI concept | Deferred | Phase 3 map layer architecture addresses this architecturally. UI deferred. |
| 94 | Container lifecycle in UI | Missing | Not specified. |
| 95 | Information density by role | Deferred | Deferred table: Houdini epic. |
| 96 | Velocity display format | Missing | Not specified. |
| 97 | Degraded mode communication | Partially | SLAM degraded state defined. No general degraded-mode framework (e.g., nav working but no camera). |
| 98 | "No map yet" cold start state | Missing | Not specified. |
| 99 | Waypoint structure | Missing | No waypoint queue or multi-stop specification. |
| 100 | Required vs optional goal inputs | Missing | Not specified. |
| 101 | Dynamic no-go zones | Missing | Not specified. |
| 102 | Operator path feedback | Missing | Not specified. |
| 103 | Patrol route setup | Deferred | Deferred table: post-Phase 1. |
| 104 | Initial mapping session flow | Partially | FASTLIO2 described technically. No user-facing mapping workflow. |
| 105 | Lidar stale topic UI | Partially | Error Handling > Lidar Failure covers detection/recovery. No explicit UI specification. |
| 106 | Post-goal-arrival behavior | Missing | Not specified (stop, await, proceed?). |
| 107 | Planned vs traveled path visual | Missing | Not specified. |
| 108 | Dark mode | Missing | Not mentioned. |
| 109 | Map zoom behavior | Missing | Not specified. |
| 110 | Waiting for lidar vs no SLAM pose | Partially | Boot sequence partially described. Not as distinct UI states. |
| 111 | Physically stuck state | Missing | Not defined as a distinct state. |
| 112 | Goal unreachable state | Missing | Not defined. ROSNav has goal_reached but no unreachable signal. |
| 113 | Shutdown flow | Partially | deploy.sh stop described (1.7) but no user-facing shutdown sequence. |
| 114 | NOS loses AOS connectivity | Addressed | Error Handling > Network Loss. |
| 115 | Operational map definition | Partially | Phase 1: live SLAM. Phase 2: persistent maps. No explicit "what is a map" user-facing definition. |
| 116 | base_autonomy vs dimos boundary | Addressed | Architecture overview + data flow clearly separate CMU container (planning/avoidance) from host dimos (mapping/vis/coordination). |
| 117 | FAR planner vs dimos CostMapper authority | Partially | Both produce costmaps. Spec says CostMapper feeds host visualization. FAR planner governs actual navigation. But no explicit "which is authoritative for collision avoidance?" statement. |
| 118 | drdds + ROS2 DDS interop | Addressed | Section on drdds Interoperability. |

### P3 Questions — Detailed Status

| # | Question | Status | Notes |
|---|----------|--------|-------|
| 119 | DDS topic collisions | Addressed | ROS_DOMAIN_ID=42 isolation. |
| 120 | Container OOM | Addressed | Error Handling > OOM on NOS. docker_memory: "1.5g". |
| 121 | Navigation while docked | Missing | |
| 122 | Incremental floor mapping | Addressed | Phase 2 multi-session mapping. |
| 123 | Limited mobility teleop | Missing | |
| 124 | Responsive UI (phone/tablet) | Missing | |
| 125 | Physical NOS access assumption | Missing | |
| 126 | M20 hardware familiarity assumption | Partially | End user is engineering team (implied familiarity). |
| 127 | Blocking credential prompts | Missing | |
| 128 | Role-based permissions | Deferred | Deferred table: Houdini epic. |
| 129 | Post-success feedback | Missing | |
| 130 | Deployment stage labeling | Missing | |
| 131 | Run ownership context | Missing | |
| 132 | State transition audit trail | Missing | |
| 133 | SSH sudo password via UI | Missing | |
| 134 | Module grouping visual | Missing | |
| 135 | Service conflict resolution info | Partially | deploy.sh auto-disables conflicts. |
| 136 | 3D map interactions | Missing | |
| 137 | Resource usage display | Partially | OOM warning at >80% mentioned. No display design. |
| 138 | RPC call visualization | Missing | |
| 139 | Secondary telemetry display | Missing | |
| 140 | Host vs container color coding | Missing | |
| 141 | Dual lidar merge iconography | Missing | |
| 142 | Robot pose centering | Missing | |
| 143 | Height cost display | Partially | CostMapper described technically. |
| 144 | Clock drift/sync | Missing | |
| 145 | EMI in battery facility | Missing | |
| 146 | Multi-robot lidar interference | Missing | |
| 147 | Payload dynamics between floors | Missing | |

---

## Assessment 2: Fresh Engineering Assessment

| Category | Status | Notes |
|----------|--------|-------|
| Objective | **Clear** | The migration from monolithic Docker to host+container ROSNav pattern is well-defined. Three phases with distinct goals. Two engineers would build the same Phase 1 architecture. Phase 2 and 3 are deliberately looser (appropriate for research-adjacent work). |
| Done Criteria | **Minor gap** | Phase 2 has concrete validation criteria (relocalization <10s, loop closure >100m, map round-trip). Phase 1 lacks explicit done criteria beyond "working single-floor autonomy." No acceptance test procedure for Phase 1 (e.g., "navigate 50m to goal within 2m accuracy, 5 consecutive runs without intervention"). Phase 3 has one: "floor A to floor B via staircase." |
| Scope | **Clear** | Files/components in scope enumerated in Key Components table with phase assignment. New files, modified files, and unchanged files clearly marked. Out of Scope table is comprehensive. |
| Constraints | **Minor gap** | Hardware constraints well-documented (NOS RAM, Docker storage, SSH quirks). Missing: velocity/speed limits for the new stack, latency budgets (goal-to-motion time, SLAM processing time), and CPU budget (will FASTLIO2 + FAR planner + base_autonomy fit in 1.5GB Docker + 2GB host?). |
| Dependencies | **Clear** | External dependencies identified: FASTLIO2 RoboSense fork, CMU nav stack, drdds-ros2-msgs rebuild. Integration points with AOS/GOS documented. Open Questions 1-6 correctly flag unresolved technical dependencies. |
| Safety | **Minor gap** | Container crash safety covered (watchdog, auto-restart). OOM protection covered. No explicit rollback plan (dismissed in Out of Scope, but "deploy.sh can redeploy old image" is vague). No data migration needed (clean break). Missing: what happens if the new stack is worse than the old stack mid-pilot? The old monolithic Docker is gone. |

### Detailed Engineering Findings

**Strengths:**

1. **Architecture is well-grounded.** The host+container split is clearly motivated, follows the proven G1 pattern, and the data flow (7 steps) is traceable end-to-end.

2. **Error handling is thorough.** Container crash, SLAM divergence, lidar failure, network loss, OOM, and stair edge cases are all addressed with detection, recovery, and user feedback.

3. **Risk is honestly assessed.** Phase 2 arise_slam porting is called out as potentially research-level. The 2-week spike before committing is the right approach.

4. **Codebase specificity.** The spec references exact file paths, line counts, config values, and IP addresses. This is not a hand-wavy architecture doc; it is implementable.

5. **Open Questions are the right questions.** The 11 open questions in the spec are genuinely unresolved technical issues, not scope ambiguity.

**Gaps:**

1. **No Phase 1 acceptance test.** "Working single-floor autonomy" is ambiguous. Needs: distance to goal accuracy, minimum traversal distance without drift, number of consecutive runs without intervention, specific test environment.

2. **Velocity command translation is under-specified.** The Twist-to-NavCmd conversion (Section 1.3) is described at concept level but not at implementation level. NavCmd has forward velocity, lateral velocity, and yaw rate. Twist has linear.x, linear.y, angular.z. The mapping seems straightforward but: are there coordinate frame differences? Is there a scale factor? What are the velocity limits? What happens to the velocity watchdog timing?

3. **No performance budget.** FASTLIO2 + FAR planner + base_autonomy in a 1.5GB Docker container on an RK3588 is ambitious. The spec does not estimate per-component CPU/RAM usage. The G1 uses a more powerful compute platform; the NOS is significantly more constrained.

4. **DDS cross-domain communication is hand-waved.** The container uses ROS_DOMAIN_ID=42 while rsdriver on AOS uses domain 0. The spec says the container subscribes to `/lidar_points` from rsdriver. But if they are on different DDS domains, discovery will not work. Either the container must use domain 0 for input topics (creating the collision risk it tries to avoid) or rsdriver must multi-publish, or a DDS bridge is needed. Open Question 6 partially acknowledges this but does not flag the domain ID conflict.

5. **No timeline estimates.** The spec has phases and gates but no duration estimates. Phase 1 could take 2 weeks or 2 months. This matters for factory pilot scheduling.

6. **Concurrent rsdriver instances (Open Question 6) is more serious than stated.** Both AOS and GOS publish identical `/lidar_points`. If FASTLIO2 subscribes and receives both, it gets duplicate point clouds at double rate. This could cause SLAM degradation or doubled compute load. This needs resolution before implementation starts, not "during implementation."

7. **No integration test plan.** The spec describes unit-level validation (SLAM convergence, loop closure) but no integration test plan for the full pipeline (lidar -> SLAM -> planner -> bridge -> host -> velocity -> robot movement -> verify arrival at goal).

---

## Assessment 3: Prioritized Gaps

### Critical (must resolve before implementation)

| # | Gap | Source | Impact |
|---|-----|--------|--------|
| C1 | **No Phase 1 acceptance criteria.** "Working single-floor autonomy" is not testable. Define: goal accuracy (meters), minimum traversal distance, consecutive success runs, test environment. | Fresh engineering (Done Criteria) + Q14 partially addressed | Cannot determine when Phase 1 is "done." Risk of scope creep or premature declaration of success. |
| C2 | **DDS domain ID conflict.** Container on domain 42 cannot discover rsdriver on domain 0. The spec's own architecture requires cross-domain DDS topic access. This is not acknowledged as a conflict. | Fresh engineering + Open Question 6 | Fundamental blocker: FASTLIO2 cannot receive lidar data without resolving this. |
| C3 | **Performance budget for NOS.** No estimate of CPU/RAM for FASTLIO2 + FAR planner + base_autonomy in 1.5GB container on RK3588 (4 cores, 2GB usable). G1 has much more headroom. | Fresh engineering (Constraints) | Risk of OOM or unacceptable SLAM latency on the target hardware. Should be validated in a spike before committing to the architecture. |
| C4 | **Concurrent rsdriver instances (AOS + GOS both publishing /lidar_points).** Must resolve before implementation: duplicate subscription causes doubled compute and potential SLAM issues. | Open Question 6, elevated from "resolve before implementation" to critical | SLAM correctness depends on this. |
| C5 | **Velocity command translation specifics.** Twist-to-NavCmd coordinate frames, scale factors, velocity limits, and watchdog timing are unspecified. Incorrect translation means the robot drives into walls. | Fresh engineering + Q29 (missing) | Safety-critical: wrong velocity mapping causes physical damage. |

### Important (should resolve before or during Phase 1)

| # | Gap | Source | Impact |
|---|-----|--------|--------|
| I1 | **Teleop/autonomy mode switching UX.** Spec does not define how operators switch between teleop and autonomous mode, or how dual-control is prevented. | Q6 (partial), Q49 (partial) | Operator confusion. Two velocity sources (UDP teleop + NAV_CMD) could conflict if both are active. |
| I2 | **Boot-to-ready timing.** No estimates for host Python + Docker pull + FASTLIO2 convergence. Operators need to know how long to wait. | Q32 (missing), Q50 (missing) | Poor operator experience if boot takes 5+ minutes with no progress indicator. |
| I3 | **Single composite readiness indicator.** Multiple subsystems must all be healthy before navigation. No aggregated "READY" signal defined. | Q1 (partial) | Operators may attempt navigation before system is ready, causing confusing failures. |
| I4 | **Goal conflict policy.** What happens when a new goal arrives while navigating to a previous goal? Replace, queue, or reject? | Q36 (partial) | Unpredictable behavior erodes operator trust. |
| I5 | **Speed/acceleration comparison.** No documentation of how the new stack's motion profile compares to the old system. | Q29 (missing) | Operators may perceive regression if the robot moves differently. |
| I6 | **Recovery without SSH.** All recovery actions assume engineering team with terminal access. No UI-accessible recovery buttons. | Q34 (missing), Q47 (missing) | Acceptable for Phase 1 (engineering team is the user), but should be documented as a known limitation. |
| I7 | **Costmap authority ambiguity.** Host CostMapper and container FAR planner both produce costmaps. Which governs collision avoidance? | Q117 (partial) | Could lead to the robot trusting one costmap while the other shows a collision. |
| I8 | **Integration test plan.** No end-to-end test procedure from lidar to goal arrival. | Fresh engineering | Cannot validate the full pipeline works without a defined test. |
| I9 | **No timeline estimates.** Phases have gates but no durations. | Fresh engineering | Cannot schedule factory pilot without knowing Phase 1 duration. |
| I10 | **Safety baseline.** No safety metrics defined. No regression criteria. | Q66 (missing), Q67 (missing) | Cannot prove the new system is as safe as the old one. |
| I11 | **FASTLIO2 mapping workflow.** Technical SLAM is described but not the user-facing procedure for building a first map (drive pattern, validation, save). | Q78 (partial), Q104 (partial) | Operators will not know how to create their first map. |

### Nice to Have (polish items, can defer)

| # | Gap | Source | Impact |
|---|-----|--------|--------|
| N1 | Default map layers in viewer (Q45) | P1 question | Minor UX decision, defaults can be adjusted. |
| N2 | Colorblind accessibility (Q87) | P2 question | Small user base during pilot (engineering team). |
| N3 | Dark mode (Q108) | P2 question | Aesthetic preference. |
| N4 | Plain language error messages (Q88) | P2 question | Engineering team can interpret technical messages. |
| N5 | Planned vs traveled path visualization (Q107) | P2 question | Debug convenience. |
| N6 | Post-goal-arrival behavior (Q106) | P2 question | Can default to "stop and wait." |
| N7 | Goal unreachable state (Q112) | P2 question | Can rely on timeout for now. |
| N8 | Physically stuck detection (Q111) | P2 question | Operators will notice visually. |
| N9 | Battery/power impact (Q71) | P2 question | Measure empirically. |
| N10 | Audible state indicators (Q41) | P1 question | Hardware-dependent, can add later. |
| N11 | Air-gapped deployment path (Q91) | P2 question | Not needed for pilot. |
| N12 | Clock drift between boards (Q144) | P3 question | DDS handles timestamps. Monitor empirically. |
| N13 | Dynamic no-go zones (Q101) | P2 question | CMU stack does dynamic avoidance. Static zones are Phase 2+. |
| N14 | Patrol route management (Q103) | Deferred | Requires basic nav first. |
| N15 | Operator path quality feedback (Q102) | P2 question | Engineering team provides verbal feedback. |

---

## Summary

The spec is strong for its primary purpose: enabling an engineer to implement the Phase 1 host+container architectural migration. Architecture, data flow, error handling, and codebase specificity are all well above average for a robotics spec.

The five critical gaps (C1-C5) must be resolved before implementation begins. C2 (DDS domain conflict) and C4 (duplicate rsdriver) are potential architectural blockers that could invalidate the proposed data flow. C3 (performance budget) could render the entire approach infeasible on NOS hardware. C5 (velocity translation) is safety-critical.

The important gaps (I1-I11) are mostly UX and operational concerns that are acceptable for an engineering-team-only pilot but would block a broader deployment. They should be resolved during Phase 1 implementation.

P2 and P3 questions are predominantly UX/UI polish items that are correctly deprioritized given the current end user (engineering team, not factory operators).

**Overall assessment: The spec is ~80% ready for Phase 1 implementation. Resolve C1-C5 first.**
