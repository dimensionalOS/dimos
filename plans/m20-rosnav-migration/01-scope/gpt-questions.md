# GPT 5.4 Analysis: m20-rosnav-migration

## User Advocate Perspective

Assumption: the daily user is an onsite operator, supervisor, or support person who cares about mission success, not the architecture change behind it.

**User Expectations**
- Will users assume the upgrade keeps their existing daily routine intact? It matters because even a better system feels broken if familiar start, send, stop, or recovery steps change.
- Will users expect noticeably fewer "robot got lost" moments from day one? It matters because users judge upgrades by reliability, not by cleaner internals.
- Will users assume startup is simpler, not something they have to babysit? It matters because daily operators expect one clear ready state.
- Will users expect the same destinations, maps, and mission names to still make sense? It matters because renaming or remapping creates immediate confusion.
- Will users assume the robot clearly tells them when it is ready for the first mission? It matters because vague readiness leads to premature launches and blame.
- Will users expect faster recovery after reboots, pauses, or interruptions? It matters because downtime is what users remember.
- Will users assume the robot explains why it cannot do something, instead of just stopping? It matters because explicit limits preserve trust better than silent failure.
- Will users hear "multi-floor" and assume seamless floor-to-floor use now, not later? It matters because phased delivery is easy for teams to understand but easy for operators to miss.
- What existing experience are users comparing this to: the previous M20 behavior, other AMRs, elevator-capable robots, or app-style navigation? It matters because those comparisons define their success bar.
- Will users assume the upgrade works on the same device with no extra daily care? It matters because needing special rituals makes the feature feel fragile.

**User Journey**
- What job are they trying to finish when they reach for this feature: patrol, inspection, delivery, or cross-floor handoff? It matters because "successful navigation" means different things in each workflow.
- What happened immediately before use: start of shift, overnight charging, a manual move, or a handoff from another operator? It matters because context shapes patience and expectations.
- How rushed or stressed are they when they start the robot? It matters because anything fiddly will be skipped under time pressure.
- What single signal tells them "safe to send the first mission"? It matters because daily users need a simple go or no-go decision.
- Where do they look for reassurance while the robot is moving? It matters because users will not read logs during live operations.
- At what point are they most anxious: first start, first route, first floor change, or first recovery? It matters because that is where UX needs the strongest confidence cues.
- If the robot hesitates or seems uncertain, what is the user's first instinct? It matters because the recovery path should match real behavior, not ideal training.
- What information do they need before they will trust the robot again after one bad run? It matters because trust recovery is usually harder than first-time adoption.
- What counts as a clean finish from their point of view: reached destination, completed task, returned home, or left a record? It matters because completion criteria drive satisfaction.
- What happens right after the mission ends: next task, shift handoff, charging, or manual relocation? It matters because the feature has to fit the whole work loop, not just the route.

**Edge Cases (User Behavior)**
- What happens if a user sends a mission before the robot is actually ready? It matters because impatient starts are normal in production.
- What happens if the user changes destination halfway through a run? It matters because plans change constantly on the floor.
- What happens if the user sends the robot to the wrong place or wrong floor by mistake? It matters because correction should be safer and easier than the original mistake.
- What happens if someone manually pushes or relocates the robot between tasks? It matters because people will intervene physically when they think they are helping.
- What happens if the robot waits a long time and the user expects it to resume without extra setup? It matters because paused work is common.
- What happens if the user reboots or power-cycles during a busy shift and expects a quick return to service? It matters because uptime pressure changes behavior.
- What happens if two people try to direct the robot around the same time? It matters because shared ownership creates conflicting intent and blame.
- What happens if the user keeps using habits from the old system? It matters because muscle memory is stronger than documentation.
- What happens if users get overconfident after a few good runs and start sending longer or riskier missions? It matters because success often expands usage beyond the original brief.
- What happens if a user abandons a task halfway and comes back later? It matters because real work is interrupt-driven, not neatly sequential.

**Accessibility & Inclusion**
- Who struggles if daily use assumes comfort with command-line tools or specialist vocabulary? It matters because operators are not always the people who set the system up.
- Who struggles if system state is only obvious to people who already know the robot's quirks? It matters because new hires and temporary staff need equal clarity.
- Who struggles if status or alerts depend only on color, sound, or tiny text? It matters because factories are noisy, distracting, and not everyone perceives cues the same way.
- Who struggles if instructions assume fluent English or one shared set of terms? It matters because mixed-language teams are common in industrial sites.
- Who struggles if recovery steps must be remembered from training instead of guided in the moment? It matters because failures happen under stress, not under classroom conditions.
- Who struggles if users need to watch a screen continuously to feel safe? It matters because many operators are walking, multitasking, or wearing gloves.
- Who struggles if the system assumes one consistent "owner" of the robot per shift? It matters because shared responsibility is normal in real operations.
- Who struggles if behavior differs subtly across floors or areas without plain-language explanation? It matters because inconsistency punishes novices first.
- Who struggles if the feature only works well for confident early adopters and not cautious operators? It matters because pilot success needs broad adoption, not just expert champions.
- Who struggles if getting help depends on knowing the right internal engineer? It matters because inclusive operations require recoverability without insider access.

If useful, I can turn these into a prioritized acceptance-review checklist or a stakeholder interview guide.

## Product Designer Perspective

Based on [context.md](/Users/afik_cohen/gt/dimos/crew/ace/plans/m20-rosnav-migration/01-scope/context.md), here are wireframe-oriented product questions to resolve before designing the M20 migration experience. I kept these non-technical and focused on operator UX.

**Information Architecture**
1. Who is the primary user for this surface: factory operator, robotics technician, remote support, or site manager? Why it matters: the information density, terminology, and control safety should change based on user expertise.
2. What is the single most important thing the user must know at a glance: "robot ready," "robot navigating," "robot lost," or "pilot blocked"? Why it matters: this defines the topmost status area and the visual hierarchy.
3. What information must always stay visible during operation? Why it matters: persistent items like robot readiness, current floor, destination, and active alerts should not require digging.
4. What information is only needed during setup or troubleshooting and can be hidden by default? Why it matters: progressive disclosure keeps the live view usable under pressure.
5. Should the UI separate "mission status" from "system health," or combine them into one summary? Why it matters: operators need to distinguish "the task is blocked" from "the robot is unhealthy."
6. Does the user need continuous floor context even in Phase 1, or only once multi-floor behavior arrives later? Why it matters: showing future-oriented structure too early can either prepare users or confuse them.
7. How much historical context does the user need: current snapshot only, recent timeline, or shift-long history? Why it matters: this changes whether the product needs a timeline, activity feed, or just live state.
8. Which alerts deserve interruptive treatment versus passive visibility? Why it matters: too many urgent banners will train users to ignore the real problems.
9. Does the user need to see what the robot will do next, not just what it is doing now? Why it matters: previews build trust, especially in a factory setting where movement has operational consequences.
10. Should the interface show deployment-stage context like "Phase 1 pilot" versus "full multi-floor mode"? Why it matters: users need a clear mental model of what the current product version can and cannot do.
11. Do users need ownership context such as "who started this run" or "who last acknowledged an issue"? Why it matters: handoffs are common in operations environments.
12. Is the main object in the UI the robot, the mission, the floor, or the site? Why it matters: that choice determines the navigation structure of the product.

**Interaction Design**
1. How does the user enter this flow: from a robot detail page, a mission planner, a site dashboard, or a startup checklist? Why it matters: entry point determines what context can be assumed.
2. What are the minimum required inputs to begin a navigation session? Why it matters: fewer required decisions means faster, safer startup.
3. Which inputs should be optional or prefilled from previous usage? Why it matters: defaults reduce friction and operator error.
4. Should startup feel like a guided checklist, a stepper, or a single primary action with validation? Why it matters: different patterns change speed, confidence, and training burden.
5. How does a user choose where the robot should go: search, map tap, saved location, or task template? Why it matters: destination entry is likely the highest-frequency action.
6. What actions are risky enough to require confirmation? Why it matters: stop, cancel, relocalize, or restart behaviors may need friction to prevent accidental disruption.
7. How should the UI communicate "working" versus "ready" during startup and mission execution? Why it matters: users need confidence without misreading a background process as success.
8. What manual controls must remain accessible when the system is uncertain? Why it matters: automation only builds trust if recovery actions are obvious.
9. How should retry work after a failure: inline retry, guided recovery, or return to start? Why it matters: the wrong recovery pattern creates repeated failure loops.
10. What level of permissions should different roles have? Why it matters: an operator may need "pause" while a supervisor gets "reconfigure" or "override."
11. How should the system ask for missing information without feeling blocking or bureaucratic? Why it matters: the product should gather necessary inputs while preserving flow.
12. What feedback should appear after success: silent completion, toast, summary card, or next-step suggestion? Why it matters: success needs closure, especially in repetitive workflows.

**User Flows**
1. What is the happy path for a first-time user on a new shift? Why it matters: this defines onboarding, empty states, and whether the UI assumes prior context.
2. What is the happy path for a repeat user who just wants to start a known routine quickly? Why it matters: daily users need speed more than explanation.
3. What is the step-by-step flow for sending the robot to a destination on the same floor? Why it matters: this is likely the simplest mission and should feel effortless.
4. What is the future step-by-step flow for sending the robot across floors? Why it matters: even if multi-floor comes later, the information model should not need a redesign.
5. What should happen if the user starts a mission before the robot is truly ready? Why it matters: premature actions are common in high-pressure environments.
6. What is the recovery flow if the robot becomes uncertain about its position? Why it matters: this is a trust-critical moment where users need clear choices, not jargon.
7. What is the recovery flow if the robot cannot proceed because the path is blocked or the destination is unavailable? Why it matters: users need actionable alternatives, not a dead-end error.
8. How does pause, resume, and cancel work mid-mission? Why it matters: operators need predictable interruption behavior during real-world disruptions.
9. What is the handoff flow when one person leaves and another takes over? Why it matters: factory operations often span shifts and roles.
10. What is the empty-state flow when no robot is connected, no mission exists, or no floor context is available? Why it matters: blank screens are disorienting and expensive in support time.
11. What is the overloaded-state flow when multiple warnings appear at once? Why it matters: the UI must still point to the single best next action.
12. What is the pilot-validation flow for proving the system is ready for the CATL trial? Why it matters: product success is not just mission execution, but operator confidence that the system is trial-ready.

**Visual & Layout**
1. Does this belong in an existing operations surface or as its own dedicated screen? Why it matters: new surfaces increase navigation cost and training overhead.
2. If it lives in an existing UI, where should it sit in the hierarchy: robot page, mission page, or site page? Why it matters: placement should match the user's mental model.
3. What deserves the largest area on the screen: map, status summary, action panel, or alert stack? Why it matters: size signals importance.
4. Should setup and live operation share one screen or split into separate modes? Why it matters: one screen is faster, but split modes can reduce clutter.
5. How should floor context be shown visually: tabs, stacked cards, floor selector, or breadcrumb? Why it matters: multi-floor mental models can become confusing quickly.
6. How much detail should be visible by default versus tucked behind "details" or drawers? Why it matters: the main view should remain glanceable in a busy factory environment.
7. What existing visual patterns from current dimos tooling should be preserved? Why it matters: consistency reduces relearning during a migration.
8. Should alerts appear as banners, a side rail, a sticky footer, or inline within the relevant panel? Why it matters: placement affects whether users act immediately or miss issues.
9. What layout works best for both control-room desktop use and on-the-go tablet viewing? Why it matters: the same product may be used in different physical contexts.
10. How should the interface visually distinguish stable state from uncertain state? Why it matters: subtle uncertainty is easy to miss unless the visual language is explicit.
11. What visual treatment makes critical actions obvious without making the whole UI feel alarm-heavy? Why it matters: constant red erodes urgency.
12. Should the product emphasize "mission progress" or "robot health" visually? Why it matters: whichever gets the prime layout slot becomes the operator's default lens.

**States & Transitions**
1. What are the top-level user-visible states the product must support? Why it matters: wireframes should be built around state models, not just happy-path screens.
2. Which states need dedicated full-screen treatment versus lightweight inline status? Why it matters: not every condition deserves the same amount of interruption.
3. How should the UI show transition states like "starting," "becoming ready," or "recovering"? Why it matters: users need to know the system is active, not frozen.
4. What separates a temporary uncertainty from a hard failure in the visual language? Why it matters: users make very different decisions in those two cases.
5. How long can a transition last before the UI should call it out as stuck? Why it matters: time thresholds shape trust and escalation behavior.
6. What should happen visually when the robot moves from one floor context to another? Why it matters: abrupt context jumps can break user orientation.
7. How should success be represented: "arrived," "ready for next task," or "mission complete with summary"? Why it matters: completion states influence whether users confidently move on.
8. What recovery states are automatic, and which require explicit user acknowledgment? Why it matters: users need to know when they are expected to act.
9. How should the UI behave when data is stale or temporarily missing? Why it matters: silent staleness is often worse than an explicit warning.
10. What state should the product return to after a cancelled or failed mission? Why it matters: reset behavior determines how quickly the next attempt can begin.
11. Should the system preserve user context across state changes, such as keeping the selected floor or destination visible after recovery? Why it matters: preserving context reduces cognitive reset cost.
12. What audit or confirmation trail is needed when key state transitions happen? Why it matters: operations teams often need accountability and post-incident clarity.

If you want, I can turn these into a tighter wireframe-ready artifact next: `screen list + primary panels + per-screen states`.

## Domain Expert Perspective

Using the codebase context in [context.md](/Users/afik_cohen/gt/dimos/crew/ace/plans/m20-rosnav-migration/01-scope/context.md), these are the domain questions I'd push before treating the brief as sufficiently scoped. They're intentionally non-technical.

**Domain Concepts**
1. What exactly counts as a "floor" at the CATL site? Why it matters: mezzanines, ramps, split levels, and separate buildings create very different operational expectations.
2. What real-world event is "multi-floor relocalization" meant to cover? Why it matters: elevator use, manual carry, restart, or mission resume imply different user needs.
3. Who is the day-to-day owner of the robot on site? Why it matters: facilities, production, EHS, and robotics teams optimize for different outcomes.
4. What job is the robot actually doing during the pilot? Why it matters: inspection, transport, patrol, and escort workflows tolerate different failure modes.
5. Which spaces are truly in scope for the pilot? Why it matters: production halls, warehouses, corridors, and loading areas have different traffic and safety norms.
6. Which spaces are explicitly out of bounds? Why it matters: restricted areas usually matter more operationally than nominal map coverage.
7. How do operators define "mission success" versus "recovery"? Why it matters: stakeholder language has to match what the system can honestly claim.
8. When site staff say the robot is "lost," what do they mean operationally? Why it matters: wrong floor, wrong aisle, stopped progress, and low confidence are different problems.
9. What human handoff is acceptable in the pilot? Why it matters: some pilots allow escorting or repositioning; others require near-full independence.
10. What does "factory pilot" mean in business terms? Why it matters: a demo, shadow run, and production-adjacent pilot require very different rigor.

**Prior Art**
1. How do comparable AMRs in factories handle multi-floor movement today? Why it matters: this sets a realistic benchmark for stakeholder expectations.
2. Do incumbent deployments treat floor changes as normal autonomous behavior or supervised exceptions? Why it matters: that distinction prevents overscoping.
3. What approaches has CATL or the customer already tried for cross-floor operation? Why it matters: repeated failed patterns destroy trust quickly.
4. Which prior navigation failures were most painful for operators? Why it matters: the brief should target remembered field pain, not abstract capability gaps.
5. What recovery workflow do operators already consider "normal"? Why it matters: replacing an accepted manual step may not be the highest-value improvement.
6. How do mature vendors communicate uncertainty or "I need help"? Why it matters: operator trust depends on conventions people already recognize.
7. What acceptance criteria are standard in similar factory pilots? Why it matters: avoids inventing a success bar the customer will not use.
8. What environmental conditions have historically broken other robots at this site? Why it matters: field history predicts rollout risk better than lab assumptions.
9. What promises have already been made to pilot stakeholders about autonomy level or timeline? Why it matters: hidden commitments create scope conflict later.
10. Which parts of prior rollouts created the most operator babysitting? Why it matters: reducing babysitting is often a stronger value proposition than adding features.

**Problem Depth**
1. Is the real problem floor-to-floor operation, or the fact that recovery currently needs expert intervention? Why it matters: symptom and root cause point to different scope decisions.
2. Is the biggest pain failed missions, slow recovery, or low operator confidence? Why it matters: only one of these is usually the true blocker.
3. How often do pilot missions actually require floor transitions? Why it matters: rare transitions may not justify early complexity.
4. If single-floor autonomy were highly reliable, would that already unlock the pilot? Why it matters: this tests whether multi-floor is essential or aspirational.
5. Are we solving for fewer failed missions, fewer human touches, or faster site commissioning? Why it matters: the success metric changes materially.
6. What adjacent problems will users assume are included once "multi-floor navigation" is promised? Why it matters: expectations often expand to elevators, doors, docking, and dispatch.
7. What is explicitly not being solved in Phases 1-3? Why it matters: boundaries prevent the pilot from turning into a platform rewrite.
8. What is the minimum operational outcome that would still justify deployment? Why it matters: this defines a credible first win.
9. If relocalization works but route execution still needs human approval, is that acceptable? Why it matters: it clarifies the target autonomy level.
10. What business consequence occurs today when the robot cannot resume after relocation? Why it matters: it quantifies the real cost of the gap.

**Edge Cases**
1. What should happen if staff manually move the robot between floors outside the intended workflow? Why it matters: this is common in real pilots.
2. How should the system behave in lookalike areas where multiple floors have similar layout? Why it matters: wrong-floor confidence is worse than low confidence.
3. What is the expected behavior during temporary layout changes like pallets, staging zones, or blocked aisles? Why it matters: factories are rarely static.
4. How should lifts, ramps, or freight elevators be treated operationally? Why it matters: a "floor change" is not always a clean event.
5. What happens during peak pedestrian or forklift traffic periods? Why it matters: a solution that only works off-shift may fail in reality.
6. What is the policy for restricted or time-gated areas across floors? Why it matters: legal access and operational access are not the same.
7. How should the robot recover after emergency stops, fire drills, or manual shutdowns mid-mission? Why it matters: abnormal recovery is heavily judged in pilots.
8. What happens when cleaning, dust, steam, or reflective surfaces temporarily change the environment? Why it matters: transient factory conditions often break neat assumptions.
9. Are there multilingual labeling, signage, or operator-training needs at the site? Why it matters: recovery workflows fail when terminology is not locally natural.
10. What site-specific safety, compliance, or customer approval steps govern autonomous movement across floors? Why it matters: an operationally disallowed workflow is still a failure.

**Success Criteria**
1. What percentage of in-scope missions must complete without human intervention to call the pilot successful? Why it matters: autonomy needs a clear bar.
2. After a floor transition or manual relocation, how quickly must the robot become operational again? Why it matters: recovery time is user-visible value.
3. What maximum rate of wrong-floor or wrong-location recovery is acceptable? Why it matters: rare but serious mislocalization can sink trust.
4. How many operator interventions per shift or per day are acceptable? Why it matters: this directly measures babysitting.
5. Which recovery actions must be doable by non-expert site staff? Why it matters: pilot scalability depends on who can unblock it.
6. What uptime across shifts, days, or weeks is required before the customer considers the system stable? Why it matters: demos and pilots use different stability bars.
7. What safety indicators must improve or at least not regress? Why it matters: no factory pilot survives if safety perception worsens.
8. What task-level benefit must the pilot prove for the end user? Why it matters: coverage, throughput, response time, or inspection consistency drive expansion decisions.
9. What operator-confidence signals will count as success? Why it matters: qualitative trust often decides rollout before hard ROI is mature.
10. What evidence should trigger Phase 2-to-3 progression or broader rollout? Why it matters: staged programs need explicit go/no-go gates.

If you want, I can turn these into a stakeholder interview checklist or rank them by what should be answered before Phase 1 versus before Phase 3.

## Cross-Perspective Themes (GPT)

### 1. **Operator Readiness & Confidence Signals**
All three perspectives emphasize clarity over abstraction. The User Advocate asks whether users will know when the robot is ready; the Product Designer questions how the UI communicates "working" versus "ready"; the Domain Expert queries what single operational signal tells staff the system is safe to engage. This convergence suggests the deliverable must provide unambiguous, glanceable proof of readiness before operators send the first mission.

### 2. **Recovery as First-Class UX, Not an Afterthought**
User expectations focus on faster recovery after reboots and clear error explanations. The Product Designer details flows for position uncertainty and blocked paths. The Domain Expert questions the acceptability of different recovery workflows and whether relocalization alone would unlock the pilot. Together, they signal that recovery paths must be as designed and rehearsed as happy paths—and may be the true value proposition if they reduce babysitting more than new features add capability.

### 3. **Factory Context Changes Everything**
The three perspectives align on the primacy of real-world constraints. User behavior includes shared robot ownership and manual physical intervention. Product design must account for gloves, noise, glanceable status, and interruption handling. Domain expertise warns that factories are rarely static and that site-specific edge cases (blocked aisles, traffic, dust, restricted areas) override lab assumptions. This unified view suggests scope cannot be finalized without site-resident pilot data.

### 4. **Phased Delivery Creates Expectation Mismatch Risk**
The User Advocate notes operators may assume multi-floor "now, not later." The Product Designer questions whether to show Phase 1 versus full-mode framing. The Domain Expert asks whether multi-floor is essential or aspirational and what minimum outcome justifies Phase 1 deployment. All three highlight a critical gap: phased rollouts must be explicitly communicated, or operators will form false expectations during the pilot that become adoption blockers.

### 5. **Stakeholder Language & Success Definitions Must Align Before Launch**
The Product Designer asks how to define pilot success operationally. The Domain Expert emphasizes that "lost" and "mission success" mean different things to different teams (facilities vs. production vs. robotics) and that acceptance criteria must match customer norms, not invented standards. The User Advocate warns that users judge upgrades by remembered pain, not technical metrics. This suggests a pre-pilot acceptance checklist co-authored with site stakeholders is non-negotiable.
