# Agent safety layer (DASL)

DimOS agents call `@skill` functions over MCP and those calls move real hardware. Today the only safety mechanism is a paragraph in the system prompt. We want a deterministic layer in the dispatch path that the model cannot talk its way around — model-level alignment stays as the outermost defense, not the only one.

## what a skill author writes

```python
from dimos.agents.safety import RiskLevel, ConfirmMode

@skill(risk=RiskLevel.DYNAMIC, uses=(CAP_MOVEMENT,),
       bounds={"x": (-0.5, 0.5), "duration": (0.0, 5.0)},
       rate_limit="2/min", confirm=ConfirmMode.OPERATOR)
def move(self, x: float, duration: float = 2.0) -> str:
    """Move the robot forward or backward. ..."""
```

defaults are `risk=READONLY`, everything else off — existing skills keep working unchanged and get the safest tier for free. Physical skills opt into higher tiers explicitly, where reviewers can see it.

## the gate

sits in `_handle_tools_call`, ahead of the existing `CapabilityRegistry.acquire`. synchronous, sub-ms, no model in the loop:

```python
def check(skill, args, ctx) -> PolicyDecision:
    if mode == "off":                        return ALLOW      # still audited
    if skill.risk == READONLY:               return ALLOW
    if ctx.trust < MIN_TRUST[skill.risk]:    return DENY("trust insufficient")
    for p, (lo, hi) in merged_bounds(skill, policy_overrides):
        if p in args and not lo <= args[p] <= hi:
            return DENY(f"'{p}={args[p]}' outside [{lo}, {hi}]")   # reject, never clamp
    if rate_limiter.hit(skill.name, skill.rate_limit):
        return DENY("rate limit exceeded")                        # denied calls don't burn quota
    if skill.confirm == OPERATOR and not ctx.consume_confirmation():
        return NEEDS_CONFIRMATION(registry.register(skill, args))   # dimos mcp confirm <id>
    return ALLOW
```

`mode: shadow` rewrites any deny into allow + `[shadow-would-deny]` audit record, so we can calibrate thresholds on replay data before enforcing. any exception inside `check` becomes a deny — a broken safety layer fails toward refusal.

## where trust comes from

every message entering the agent context gets tagged by code (never self-declared by the model): CLI/teleop = `OPERATOR`, speech = `USER_VOICE`, tool results = `TOOL_RESULT`. chain trust = min over the chain; an untagged message degrades the chain, never upgrades it. the client puts it in `_meta["dimos/trust"]`, the gate compares against `MIN_TRUST`. this is what stops indirect injection: an instruction smuggled inside a sensor reading can never trigger a DYNAMIC skill, no matter what the LLM decides.

## runtime monitor (independent module, no LLM)

subscribes `Detection3DModule.detection_stream_3d` (real pointcloud depth — not `PersonTracker`, which assumes depth=2.0m and only tracks the largest bbox) plus costmap as a non-ML redundant trigger:

```python
on_frame(detections, costmap):
    d = min_distance_to_person(detections)     
    if d < HALT_M or costmap.nearest < COSTMAP_HALT_M:
        state = HALT; seize(CAP_MOVEMENT); zero_velocity()     # bypasses agent path entirely
    elif d < CAUTION_M:
        state = CAUTION; gate.set_dynamic_factor(0.3)          # motion bounds tighten via L2
    elif clear_for(CLEAR_DELAY_S):
        state = CLEAR; release(CAP_MOVEMENT)                   # auto-resume, like ER2 but deterministic

watchdog:  # silent perception failure must not look like "nobody there"
    if no_frame_for(500ms):  state = CAUTION
    if no_frame_for(2000ms): state = HALT
```

thresholds have hysteresis (enter CAUTION at 2.0m, leave at 2.3m) so the robot doesn't oscillate when someone stands at the boundary.

## execution verification

skills return structured `SkillResult{status, code, detail, telemetry}`; motion skills declare a `verify` predicate checked against real telemetry after execution (e.g. `move`: odom displacement >= 60% of commanded). failures feed back two ways: the result carries the numbers so the agent can reason ("commanded 1.00m, actual 0.12m" -> probably blocked), and 2 consecutive failures put the skill on a 60s cooldown at the gate. optional async VLM recheck on failures, but it can only downgrade FAILED to PARTIAL, never upgrade to SUCCESS.

## audit

every decision (allow/deny/confirm/monitor transition/verify result) is one JSONL record in a sha256 hash-chained `audit.jsonl` per run, `dimos log --audit --verify` to check integrity. incident review becomes "replay the chain", not "grep the logs".

## open questions

- headless deployments (daemon, no teleop UI): confirmation timeout = deny, or degrade to delayed execution? leaning deny.
- `_meta["dimos/trust"]` defaults to OPERATOR for legacy clients until L1 ships everywhere — when do we flip the default to UNTRUSTED?
- multi-robot: two robots see each other as costmap obstacles, which covers collision — but do capabilities/geofences need a cross-robot arbiter? separate proposal, probably.
- watchdog means a covered camera halts the robot. we think that's correct behavior, but it will surprise users — needs a doc page, not just a log line.

## prior art

- CaP-X (arXiv:2603.22435) — we take structured execution feedback + verifiable outcomes, but swap the multi-round test-time-compute approach for zero-latency deterministic predicates (a robot can't wait for ensemble voting).
- Gemini Robotics ER 2 — we take human-proximity halt/resume and the safety-benchmark framing, but the halt decision is O(1) deterministic logic on local perception streams, not cloud model inference; there is no sub-second cloud round trip on a deployed robot.
