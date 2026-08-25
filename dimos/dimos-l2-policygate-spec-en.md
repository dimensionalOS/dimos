# L2 PolicyGate — Implementation Spec

Companion to the DASL proposal, section 5.2. This is the P1 deliverable; it assumes P0 has already landed (the `@skill` metadata and the audit skeleton).

Blast radius: new package `dimos/agents/safety/`, extra kwargs on the `@skill` decorator, one block inserted into `mcp_server.py`, one new field on `SkillInfo`. That's the whole diff surface.

## 1. Problem

Today a `tools/call` comes in, the server checks the capability isn't held, and fires the RPC. Nothing asks whether the call *should* happen. We want a deterministic gate in front of that: risk tiers, argument bounds, rate limiting, operator confirmation. It has to be synchronous and sub-millisecond — this sits in the hot path of every single tool call.

Why not fold this into `CapabilityRegistry`? That class is about mutual exclusion over shared resources. "Is the resource free" and "should this call happen at all" are different questions, and they'll evolve on different schedules. Keep them apart.

In scope: risk metadata, trust-level checks, argument bounds, rate limits, confirmations, shadow mode, policy file loading, audit emission.
Out of scope: how L1 actually tags messages (we just consume the result), the L3 monitor, L4 execution verification, and anything the confirmation UI looks like.

## 2. Layout

```
dimos/agents/safety/
├── __init__.py
├── policy.py            # PolicyGate, PolicyDecision, TrustLevel, MIN_TRUST
├── policy_loader.py     # YAML -> validated PolicyConfig
├── rate_limiter.py      # SlidingWindowLimiter
├── confirmation.py      # ConfirmationRegistry (pending / approve / consume / expire)
└── policies/
    └── default.yaml
```

## 3. Types

### 3.1 Skill metadata (lands with P0, shown for context)

```python
class RiskLevel(Enum):
    READONLY = "readonly"
    MOTION = "motion"
    DYNAMIC = "dynamic"
    MANIPULATION = "manipulation"
    EXTERNAL = "external"

class ConfirmMode(Enum):
    NONE = "none"
    OPERATOR = "operator"

@dataclass(frozen=True)
class SkillSafety:
    risk: RiskLevel = RiskLevel.READONLY
    bounds: dict[str, tuple[float, float]] = field(default_factory=dict)
    rate_limit: str | None = None          # "<n>/<s|min|h>", e.g. "6/min"
    confirm: ConfirmMode = ConfirmMode.NONE
    verify: str | None = None
```

### 3.2 Trust levels

```python
class TrustLevel(IntEnum):
    UNTRUSTED = 0    # web text, unauthenticated agent_send
    TOOL_RESULT = 1  # anything a tool or sensor returned
    USER_VOICE = 2   # on-site speech (STT)
    OPERATOR = 3     # authenticated local operator / teleop UI

# minimum trust required to trigger each tier (the hard rule from the proposal)
MIN_TRUST: dict[RiskLevel, TrustLevel] = {
    RiskLevel.READONLY:     TrustLevel.UNTRUSTED,
    RiskLevel.MOTION:       TrustLevel.TOOL_RESULT,
    RiskLevel.DYNAMIC:      TrustLevel.USER_VOICE,
    RiskLevel.MANIPULATION: TrustLevel.USER_VOICE,
    RiskLevel.EXTERNAL:     TrustLevel.OPERATOR,
}
```

### 3.3 Decision

```python
@dataclass(frozen=True)
class PolicyDecision:
    allowed: bool
    reason: str = ""                    # shadow hits get a "[shadow-would-deny] " prefix
    needs_confirmation: bool = False
    confirmation_id: str | None = None  # set iff needs_confirmation
```

### 3.4 Config (what `policy_loader.py` hands you)

```python
@dataclass(frozen=True)
class PolicyConfig:
    mode: Literal["enforce", "shadow", "off"] = "shadow"
    defaults: dict[RiskLevel, SkillSafety]     # per-tier defaults
    overrides: dict[str, SkillSafety]          # per-skill, highest priority
    confirmation_timeout_s: float = 30.0
```

Merge order: decorator < `defaults[risk]` < `overrides[skill_name]`. Field-by-field, not wholesale replacement — an override that only sets `bounds` leaves everything else alone.

## 4. The gate itself

```python
class PolicyGate:
    def __init__(self, config: PolicyConfig,
                 confirmations: ConfirmationRegistry,
                 audit: AuditTrail | None) -> None: ...

    def check(self,
              skill_name: str,
              safety: SkillSafety,
              args: dict[str, Any],
              provenance: TrustLevel) -> PolicyDecision: ...
```

Non-negotiables:

- Synchronous, no I/O (the audit write aside), under 1 ms per call. Hot path, every tool call.
- Nothing escapes as an exception. Any internal error becomes `PolicyDecision(False, "policy gate error: ...")`. If the safety layer has a bug, it fails toward denial, never toward permission.
- It never touches robot state. The only side effects are the rate-limit window and the audit record.
- `mode="off"` always allows — but still audits. Otherwise the next incident review turns into "was it the gate?" and nobody can answer.

## 5. Decision order

```
1. mode == "off"            -> allow
2. risk == READONLY         -> allow (audit still written)
3. provenance < MIN_TRUST[risk]
                            -> deny "trust {X} insufficient for risk '{Y}'"
4. merged bounds: for each (param, (lo, hi)),
   if args[param] is numeric and out of range
                            -> deny "'{param}={v}' outside safe range [{lo}, {hi}]"
5. rate limit hit           -> deny "rate limit {spec} exceeded for '{skill}'"
6. confirm == OPERATOR:
   - no valid confirmation  -> register one, deny with needs_confirmation + id
   - approved id in _meta   -> consume it, allow
7. otherwise                -> allow
```

Fixed order, first hit returns. Don't get clever about reordering.

Two things worth spelling out:

- Bounds violations are **rejected, not clamped**. The denial message includes the legal range. Clamping teaches the agent a wrong causal story — it asked for 10 m/s, something moved, it will ask for 10 m/s again.
- In `shadow` mode, a deny from steps 3–6 is rewritten to allow with the reason prefixed `[shadow-would-deny] `. Step 6 in shadow mode does **not** create a real `ConfirmationRequest` — audit only — or you'd accumulate a pile of confirmations nobody will ever resolve.

## 6. Pieces

### 6.1 SlidingWindowLimiter

```python
class SlidingWindowLimiter:
    def hit(self, key: str, spec: str) -> bool:
        """Record one call. Returns True if the limit is now exceeded (caller should deny)."""
```

- Spec grammar: `"<n>/<s|min|h>"`. Parsed and validated at load time; a bad spec is a `ValueError` at startup, so `hit()` never sees one.
- One `deque[float]` of `time.monotonic()` stamps per skill. Expired entries are evicted lazily on access — no background sweeper.
- Thread safety: the server calls `check()` from executor threads, so window ops take a lock. The critical section is a popleft loop and one append; microseconds, not a bottleneck.
- **Denied calls don't consume quota.** Otherwise the first over-limit burst permanently locks the skill out — a self-inflicted denial spiral.

### 6.2 ConfirmationRegistry

```python
@dataclass(frozen=True)
class ConfirmationRequest:
    id: str                    # uuid4 hex
    skill_name: str
    args: dict[str, Any]
    created_ts: float
    expires_ts: float

class ConfirmationRegistry:
    def register(self, skill_name: str, args: dict) -> ConfirmationRequest: ...
    def approve(self, confirmation_id: str) -> bool: ...   # False if unknown or expired
    def consume(self, confirmation_id: str) -> bool: ...   # one-shot: removes it
    def pending(self) -> list[ConfirmationRequest]: ...
```

- Requests live for `confirmation_timeout_s` (30 s default), reaped lazily on access.
- `consume` is one-shot. One id, one allowed call. No replaying an approval.
- Approval goes through the CLI: `dimos mcp confirm <id>`, list pending with `dimos mcp confirmations`. Reuses the existing MCP admin surface — no new ports, no new endpoints.

### 6.3 Policy file

```yaml
mode: shadow
confirmation_timeout_s: 30
defaults:
  motion:        {rate_limit: "30/min"}
  dynamic:       {confirm: operator, rate_limit: "2/min"}
  manipulation:  {confirm: operator}
  external:      {confirm: operator}
overrides:
  move:
    bounds: {x: [-0.5, 0.5], duration: [0.0, 5.0]}
  execute_sport_command:
    rate_limit: "2/min"
```

- Everything validates at load: unknown risk names, malformed rate limits, `lo > hi` bounds all raise immediately. Better to crash at startup than to run with a policy that isn't what you think it is.
- Path comes from `GlobalConfig.safety_policy`; falls back to `policies/default.yaml`.
- A blueprint can ship its own YAML; it merges over the default with the same field-level rules.

## 7. Wiring into the MCP server

One block in `_handle_tools_call`, ahead of the existing `CapabilityRegistry.acquire`. About fifteen lines; nothing already there changes:

```python
safety: SkillSafety = skill_info.safety if skill_info else SkillSafety()
provenance = TrustLevel[meta.get("dimos/trust", "OPERATOR")]  # from L1; OPERATOR until L1 lands

decision = app.state.policy_gate.check(name, safety, args, provenance)
app.state.audit.record({
    "skill": name, "args": args, "risk": safety.risk.value,
    "trust": provenance.name,
    "decision": "allow" if decision.allowed else "deny",
    "reason": decision.reason,
    "confirmation_id": decision.confirmation_id,
})
if not decision.allowed:
    if decision.needs_confirmation:
        return _jsonrpc_result_text(
            req_id,
            f"Action requires operator confirmation. id={decision.confirmation_id}. "
            f"Ask the operator to run: dimos mcp confirm {decision.confirmation_id}")
    return _jsonrpc_result_text(req_id, f"Safety policy: {decision.reason}")
```

The rest of the touch points:

| Where | What | Note |
|-------|------|------|
| `dimos/core/module.py` | `SkillInfo` gains `safety: SkillSafety` | `get_skills()` picks up `__safety__`; unannotated legacy skills default to READONLY |
| `McpServer.start` | build the gate, attach to `app.state.policy_gate` | policy path from `GlobalConfig.safety_policy` |
| `McpServer.stop` | nothing | no background resources to release |
| startup failure | bad YAML -> `start` raises, module never comes up | no valid policy, no MCP surface. Not negotiable. |

Also: `_handle_tools_list` adds `dimos/risk` to each tool's `_meta` so clients can see the tier up front. Informational only; it plays no part in the decision.

## 8. Invariants

Check these in review:

1. **I1** — every `tools/call` goes through `PolicyGate.check`. There is no second path to the RPC.
2. **I2** — `check()` never raises at its caller. An error is a deny.
3. **I3** — out-of-bounds arguments never reach the skill. We reject; we don't silently fix.
4. **I4** — a confirmation id admits exactly one call.
5. **I5** — `mode` controls blocking, never logging.
6. **I6** — a denied call doesn't burn rate-limit quota.

## 9. Tests (`dimos/agents/safety/test_policy.py`)

| Case | Input | Expect |
|------|-------|--------|
| readonly passthrough | risk=READONLY, trust=UNTRUSTED | allow |
| insufficient trust | risk=DYNAMIC, trust=TOOL_RESULT | deny, "insufficient" in reason |
| trust exactly at floor | risk=DYNAMIC, trust=USER_VOICE | proceeds past step 3 |
| arg out of bounds | bounds x in [-0.5, 0.5], args x=10.0 | deny, reason names the legal range |
| boundary values | x = 0.5 and x = -0.5 | allow |
| non-numeric arg | x="fast" | bounds check skipped (schema layer's job) |
| override precedence | decorator bounds wide, YAML override narrow, value inside wide but outside narrow | deny |
| rate limit | "2/min", three calls back to back | third denied; a fourth still denied (no quota burned) |
| window slides | over limit, mock clock +61 s | allow |
| confirmation flow | confirm=OPERATOR, first call | deny + needs_confirmation + id |
| confirmation consumed | approve(id), retry with id | allow; same id again -> deny (I4) |
| confirmation expired | register, mock clock +31 s | approve -> False |
| shadow mode | mode=shadow, out-of-bounds call | allow, reason prefixed, audit holds the original deny |
| off mode | mode=off, out-of-bounds call | allow, audit still written (I5) |
| internal error | malformed bounds structure | deny "policy gate error", nothing raised (I2) |
| concurrency | 10 threads hitting one window | exactly n allows |
| integration | boot McpServer, `tools/call` an out-of-bounds `move` | "Safety policy: ..." returned, capability never acquired |

## 10. Done means

1. Everything above passes; `uv run pytest dimos/agents/safety/` runs in the default fast suite.
2. `check()` p99 under 1 ms — a benchmark test pinning the order of magnitude is enough, no strict perf gate.
3. `mypy --strict` clean.
4. Existing blueprints behave identically under the default `mode: shadow`. `test_tool_stream.py` stays green — that's the regression floor.
5. Every `tools/call` produces exactly one audit record. Not zero, not two.
