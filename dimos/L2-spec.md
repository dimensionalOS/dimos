# L2: PolicyGate

We need to add a deterministic gate prior to the capability check, covering risk classification, parameter boundaries, rate limiting, and operator confirmation. The decision is made by code, not by the model.

## 1. Skill author API 

```python
@skill(risk=RiskLevel.MOTION, uses=(CAP_MOVEMENT,),
       bounds={"x": (-0.5, 0.5), "duration": (0.0, 5.0)},
       rate_limit="30/min")
def move(self, x: float, duration: float = 2.0) -> str:
    """move forward or backward ..."""

@skill(risk=RiskLevel.DYNAMIC, uses=(CAP_MOVEMENT,),
       rate_limit="2/min", confirm=ConfirmMode.OPERATOR)
def jump(self) -> str:
    """jump ..."""
```

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

no arguments = `risk=READONLY`. every existing skill compiles and runs exactly as before; higher tiers are opt-in and visible in review. `SkillInfo` gains a `safety` field so the metadata rides along with the existing `get_skills()` — `McpServer.on_system_modules` doesn't change.

## 2. Trust levels 

```python
class TrustLevel(IntEnum):
    UNTRUSTED = 0    # web text, unauthenticated agent_send
    TOOL_RESULT = 1  # anything a tool or sensor returned
    USER_VOICE = 2   # on-site speech
    OPERATOR = 3     # auth local operator / teleop ui

# minimum trust required to trigger each tier

MIN_TRUST: dict[RiskLevel, TrustLevel] = {
    RiskLevel.READONLY:     TrustLevel.UNTRUSTED,
    RiskLevel.MOTION:       TrustLevel.TOOL_RESULT,
    RiskLevel.DYNAMIC:      TrustLevel.USER_VOICE,
    RiskLevel.MANIPULATION: TrustLevel.USER_VOICE,
    RiskLevel.EXTERNAL:     TrustLevel.OPERATOR,
}   
```

## 3. Decision 

```python
@dataclass(frozen=True)
class PolicyDecision:
    allowed: bool
    reason: str = ""                    # shadow hits get a "[shadow-would-deny] " prefix
    needs_confirmation: bool = False
    confirmation_id: str | None = None  # set iff needs_confirmation
```

## 4. Config （`policy_loader.py`）

```python
@dataclass(frozen=True)
class PolicyConfig:
    mode: Literal["enforce", "shadow", "off"] = "shadow"
    defaults: dict[RiskLevel, SkillSafety]     # per-tier defaults
    overrides: dict[str, SkillSafety]          # per-skill, highest priority
    confirmation_timeout_s: float = 30.0
```


merge order: decorator < `defaults[risk]` < `overrides[skill_name]`. field-by-field, not wholesale replacement.

## 5. gate 

`_handle_tools_call` insert before `CapabilityRegistry.acquire`：

```python
decision = app.state.policy_gate.check(name, skill_info.safety, args, provenance)
app.state.audit.record({"skill": name, "args": args, "decision": decision.allowed,
                        "reason": decision.reason})
if not decision.allowed:
    return _jsonrpc_result_text(req_id, f"Safety policy: {decision.reason}")

```

the check is synchronous and sub-ms

```python
def check(self, name, s: SkillSafety, args, trust: TrustLevel) -> PolicyDecision:
    try:
        d = self._inner(name, s, args, trust)
    except Exception as e:
        d = DENY(f"policy gate error: {e}")
    if self.mode == "shadow" and not d.allowed:
        return ALLOW(f"[shadow-would-deny] {d.reason}")   # just record, not block
    return d

def _inner(self, name, s, args, trust):
    if self.mode == "off":             return ALLOW          # audit only
    if s.risk == READONLY:             return ALLOW
    if trust < MIN_TRUST[s.risk]:      return DENY(f"trust {trust} insufficient")
    for p, (lo, hi) in self._merged_bounds(name, s).items():
        if isinstance(args.get(p), (int, float)) and not lo <= args[p] <= hi:
            return DENY(f"'{p}={args[p]}' outside [{lo}, {hi}]")   # deny, not clip
    if s.rate_limit and self._limiter.hit(name, s.rate_limit):
        return DENY(f"rate limit {s.rate_limit} exceeded")   # deny, not count
    if s.confirm == OPERATOR and not self._confirmations.consume(args_confirmation_id):
        req = self._confirmations.register(name, args)
        return DENY(f"needs operator confirmation: dimos mcp confirm {req.id}",
                    needs_confirmation=True, confirmation_id=req.id)
    return ALLOW
```



A rejection with the legal range in the message teaches it the actual envelope.
In shadow mode a deny from steps 3–6 becomes allow + audit; step 6 doesn't create a real ConfirmationRequest in shadow, or you'd pile up confirmations nobody will ever resolve.


## 6. Policy files 


decorator values are the floor; a YAML file adjusts per deployment without touching code. merged field-by-field: decorator < `defaults[risk]` < `overrides[skill]`.

```yaml
mode: shadow          # shadow first, flip to enforce after replay calibration
defaults:
  dynamic:  {confirm: operator, rate_limit: "2/min"}
overrides:
  move:
    bounds: {x: [-0.3, 0.3]}        # different labs have different limits
```


load-time validation is strict: unknown risk name, malformed rate limit, `lo > hi` — startup error, no running with a policy that isn't what you think it is. bad policy = McpServer doesn't come up.

## 7. Two small components 

```python
class SlidingWindowLimiter:
    def hit(self, key: str, spec: str) -> bool:  
        # "6/min"；True = exceeded
        # each skill a deque, pop expired when needed
        # lock protected, append when not exceeded

class ConfirmationRegistry:
    def register(self, skill, args) -> Request    # 30s expiration
    def approve(self, id) -> bool                 #  mcp confirm <id>
    def consume(self, id) -> bool                 # once per approval
```

## 8. Non-goals


- no model in the decision path — the gate is string compares and a deque
- no second path around it — every `tools/call` passes through, no exceptions
- `mode` only controls blocking, never logging — off still audits
- trust tagging itself is L1's job; the gate just reads `TrustLevel` 
