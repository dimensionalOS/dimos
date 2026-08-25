# L2: PolicyGate — dispatch-time policy checks for skills

Part of the agent safety layer umbrella (DASL). This is the piece that lands first and covers the most: today any `tools/call` that passes the capability mutex goes straight to RPC. `move(x=10.0)` is a legal call as far as the server is concerned. We want a deterministic gate in front of that check — risk tiers, argument bounds, rate limits, operator confirmation — decided by code, not by the model.

## what a skill author writes

```python
@skill(risk=RiskLevel.MOTION, uses=(CAP_MOVEMENT,),
       bounds={"x": (-0.5, 0.5), "duration": (0.0, 5.0)},
       rate_limit="30/min")
def move(self, x: float, duration: float = 2.0) -> str:
    """Move the robot forward or backward. ..."""

@skill(risk=RiskLevel.DYNAMIC, uses=(CAP_MOVEMENT,),
       rate_limit="2/min", confirm=ConfirmMode.OPERATOR)
def jump(self) -> str:
    """Jump in place. ..."""
```

no arguments = `risk=READONLY` = untouched behavior. every existing skill compiles and runs exactly as before; higher tiers are opt-in and visible in review. `SkillInfo` gains a `safety` field so the metadata rides along with the existing `get_skills()` serialization — `McpServer.on_system_modules` doesn't change.

## the gate

one new call in `_handle_tools_call`, inserted before `CapabilityRegistry.acquire`:

```python
decision = app.state.policy_gate.check(name, skill_info.safety, args, provenance)
app.state.audit.record({"skill": name, "args": args, "decision": decision.allowed,
                        "reason": decision.reason})
if not decision.allowed:
    return _jsonrpc_result_text(req_id, f"Safety policy: {decision.reason}")
# existing capability logic below is untouched
```

the check itself, synchronous and sub-ms:

```python
def check(self, name, s: SkillSafety, args, trust: TrustLevel) -> PolicyDecision:
    try:
        d = self._inner(name, s, args, trust)
    except Exception as e:
        d = DENY(f"policy gate error: {e}")          # fail closed, always
    if self.mode == "shadow" and not d.allowed:
        return ALLOW(f"[shadow-would-deny] {d.reason}")   # log, don't block (yet)
    return d

def _inner(self, name, s, args, trust):
    if self.mode == "off":             return ALLOW          # audited anyway
    if s.risk == READONLY:             return ALLOW
    if trust < MIN_TRUST[s.risk]:      return DENY(f"trust {trust} insufficient")
    for p, (lo, hi) in self._merged_bounds(name, s).items():
        if isinstance(args.get(p), (int, float)) and not lo <= args[p] <= hi:
            return DENY(f"'{p}={args[p]}' outside [{lo}, {hi}]")   # reject, never clamp
    if s.rate_limit and self._limiter.hit(name, s.rate_limit):
        return DENY(f"rate limit {s.rate_limit} exceeded")   # denied calls don't burn quota
    if s.confirm == OPERATOR and not self._confirmations.consume(args_confirmation_id):
        req = self._confirmations.register(name, args)
        return DENY(f"needs operator confirmation: dimos mcp confirm {req.id}",
                    needs_confirmation=True, confirmation_id=req.id)
    return ALLOW
```

why reject instead of clamp: a clamped call *succeeds*, so the agent learns "asking for 10 m/s works" and will do it again. a rejection with the legal range in the message teaches it the actual envelope.

## policy file

decorator values are the floor; a YAML file adjusts per deployment without touching code. merged field-by-field: decorator < `defaults[risk]` < `overrides[skill]`.

```yaml
mode: shadow          # shadow first, flip to enforce after replay calibration
defaults:
  dynamic:  {confirm: operator, rate_limit: "2/min"}
overrides:
  move:
    bounds: {x: [-0.3, 0.3]}        # this lab's robots, this lab's limits
```

load-time validation is strict: unknown risk name, malformed rate limit, `lo > hi` — startup error, no running with a policy that isn't what you think it is. bad policy = McpServer doesn't come up.

## the two helper pieces, also small

```python
class SlidingWindowLimiter:
    def hit(self, key: str, spec: str) -> bool:   # "6/min"; True = over limit
        # deque[monotonic_ts] per skill, evict expired lazily, take a lock,
        # only append when under the limit

class ConfirmationRegistry:
    def register(self, skill, args) -> Request    # uuid, expires in 30s
    def approve(self, id) -> bool                 # dimos mcp confirm <id>
    def consume(self, id) -> bool                 # one-shot: one approval, one call
```

## prohibited

- no model in the decision path — the gate is string compares and a deque
- no second path around it — every `tools/call` passes through, no exceptions
- `mode` only controls blocking, never logging — off still audits
- trust tagging itself is L1's job; the gate just reads `TrustLevel` (defaults OPERATOR until L1 ships, so old clients keep working)

## open questions

- headless deployments (daemon, no UI): confirmation timeout = deny, or queue-and-execute-later? leaning deny — silent queuing of physical actions feels worse than a clean refusal.
- shadow → enforce: how many replay hours do we want before flipping the default? the replay DBs we already have (`go2_bigoffice` etc.) should answer the false-deny question empirically.
- L3's safety monitor wants to tighten bounds dynamically (`set_dynamic_factor`) when a person is nearby — worth defining that interface here so L3 doesn't fork the gate later.
- per-blueprint policy files: merge over the default YAML, or full replacement? leaning merge, full replacement invites "forgot the defaults" accidents.

## relations

umbrella spec: DASL issue (trust levels, runtime monitor, verification, audit). this layer is P1 in the rollout and the one that unblocks everything else — L0/L5 land first as prerequisites, then this.
