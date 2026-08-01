<!-- Copyright 2026 Dimensional Inc. -->

# Static spatial benchmark agent-evaluation proposal

## Status and boundary

This document records the current design exploration for evaluating an agent
against the static spatial benchmark. It is not an implementation plan or an
accepted benchmark protocol. The existing corpus evaluates questions over an
immutable `PointCloud2` map variant; it does not yet define agent inputs,
tools, answer capture, or scoring.

The evaluation must measure a configured spatial evaluation system rather than
only a language model. That system includes the model, prompt, public map
representation, allowed tools, and answer protocol. The private oracle remains
outside that system.

## Native DimOS evaluation path

The primary candidate is a per-instance, isolated DimOS deployment that uses
the normal agent pathway rather than a benchmark-only wrapper:

```text
public case envelope
  -> PublicCorpusSource
  -> public spatial-analysis modules
  -> @skill methods through McpServer
  -> McpClient with a benchmark prompt
  -> prediction capture

private scorer
  -> oracle answer and review override
  -> immutable score record
```

`PublicCorpusSource` would verify the selected public map artifact and its
frame contract before making it available to analysis modules. The agent would
receive the public question and call only an explicit MCP allowlist. It would
not receive corpus paths, a raw module registry, an LCM endpoint, private
geometry, answers, review overrides, or source annotations.

The proposed agent-facing operations are deliberately bounded. A map summary
can report public metadata. A top-down inspection operation can return a
quantized, size-limited map region. Other generic spatial operations may be
considered only when their authority is clear and identical across cases. An
answer-capture operation would accept one typed answer and confirm receipt
without reporting correctness.

The public case should start with fresh agent history and fresh case state.
The scheduler and private scorer should remain outside the blueprint. They are
benchmark infrastructure, not capabilities available to the evaluated agent.

## Public and private trust domains

The agent process must be unable to discover the private oracle by filesystem,
module reference, transport, log, or tool call. The design therefore separates
three trust domains:

```text
Public scheduler: manifest, selected public case, run configuration
Agent sandbox: one public map, one question, allowed public tools
Private scorer: answers, overrides, scoring policy, aggregate reports
```

The private scorer receives a prediction after the case ends. It never returns
incremental correctness, answer-dependent errors, or score information to the
agent environment. A public-only loader is required because the corpus loader
used for inspection may join public and oracle records.

## Candidate code-as-action track

SpatialClaw demonstrates a different interaction model: an agent writes
Python cells that execute against a stateful tool namespace. A single DimOS
`execute_code` skill running arbitrary Python inside a normal module would
resemble that interface, but it would not be an acceptable official track.

Normal DimOS workers are not hostile-code sandboxes. An executor with a broad
set of RPC proxies could bypass typed Specs, invoke non-skill RPCs, inspect
ambient files and environment variables, access transports, evade nested tool
metering, or reach private material. MCP capability accounting would cover the
outer `execute_code` call but not the arbitrary actions made inside it.

If code-as-action is later evaluated, it should be a separately reported
experimental track with this boundary:

```text
agent -> execute_code(code) -> ephemeral external sandbox
                              -> constrained public SDK
                              -> capability broker
                              -> explicit public DimOS operations
```

The sandbox would receive no `RPCClient`, module instance, LCM access, MCP URL,
filesystem mount, credential, or oracle reference. The capability broker would
hold declared public dependencies and exchange only validated primitive values
or canonical JSON. It would bind every request to the current case, enforce
resource and response limits, and record ordered nested operations. A normal
DimOS worker, a restricted Python globals dictionary, an AST filter, or an
import denylist is not a sufficient sandbox.

## Scoring and evidence

Each case produces at most one canonical prediction for its `instance_id`.
Missing, late, duplicate, malformed, or type-incompatible submissions are
scored as incorrect. The private scorer joins the prediction to the physical
question's oracle answer and applies a versioned review-override policy.

The principal result should be macro exact accuracy across the seven
predicates. Supporting measures should include micro accuracy, per-predicate
and per-variant accuracy, clean-to-noisy degradation, all-variant consistency,
invalid submissions, timeouts, tool failures, tool calls, tokens, latency, and
cost. Confidence intervals must resample scenes rather than correlated
map-question instances.

Every run needs an immutable manifest recording corpus and map hashes, public
case ordering, DimOS revision and dependency image, prompt, tool schema and
implementation digests, model identity and sampling parameters, budgets, and
the prediction ledger. Per-case transcripts and tool traces are evidence for
diagnosis, not sources of scoring truth.

## Proposed rollout

First restore and freeze the corpus, then implement a public-only case loader,
private scorer, and deterministic non-agent baselines. Next, prove the native
MCP plumbing with a mock model and run a small development subset with a real
model. Freeze prompts, schemas, budgets, and runtime images before the blind
held-out run. Parallel execution, cache reuse, new tool families, and a
code-as-action track should follow only after that baseline is stable.

## Decisions still required

The next design decision is whether the official benchmark measures only the
native MCP skill interface or reports both a native MCP track and an assisted
tool track. A separate decision is whether unrestricted program composition is
a research goal. If it is, code-as-action requires an external sandbox and a
capability broker; it must not be implemented as arbitrary `exec()` inside a
regular DimOS module.
