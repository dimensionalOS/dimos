---
status: superseded by ADR-0006
---

# Preserve LIBERO-PRO comparability

The LIBERO-PRO integration will preserve the original benchmark's episode, observation, action, rollout, success, and aggregation semantics as closely as possible so its score can be compared with other reported LIBERO-PRO scores. We reject a separate DimOS-native track with continuous real-time stepping, Panda joint-position control, and a custom horizon because those changes would produce a different benchmark even if it reused LIBERO-PRO tasks and native success predicates.
