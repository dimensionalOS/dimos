---
status: superseded by ADR-0016
---

# Seal LIBERO-PRO evaluation cases

The single LIBERO-PRO integration configures code-policy exploration with an unperturbed training suite from the pinned LIBERO-PRO source, while perturbed evaluation suites, evaluation initial states, and native outcome feedback remain sealed until the scored Evaluation Stage. The Policy Artifact is frozen before it encounters that held-out configuration. This preserves task-specific interactive learning while retaining LIBERO-PRO's intended measurement of generalization under perturbation; it does not introduce a separate LIBERO integration.
