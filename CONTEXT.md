# DimOS

DimOS composes software capabilities and data into runnable robot systems.

## Language

**Public model artifact**:
A machine-learning model artifact that anyone can download from its canonical provider without authentication, access approval, or accepting a gated-access request. Its license must remain discoverable, but the license type does not determine migration eligibility.
_Avoid_: Unlicensed model, unrestricted model

**Pinned model artifact**:
The exact model weight already used by DimOS, identified in its public source repository by an immutable revision and filename. Replacing it with compatible or newer weights is a model upgrade, not an artifact-location change.
_Avoid_: Latest model, equivalent model

**Canonical model provider**:
The model’s original publisher or recognized maintainer distributing the artifact from a repository they control. DimOS-owned and personal mirrors are not canonical providers for third-party or custom models.
_Avoid_: Public mirror, model cache

**Model-family migration**:
An atomic move of every actively supported weight in one model family to its canonical provider. If any active weight lacks an exact canonical source, the family remains in its current store rather than being split across providers.
_Avoid_: Partial migration, mixed-source model

**Orphaned model artifact**:
A stored model or model fixture with no loader, test, blueprint, skill, or other supported consumer in the current DimOS codebase. It is removed rather than migrated to another artifact store.
_Avoid_: Legacy model, inactive model
