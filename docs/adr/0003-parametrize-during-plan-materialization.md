# Parametrize during plan materialization

Trajectory parametrization runs immediately after geometric planning, before a `GeneratedPlan` is accepted or cached. Preview and execution therefore consume the same validated timed trajectory, and parametrization failures prevent an untimed plan from being presented as ready rather than surfacing during execution.
