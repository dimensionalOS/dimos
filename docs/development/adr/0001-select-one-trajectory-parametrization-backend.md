# Select one trajectory parametrization backend at startup

Each manipulation deployment selects exactly one trajectory parametrization
backend at startup. `TrajectoryParametrizerSpec` owns the complete successful
`PlanningResult`-to-`GeneratedPlan` conversion: it preserves the source path,
converts untimed paths, recognizes planner-native timed results, and validates
the canonical trajectory. The manipulation module only selects planning groups
and atomically stores the accepted result.

Every untimed geometric path uses the selected backend. Planner-native timed
results bypass parametrization because they are already trajectories, not
because a backend failed. If the selected backend cannot parametrize a
geometric path, plan materialization fails explicitly; the system does not fall
back to another parametrizer because doing so would silently change trajectory
semantics, timing, and failure behavior. A selected backend may use its own
documented safety behavior between internal curve-fitting modes, such as
RoboPlan TOPP-RA falling back from a colliding linear blend to Hermite fitting.
