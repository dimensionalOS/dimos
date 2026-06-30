# Use generated Composite RoboPlan models for multi-robot planning

For multi-robot RoboPlan planning, DimOS will generate one Composite RoboPlan model: a single RoboPlan-facing URDF/SRDF built from the registered robot models. This lets RoboPlan plan coupled Composite planning groups and check inter-robot collisions in one `Scene`, instead of using separate per-robot scenes that cannot represent coordinated collision-aware motion.

Single-robot RoboPlan may continue to pass a provided SRDF directly. Multi-robot RoboPlan always generates a composite SRDF at world finalization, including deterministic Composite planning groups for all non-overlapping planning-group combinations within a safety cap. Non-selected joints remain fixed by setting RoboPlan `Scene` current joint positions from the Planning world before invoking group RRT.
