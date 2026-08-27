# Use operator motion as the engaged SONIC motion source

While full-body SONIC teleoperation is engaged, the operator pose is the sole whole-body motion source; physical stepping and turning drive the robot, and planner velocity resumes after disengagement. The current SONIC policy selects streamed SMPL motion or planner motion rather than fusing them, so simultaneous thumbstick locomotion would require a separate policy-interface design. Supported heading adjustment may still accompany the operator stream.
