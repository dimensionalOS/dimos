# Model can-motor-control robots as whole-body hardware

DimOS integrates each upstream `can_motor_control.Robot` through `WholeBodyAdapter` as one ordered set of named joints rather than wrapping each arm in `ManipulatorAdapter`. Arm groups contribute angular joints and gripper groups contribute calibrated normalized opening joints; tasks select any arm or gripper subset by name, while the adapter queues all group commands and advances the upstream robot once per control cycle. This matches the existing G1 integration shape and lets the same adapter naturally cover single-arm and dual-arm topologies without inventing richer manipulator behavior that upstream does not provide.

Whole-body position coordinates are joint-specific: arm positions are radians, while calibrated gripper openings use `[0.0, 1.0]`. Until calibrated opening velocity and physical jaw effort exist, gripper velocity and effort feedback are zero and gripper commands use only the position target; other command fields are ignored for that joint.

Damiao grippers use the same arbitrated joint-command path as other whole-body joints. The Damiao integration does not add or implement manipulator-specific gripper RPCs; existing xArm and Piper gripper methods remain compatibility paths for those adapters.

The generic Damiao whole-body adapter provides URDF-based gravity compensation for angular joints. It treats commanded torque as residual torque and sends `tau_command + g(q)` while passing position, velocity, and gains through unchanged; normalized gripper joints are excluded. The gravity-model path is inherent to the robot subclass, a runtime switch enables or disables compensation, and model mismatch or non-finite gravity output prevents activation.

The generic Damiao adapter and robot-specific subclasses live in the whole-body hardware family and register with its adapter registry. They are not manipulator adapters and do not register with the manipulator hardware family.

The adapter delegates topology validation, CAN routing, transport errors, lifecycle ordering, and gripper calibration to upstream. DimOS performs gravity-model preflight before activation, converts errors at the whole-body interface, and advances the complete upstream robot once per control cycle; it does not add local transport retries, kernel-interface probes, or per-arm caches.

Robot subclasses use inherent class properties mapping upstream arm-group names to ordered DimOS arm joints and upstream gripper-group names to normalized DimOS gripper joints. These mappings are the only robot-specific integration metadata; they do not re-describe upstream physical topology.

Simulation and blueprint tests use a generic in-memory whole-body adapter rather than the manipulator mock. It stores ordered joint state and accepts whole-body command vectors without adding robot-specific behavior.

The hardware identifier names the physical upstream robot, while logical arm and gripper prefixes remain joint namespaces used by planners and tasks. A single OpenYAM therefore uses hardware identifier `openyam` with `arm/...` joints; a dual-arm robot can use one physical identifier with independent `left_arm/...` and `right_arm/...` joint subsets.

Gripper servo tasks start without a default target. Upstream calibration establishes real opening feedback, and normal control sends no gripper target until an explicit command arrives; calibration motion to both endpoints remains an activation-time safety consideration.
