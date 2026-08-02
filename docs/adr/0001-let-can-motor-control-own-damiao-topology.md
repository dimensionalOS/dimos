# Let can-motor-control and adapter subclasses own Damiao topology

Each Damiao adapter subclass constructs its inherent hardware topology directly with upstream `can_motor_control` types instead of mirroring buses, motors, arm groups, gripper groups, or topology validation in DimOS configuration types. Module configuration contains only deployment and control-policy values that may vary at runtime, including address overrides keyed by the subclass's inherent bus names. Topology construction remains an implementation detail of the subclass, avoiding divergent validation while preserving upstream multi-arm and multi-gripper composition.

The shared Damiao runtime configuration is limited to named bus-address overrides, the gravity-compensation switch, and the upstream tick deadline. Gains use the existing whole-body configuration; mock transports belong to test subclasses rather than production configuration.
