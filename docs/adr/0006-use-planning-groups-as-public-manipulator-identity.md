# Use planning groups as the public manipulator identity

The manipulation RPC interface identifies controllable kinematic groups only by opaque planning-group ID. It exposes no robot-name selectors, robot registries, or robot-keyed state because planning, Cartesian targets, joint targets, end-effector state, and immediate moves all operate on groups. Hardware and model ownership remain implementation details, allowing the underlying robot-name concept to be removed without another public interface migration.
