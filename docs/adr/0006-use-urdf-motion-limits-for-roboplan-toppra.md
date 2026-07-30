# Use URDF motion limits for RoboPlan TOPP-RA

The RoboPlan TOPP-RA backend uses velocity and acceleration limits loaded by its RoboPlan scene from the robot URDF. Missing required limits fail explicitly rather than falling back to DimOS's current generic motion-limit fields; formal, globally named per-joint DimOS overrides are deferred to a separate change and will later map into RoboPlan's supported limit-override mechanism.
