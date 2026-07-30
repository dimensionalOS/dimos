# Scope RoboPlan TOPP-RA to RoboPlanWorld

The RoboPlan TOPP-RA parametrization backend accepts geometric paths produced by any planner, but it is available only when the manipulation world is `RoboPlanWorld`. This preserves planner independence without introducing and synchronizing a second RoboPlan robot model for other world backends; unsupported backend combinations fail during startup.
