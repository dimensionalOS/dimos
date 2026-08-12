# Linear moves skip planning-world collision checks by default

`move_linear` is an immediate, world-relative Cartesian motion primitive intended for short visual-servo corrections, so it defaults to `check_collision=False`. It still uses RoboPlan Cartesian trajectory generation and retains reachability, joint-limit, timing, trajectory-validation, arbitration, and execution checks. Callers request planning-world collision validation explicitly with `check_collision=True`, and the returned `MoveResult` representation always exposes whether collision checking was enabled.
