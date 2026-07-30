# Select one trajectory parametrization backend at startup

Each manipulation deployment selects exactly one trajectory parametrization backend at startup. If that backend cannot parametrize a geometric path, plan materialization fails explicitly; the system does not fall back to another parametrizer because doing so would silently change trajectory semantics, timing, and failure behavior. A selected backend may use its own documented safety behavior between internal curve-fitting modes, such as RoboPlan TOPP-RA falling back from a colliding linear blend to Hermite fitting.
