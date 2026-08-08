# Compose exactly one Grasp Provider

Each pick/place blueprint composes exactly one module implementing the grasp-provider interface, initially either heuristic grasp generation or GraspGenX. The transaction contains no embedded heuristic and never falls back automatically, so provider dependencies, candidate semantics, failures, and tests remain explicit and deterministic.
