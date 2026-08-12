# Use per-call manipulation speed scales

Planning and immediate-move RPCs accept an optional speed scale for that call, falling back to stable operation-specific module configuration. The cleaned interface removes session-mutating speed setters and getters because agent-authored modules should not depend on prior calls or silently affect another caller's future plan. Speed scales remain dimensionless values greater than zero and at most one.
