# Use Unchecked Contact Motion for contact legs

Pre-grasp approach remains collision-checked, while grasp, retreat, place lowering, and post-release retract use short straight TCP paths with planning-scene collision checks disabled. This keeps the scene unchanged and matches the working implementation, but it provides no non-target collision guarantee on those legs; selected-target-only collision allowance is a future planner capability rather than part of the integration.
