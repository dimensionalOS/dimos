# Require verified holding for pick success

`pick_selected` succeeds only when configured gripper feedback verifies that an object prevented empty closure. Real hardware, simulation, and test adapters must provide equivalent verification behavior; missing feedback, empty closure, or any post-close transaction error returns failure, and the module never reports an unverified success or automatically opens after a post-close failure.
