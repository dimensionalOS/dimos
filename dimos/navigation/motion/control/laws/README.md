# laws

One module per law. The follower runs `hinted` (the default of
`TrajectoryFollowerConfig.controller`; the native twin runs its rust port).

`seed` is the permanent baseline — every A/B is against it and every search
seeds from it, so it does not absorb research results. A research generation
lands by replacing `hinted`.

Each `make_rust` law is a port of its python twin, not a variant:
`test_rust_parity.py` holds every pair to 1e-9 per twist component.
