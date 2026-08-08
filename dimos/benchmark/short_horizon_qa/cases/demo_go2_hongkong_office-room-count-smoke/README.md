# Hong Kong office room-count CLI smoke case

This case exercises the real `go2_hongkong_office` recording at progress `1.0`
through the standalone evaluation CLI.

Its expected count is the synthetic sentinel `0`. It validates CLI and runtime
plumbing only and is not the benchmark room-count oracle. Do not interpret a
failed task score as an agent or mapping regression.

The authoritative case remains incomplete until a human-authored room inventory,
counting policy, and independent review establish the expected count.

Use this case to exercise the direct stock-Pi CLI path. Any observed answer is
experimental until the oracle is replaced with a reviewed room inventory.
