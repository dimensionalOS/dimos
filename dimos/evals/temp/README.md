# temp/ — autoresearch scaffolding

Everything here exists to run one [evo](https://github.com/evo-hq/evo)
optimization of `PointCloud2.agent_encode()`. **It comes out before the PR
lands.** Nothing outside `dimos/evals/` imports it.

| File | What it is |
|---|---|
| `split.py` | train / holdout / spare slices over generated rows; the four pointcloud suites import it |
| `tool_evo_bench.py` | the benchmark evo runs — evo's inline instrumentation contract over `EvalRunner` |
| `tool_evo_gate.py` | the four gates: `static`, `budget`, `floors`, plus the holdout floor in the bench |
| `test_evo.py` | offline tests for all of the above |
| `evo_frozen.json` | hash manifest the `static` gate checks the benchmark against |

Deleting this directory means reverting one line in `generate.py` (the slice
tag) and the `split.assign(...)` call in the three sliced suites — after that
the suites run every row, as they did before.

How the loop is wired, what it costs, and what it is trying to fix:
[autoresearch.md](autoresearch.md).
