# Project terminology

Use these terms consistently in code, documentation, issues, and reviews.

| Term | Meaning |
|------|---------|
| Recording engine | The implementation that captures streams: `python` or experimental `rust`. |
| Recording format | The on-disk container and layout: `sqlite` or `mcap`. |
| Recording artifact | The single file produced for a run, such as `memory.db` or `memory.mcap`. |
| Stream selection | The logical blueprint stream names chosen by `--record-topics`. |

Avoid using “backend” for recording choices because it can mean either the
recording engine, recording format, or transport. Name the specific concept.
