# Experimental agent encoding

Agent-facing views of sensor messages. Message types keep a thin `agent_encode()`
entry point and an `agent_encode_legend()`; the encoders load on first use.

- `pointcloud/`: queries, fields, renders, picking. Entry: `PointCloud2.agent_encode()`.

## Point clouds

Overview: [`pointcloud/README.md`](/dimos/experimental/agent_encode/pointcloud/README.md). Agent instructions:
`legend()` in [`pointcloud/runtime/dispatch.py`](/dimos/experimental/agent_encode/pointcloud/runtime/dispatch.py), served as `PointCloud2.agent_encode_legend()`.

## Testing

Run the encoding tests from the repository root:

```bash
uv run pytest dimos/experimental/agent_encode
```

These tests check geometry queries, fields, rendering, picking and response budgets
using synthetic inputs. They do not require a running robot or an LLM API key.
