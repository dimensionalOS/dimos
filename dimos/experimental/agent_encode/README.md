# Experimental agent encoding

Agent-facing views of sensor messages. Message types keep a thin `agent_encode()`
entry point and an `agent_encode_legend()`; the encoders load on first use.

- `pointcloud/`: shapes, queries, grids, images and picking. Entry: `PointCloud2.agent_encode()`.

## Point clouds

Overview: [`pointcloud/README.md`](/dimos/experimental/agent_encode/pointcloud/README.md). Agent instructions:
`legend()` in [`pointcloud/legend.py`](/dimos/experimental/agent_encode/pointcloud/legend.py), served as `PointCloud2.agent_encode_legend()`.

## Testing

Run the encoding tests from the repository root:

```bash
uv run pytest dimos/experimental/agent_encode
```

These tests check shapes, queries, grids, images, picking and the default overview
using synthetic inputs. They do not require a running robot or an LLM API key.
