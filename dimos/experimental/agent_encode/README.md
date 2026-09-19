# Experimental agent encoding

Agent-facing views of sensor messages. Message types keep a thin `agent_encode()`
entry point and an `AGENT_ENCODE_LEGEND`.

- `pointcloud/`: queries, fields, renders, picking. Entry: `PointCloud2.agent_encode()`.
- `odometry.py`: native-frame position, quaternion, linear/angular velocity. No
  frame transforms, no covariances. Entry: `odom.agent_encode()`.

# Point clouds

Overview: [`pointcloud/README.md`](pointcloud/README.md). Agent instructions:
`legend()` in [`pointcloud/runtime/dispatch.py`](pointcloud/runtime/dispatch.py), served as `PointCloud2.AGENT_ENCODE_LEGEND`.

# Odometry

```python
r = odom.agent_encode()  # position, orientation (quaternion), linear/angular velocity, frame ids
```

# Validation

```bash
uv run pytest dimos/experimental/agent_encode
uv run python dimos/evals/context_efficiency/tool_benchmark.py --offline --slice all --repeats 1
# paid (Pi + OPENAI_API_KEY):
uv run python dimos/evals/context_efficiency/tool_benchmark.py --slice all --repeats 1 --metric correctness --worktree "$PWD"
```
