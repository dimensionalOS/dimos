# Experimental agent encoding

Agent-facing views of sensor messages. Message types keep a thin `agent_encode()`
entry point and an `AGENT_ENCODE_LEGEND`.

- `pointcloud/`: queries, fields, renders, picking. Entry: `PointCloud2.agent_encode()`.
- `odometry.py`: native-frame position, quaternion, linear/angular velocity. No
  frame transforms, no covariances. Entry: `odom.agent_encode()`.

## Point clouds

Overview: [`pointcloud/README.md`](/dimos/experimental/agent_encode/pointcloud/README.md). Agent instructions:
`legend()` in [`pointcloud/runtime/dispatch.py`](/dimos/experimental/agent_encode/pointcloud/runtime/dispatch.py#L273), served as `PointCloud2.AGENT_ENCODE_LEGEND`.

## Odometry

Returns pose and velocity with explicit units and frame IDs. Position and
orientation describe the child frame in the parent frame; velocities are in the
child frame. Covariances are omitted.

```python
from dimos.msgs.nav_msgs.Odometry import Odometry

odom = Odometry(frame_id="map", child_frame_id="base_link")
print(odom.agent_encode())
```

## Testing

Run the encoding tests from the repository root:

```bash
uv run pytest dimos/experimental/agent_encode
```

These tests check geometry queries, fields, rendering, picking, response budgets,
and odometry serialization using synthetic inputs. They do not require a running
robot or an LLM API key.
