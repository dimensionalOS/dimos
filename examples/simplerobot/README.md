# SimpleRobot

A minimal virtual robot for testing and development. It implements some of the same LCM interface as real robots, making it ideal for testing third-party integrations (see `examples/language-interop/`) or experimeting with dimos Module patterns

## Interface

| Topic      | Type          | Direction | Description                             |
|------------|---------------|-----------|-----------------------------------------|
| `/cmd_vel` | `Twist`       | Subscribe | Velocity commands (linear.x, angular.z) |
| `/odom`    | `PoseStamped` | Publish   | Current pose at 30Hz                    |

Physical robots typically publish multiple poses in a relationship as `TransformStamped` in a TF tree, while SimpleRobot publishes `PoseStamped` directly for simplicity.

For details on this check [Transforms](/docs/usage/transforms.md)

## Usage

```bash
# With pygame visualization
python examples/simplerobot/simplerobot.py

# Headless mode
python examples/simplerobot/simplerobot.py --headless

# Run self-test demo
python examples/simplerobot/simplerobot.py --headless --selftest
```

Use `lcmspy` in another terminal to inspect messages. Press `q` or `Esc` to quit visualization.

## Sending Commands

From any language with LCM bindings, publish `Twist` messages to `/cmd_vel`:

```python
from dimos.core import LCMTransport
from dimos.msgs.geometry_msgs import Twist

transport = LCMTransport("/cmd_vel", Twist)
transport.publish(Twist(linear=(0.5, 0, 0), angular=(0, 0, 0.3)))
```

See `examples/language-interop/` for C++, TypeScript, and Lua examples.

## Physics

SimpleRobot uses a 2D unicycle model:
- `linear.x` drives forward/backward
- `angular.z` rotates left/right
- Commands timeout after 0.5s (robot stops if no new commands)
