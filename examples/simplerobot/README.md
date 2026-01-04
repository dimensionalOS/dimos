# Simple Robot

A virtual 2D robot for testing dimos integrations.

Subscribes to `/cmd_vel` (Twist) and publishes `/odom` (PoseStamped).

## Run

```bash
python simplerobot.py
```

Options:
- `--headless` - disable pygame visualization
- `--selftest` - run demo movements

## Visualization

Uses pygame to show robot position and heading. Press `q` or `Esc` to quit.
