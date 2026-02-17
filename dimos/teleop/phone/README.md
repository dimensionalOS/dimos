# Phone Teleop

Teleoperation via smartphone motion sensors. Tilt to drive.

## Architecture

```
Phone Browser (DeviceOrientation + DeviceMotion)
    |
    |  TwistStamped + Bool via WebSocket
    v
Deno Bridge (teleop_server.ts)
    |
    |  LCM topics
    v
PhoneTeleopModule
    |  Orientation delta from home pose
    |  Gains -> velocity commands
    v
TwistStamped / Twist outputs
```

## Modules

### PhoneTeleopModule
Base module. Receives raw sensor data and button state. On engage (button hold), captures home orientation and publishes deltas as TwistStamped. Launches the Deno bridge server automatically.

### SimplePhoneTeleop
Filters to mobile-base axes: pitch -> linear.x, roll -> linear.y, yaw -> angular.z.

### PhoneGo2Teleop
Extends SimplePhoneTeleop. Adds `cmd_vel: Out[Twist]` for direct Go2 autoconnect wiring.

## Subclassing

Override these methods:

| Method | Purpose |
|--------|---------|
| `_handle_engage()` | Customize engage/disengage logic |
| `_should_publish()` | Add conditions for when to publish |
| `_get_output_twist()` | Customize twist computation |
| `_publish_msg()` | Change output format |

**Do not acquire `self._lock` in overrides.** The control loop already holds it.

## LCM Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/phone/sensors` | TwistStamped | linear=(roll,pitch,yaw) deg, angular=(gyro) deg/s |
| `/phone/button` | Bool | Teleop engage button (1=held) |
| `/teleop/twist` | TwistStamped | Output velocity command |

## Running

```bash
dimos run phone-go2-teleop     # Go2
dimos run simple-phone-teleop  # Generic ground robot
```

Server starts on port `8444`. Open `https://<host-ip>:8444` on phone, accept the self-signed certificate, allow sensor permissions, connect, hold button to drive.

## File Structure

```
phone/
├── phone_teleop_module.py   # Base phone teleop module
├── phone_extensions.py      # SimplePhoneTeleop, PhoneGo2Teleop
├── blueprints.py            # Pre-wired configurations
└── web/
    ├── teleop_server.ts     # Deno WSS-to-LCM bridge
    └── static/index.html    # Mobile web app
```
