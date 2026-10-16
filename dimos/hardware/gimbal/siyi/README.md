# SIYI A8 mini gimbal

`SiyiA8Gimbal` publishes the gimbal tf chain, the camera intrinsics and aim requests. It
opens no MAVLink socket: the module that owns the MAVLink link (`Px4DroneConnection`)
relays the attitude in and the aim requests out.

- Reads `gimbal_attitude` (JointState, radians) and publishes
  `base_link > gimbal_base > gimbal_link > a8_optical` on `tf` at 10 Hz, and `camera_info`
  at 1 Hz.
- `ip`: the camera's address for the SIYI SDK (UDP 37260); unset, the SDK is off. With it
  set the module polls the zoom and withholds `camera_info` unless the zoom reads 1x.
- `aim_enabled=True` turns `target_los` into `gimbal_target` aim requests, at most `aim_hz` (10).
- `mount_xyz`: the offset from `base_link` to the gimbal base (metres, FLU). Measure it on
  your airframe; the module warns while it is 0, 0, 0.
- The A8 reports yaw relative to the body and pitch stabilised to the earth.

RPCs: `aim(pitch_deg, yaw_deg)`, `state()`.

## Run

```bash
dimos run siyi-a8-gimbal --siyia8gimbal.ip=<camera>
```

Images must carry the frame of `camera_info`: run the camera with
`--rtspcamera.frame-id=a8_optical`.

## Test

```bash
uv run pytest dimos/hardware/gimbal/siyi
```
