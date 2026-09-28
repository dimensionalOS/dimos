# SIYI A8 mini gimbal

`SiyiA8Gimbal` publishes the gimbal tf chain, the camera intrinsics and aim requests.

## How it works

- Reads `gimbal_attitude` (JointState, radians) and publishes
  `base_link > gimbal_base > gimbal_link > a8_optical` on `tf` at 10 Hz.
- Publishes `camera_info` at 1 Hz.
- `ip` is the camera's address for the SIYI SDK (UDP 37260). It has no default; unset means
  the SDK is off. With `ip` set the module polls the zoom and withholds `camera_info` while
  the zoom is unknown or not 1x.
- With `aim_enabled=True` it turns `target_los` into `gimbal_target` aim requests.
- It opens no MAVLink socket. The module that owns the MAVLink link reads `gimbal_target`
  and commands the gimbal.
- The A8 reports yaw relative to the body and pitch stabilised to the earth. The mount
  preset (`frame.py`) is applied by the module that decodes the MAVLink attitude.

| File | Holds |
|---|---|
| `gimbal.py` | the module |
| `frame.py` | frame maths: mount presets, limits, attitude normalisation |
| `sdk.py` | SIYI SDK packets and client: the zoom query |
| `replay.py` | a `gimbal_attitude` sample for tests and simulators |

## Run

```bash
dimos run siyi-a8-gimbal
dimos run siyi-a8-gimbal --siyia8gimbal.ip=<camera>    # with the SIYI SDK
```

Images must carry the frame of `camera_info` and the tf chain: run the camera with
`--rtspcamera.frame_id=a8_optical`, or set `optical_frame_id` to the camera's `frame_id`.

RPCs: `aim(pitch_deg, yaw_deg)`, `state()`.

## Setup

Set `mount_xyz`, the offset from `base_link` to the gimbal base, for your airframe
(metres, FLU; default 0, 0, 0).

## Test

No gimbal needed.

```bash
uv run pytest dimos/hardware/gimbal/siyi
```
