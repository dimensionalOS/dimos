# PICO WebXR body tracking handoff

Research verified on 2026-08-24 with a PICO 4 Ultra and Motion Trackers.

## Goal

Add reusable body-joint poses to the existing WebXR teleoperation module without
changing its controller protocol or robot-facing behavior. The WebXR module
should gain one output:

```python skip
body_tracking: Out[BodyTrackingSnapshot]
```

The first implementation is an API test. It proves that PICO Browser can send
Motion Tracker body poses to dimOS. It does not retarget a robot or convert the
poses to SMPL.

## Verified native path

```text
PICO Motion Trackers
        |
        v
PICO Browser WebXR session with "body-tracking"
        |
        v
XRFrame.body: joint name -> XRSpace
        |
        v
XRFrame.getPose(jointSpace, bodyReferenceSpace)
        |
        v
named WebXR joint poses
```

The updated headset successfully displayed moving joints in the
[Babylon.js body-tracking testbed](https://drumath2237.github.io/babylon-body-tracking-testbed/).
Its implementation reads `XRFrame.body`, resolves each joint independently,
and keeps every pose that resolves. This behavior matters: one unavailable
joint must not discard the entire body frame.

The WebXR Body Tracking specification defines `body-tracking` as a session
feature for `immersive-ar` and `immersive-vr`. A session that did not receive
that feature returns `null` from `XRFrame.body`. See the
[initialization algorithm](https://immersive-web.github.io/body-tracking/#initialization)
and [privacy requirements](https://immersive-web.github.io/body-tracking/#privacy-security).

## Minimal API

Add a small device-neutral value type next to the existing WebXR controller
types:

```python skip
@dataclass(frozen=True)
class BodyTrackingSnapshot:
    capture_time_s: float
    frame_id: str
    joints: dict[str, Pose] | None
```

The values have these meanings:

| Value | Meaning |
|---|---|
| `joints is None` | `XRFrame.body` was unavailable. |
| `joints == {}` | A body source existed, but no joint pose resolved. |
| missing key | That individual joint did not resolve in this frame. |
| `frame_id` | The WebXR reference-space type used for every pose. |

Keep browser-provided joint names. Do not impose a fixed joint order or an
SMPL skeleton in this module. A downstream adapter can map these reusable poses
to SONIC, SMPL, visualization, recording, or another consumer.

## Integration approach

Preserve the existing binary WebSocket messages:

```text
PoseStamped / Joy -> existing decoder -> existing controller outputs
```

Add a separate JSON body-snapshot message on the same WebSocket. The server
accepts binary controller messages and this one text-message type. It publishes
decoded snapshots through `body_tracking`; it does not combine viewer,
controller, hand, and body data into an aggregate frame.

Capture body poses in the existing WebXR animation loop:

1. Request `body-tracking` as an optional feature for normal teleoperation.
2. When `require_body_tracking=True`, require the feature in `immersive-ar` and
   report session-request failure instead of falling back to VR.
3. Request `bounded-floor` for body poses and fall back to the existing
   `local-floor` space when unavailable. Keep controller poses in their current
   `local-floor` space.
4. Iterate the map returned by `frame.body`.
5. Call `frame.getPose(jointSpace, bodyReferenceSpace)` for each joint.
6. Omit only the joints whose poses do not resolve.
7. Send snapshots at the existing tracking cadence. In required mode, also
   send `joints: null` so the API-test monitor can distinguish a live session
   from one that has produced no body messages.

Use one module setting:

```python skip
require_body_tracking: bool = False
```

The default keeps body tracking optional for every existing blueprint. A
dedicated PICO API-test blueprint sets it to `True` and connects
`body_tracking` to a small health monitor over `pLCMTransport`.

## Acceptance checks

- Existing controller `PoseStamped`, `Joy`, engagement, timeout, and robot
  output tests continue to pass unchanged.
- A body JSON message produces one `BodyTrackingSnapshot` with the same joint
  names, positions, orientations, capture time, and reference-space ID.
- Null, empty, partially resolved, and malformed body messages have explicit
  tests.
- Normal teleoperation requests body tracking optionally and retains its
  current AR-to-VR fallback.
- Required mode requests `immersive-ar` with `body-tracking` required and shows
  the browser rejection when the feature cannot be granted.
- The API-test blueprint logs snapshot rate, joint count, and selected joint
  positions from the physical PICO setup.

## Out of scope

- `OperatorTrackingFrame` or another aggregate operator schema
- controller protocol replacement
- fixed joint arrays or emulated-position quality metadata
- SMPL/SONIC conversion or robot retargeting
- production recording and visualization
