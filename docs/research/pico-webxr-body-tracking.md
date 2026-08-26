# PICO WebXR body tracking handoff

Research verified on 2026-08-24 with a PICO 4 Ultra and Motion Trackers.

## Goal

The WebXR teleoperation module exposes reusable body-joint poses without
changing its controller protocol or robot-facing behavior. It provides one
output:

```python skip
body_tracking: Out[BodyTrackingSnapshot]
```

The first consumer is an API-test monitor. It proves that PICO Browser can send
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

The module publishes a small, device-neutral value type:

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

Body snapshots use a separate JSON message on the same WebSocket. The server
accepts binary controller messages and this one text-message type. It publishes
decoded snapshots through `body_tracking`; it does not combine viewer,
controller, hand, and body data into an aggregate frame.

Capture body poses in the existing WebXR animation loop:

1. Leave body tracking off for normal teleoperation so existing sessions do
   not request access to privacy-sensitive body data.
2. In optional mode, request `body-tracking` without changing the existing
   AR-to-VR fallback. In required mode, require the feature in `immersive-ar`
   and report session-request failure instead of falling back to VR.
3. Request `bounded-floor` for body poses and fall back to the existing
   `local-floor` space when unavailable. Keep controller poses in their current
   `local-floor` space.
4. Iterate the map returned by `frame.body`.
5. Call `frame.getPose(jointSpace, bodyReferenceSpace)` for each joint.
6. Omit only the joints whose poses do not resolve.
7. Convert the XR animation-frame timestamp to Unix seconds with
   `performance.timeOrigin`, then send snapshots at the existing tracking
   cadence. In required mode, also send `joints: null` so the API-test monitor
   can distinguish a live session from one that has produced no body messages.

Use one explicit module setting:

```python skip
body_tracking_mode: Literal["off", "optional", "required"] = "off"
```

| Mode | Session behavior | Missing body source |
|---|---|---|
| `off` | Do not request `body-tracking`; preserve existing teleoperation. | No body messages. |
| `optional` | Request the feature and preserve AR-to-VR fallback. | No body messages until a source exists. |
| `required` | Require the feature in `immersive-ar`; do not fall back to VR. | Send `joints: null` heartbeats. |

The `off` default avoids a new consent prompt in every existing WebXR
blueprint. The dedicated `demo-pico-body-tracking` blueprint selects
`required` and connects `body_tracking` to a live health monitor over
`pLCMTransport`. Run it with:

```bash
dimos run demo-pico-body-tracking
```

The monitor declares success when it receives any non-empty joint map. It logs
the snapshot rate, reference space, joint count, and selected joint positions
without requiring a fixed PICO skeleton.

## Acceptance checks

- Existing controller `PoseStamped`, `Joy`, engagement, timeout, and robot
  output tests continue to pass unchanged.
- A body JSON message produces one `BodyTrackingSnapshot` with the same joint
  names, positions, orientations, capture time, and reference-space ID.
- Null, empty, partially resolved, and malformed body messages have explicit
  tests.
- Normal teleoperation leaves body tracking off and retains its current
  AR-to-VR fallback without requesting new consent.
- Optional mode requests body tracking while retaining AR-to-VR fallback and
  suppresses null snapshots.
- Required mode requests `immersive-ar` with `body-tracking` required and shows
  the browser rejection when the feature cannot be granted.
- The API-test blueprint logs success after the first resolved joint, then
  reports snapshot rate, joint count, and selected joint positions from the
  physical PICO setup.

## Out of scope

- `OperatorTrackingFrame` or another aggregate operator schema
- controller protocol replacement
- fixed joint arrays or emulated-position quality metadata
- SMPL/SONIC conversion or robot retargeting
- production recording and visualization
