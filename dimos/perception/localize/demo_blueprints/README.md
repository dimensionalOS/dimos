# Live perception and demo blueprints

Run the localize stack live on a robot's ports, or feed it a recording for
development without a robot.

| blueprint | rig | shape |
|---|---|---|
| `go2-localize` | Go2 camera, lidar, tf | `unitree-go2` navigation stack with `PerceptionLocalize` on the lidar and MCP |
| `xarm-feed` + `xarm-localize-live` | depth camera, tf | `PerceptionLocalize` with in-memory memory, the feed in a second process |

`PerceptionLocalize` in `../module.py` is the module a robot blueprint
takes. It consumes `color_image`, `camera_info`, `tf`, and the geometry its
`geometry` config names: `depth_image` aligned to the colour camera, or
`lidar`. It keeps bounded sensor history in RAM, embeds the colour feed as it
arrives, and answers `localize` from what it has embedded. It opens no file
and does not know whether its ports carry a robot or a replay. `go2-localize`
lives in `dimos/robot/unitree/go2/blueprints/smart/go2_localize.py`.

## go2-localize: real robot

Terminal 1, start the robot stack (replace the IP with your robot's address):

    uv run dimos --no-replay run go2-localize --robot-ip <robot-ip>

This extends `unitree-go2`, including its mapping, navigation, teleop through
`MovementManager` and viewer controls. Drive around using the Go2 controls while localization collects the
camera and lidar streams. `PerceptionLocalize` takes Go2's `lidar` and lifts
each call through that window's scans merged; Go2 supplies `camera_info` and
the camera TF chain in `world`. The blueprint passes `params=GO2_PARAMS` and
`mobile=True`.
`uv run dimos --replay run go2-localize` runs the same stack on a recording.

Terminal 2, attach to that same stack:

    uv run dimos shell
    app.PerceptionLocalize.state()
    app.PerceptionLocalize.localize("chair", -30.0, 30.0)
    app.PerceptionLocalize.localize("table,green plant", -60.0, 60.0)

Wait for `state()` to report ready after calibration and the first posed
embedded image arrive. Collect camera and lidar history while driving, then
query the window you want.

Terminal 3, use MCP while the same stack continues running:

    uv run dimos mcp list-tools
    uv run dimos mcp call state
    uv run dimos mcp call localize -j '{"objects": "chair", "start": -30, "duration": 30}' --timeout 300

## xarm-localize-live

Terminal 1, the sensors. One coordinator per bus is allowed, so the feed runs
as a plain process publishing on the same topics a robot connection would:

    uv run python -m dimos.perception.localize.demo_blueprints.xarm_feed

The recording is ten minutes. `--seek 427 --duration 76` plays only the window
`tools/tool_localize` uses in its reference command; `--speed`, `--dataset`
exist too.

Terminal 2, the blueprint. Start it before the feed if you want the weights
loaded while nothing is arriving:

    uv run dimos run xarm-localize-live

Its stages, in order:

    localize: loading siglip, owlv2, edgetam weights
    localize: waiting for camera_info
    localize: waiting for the first posed frame of the feed
    localize: ready on N embedded frames

Terminal 3:

    uv run dimos shell
    app.PerceptionLocalize.state()
    app.PerceptionLocalize.localize("roll of black tape,book,pen,red marker,yellow sticky notes", 0.0, 76.0)

With the seek above, that call names the reference window once the feed has
played the whole 76 s. Rerun shows the camera on the left and, on the right,
the tf axes, the camera frustum, and every verified instance as a labelled box
under `world/detections`.

Configuration of `PerceptionLocalize`, set on the blueprint:

- `world_frame`: the tf root the camera frame is resolved against.
- `geometry`: `"depth"` (default) or `"lidar"`. Only the selected input is
  subscribed; the other sensor is not required.
- `mobile`: `True` when the camera rides a walking base. The index then runs
  at 3 Hz instead of 1 Hz. `go2-localize` sets it.
- `horizon_s`: how far back `localize` can look. Sets the tf buffer and the
  memory windows.
- `feed_frames`: raw frames kept for the embed tail and the pairing.
- `embedder`, `detector`, `segmenter`: each a dandetect registry name or a
  `module:attr` path, see `../README.md`.
- `params`: the rig's `LocalizeParams`: the preset `XARM_PARAMS`,
  `DUAL_YAM_PARAMS` or `GO2_PARAMS` from `../localize_params.py`, or params
  built from one; `XARM_PARAMS` when not given. A call's `params` changes them
  for that call, see below and `../README.md`.

Memory: the raw feeds are kept raw for `feed_frames`. What `localize` reads is
at index rate: the embedded frames as JPEG, and the depth frame or lidar scan
paired with each of them as lz4. Both roll over `horizon_s`. A lidar call
lifts through its window's paired scans merged into one map.

### Other rigs

The module uses the incoming calibration and TF frame names. Select one
colour camera, its calibration and its aligned depth. For a G1 depth rig using
the topic names in `unitree-g1-record`, compose:

    from dataclasses import replace

    from dimos.perception.localize.localize_params import GO2_PARAMS
    from dimos.perception.localize.module import PerceptionLocalize

    localize = PerceptionLocalize.blueprint(
        mobile=True, params=replace(GO2_PARAMS, min_points=30)
    ).remappings([
        (PerceptionLocalize, "depth_image", "realsense_depth_image"),
        (PerceptionLocalize, "camera_info", "realsense_camera_info"),
    ])

A G1 walks, so `mobile=True`, and sees a room. `replace(GO2_PARAMS,
min_points=30)` are the params `tools/tool_localize` runs `g1_sf_office.db`
with: room scale, with a cloud floor for depth denser than a 5 cm lidar map.
This is a composition example; the folder adds a runnable blueprint only for
the xArm.

## The two skills

`state()` says ready or not, and what the module is doing if not. Until it
says ready, `localize` tells you which stage it is in instead of answering.

`localize(objects, start, duration, params)`.

`objects` is one label, or several split by commas. Several labels share one
detection pass.

`start` and `duration` are seconds, and they name the window this call looks
at. They work like `--from` and `--duration` on `tools/tool_localize`.
Positive `start` counts forward from the oldest retained embedded frame.
Negative counts back from the newest frame, like a negative Python index. The
default is the last ten seconds.

    app.PerceptionLocalize.localize("book", -30.0, 30.0)   # last 30 seconds
    app.PerceptionLocalize.localize("book", 5.0, 10.0)     # 10 seconds, from 5s in

The window is only what this call examines. What earlier calls proved is
remembered and still answered, so coverage builds up while the cost per call
does not. A wide window over time you have never asked about is the expensive
case: every frame in it needs a detector pass. The same window asked twice is
nearly free.

`params` changes the module's params for this one call. A preset name,
`xarm`, `dual_yam` or `go2`, switches to that preset. A JSON object sets
fields of `LocalizeParams` on the module's params, or on the preset its
`"base"` key names. The next call without `params` is back on the module's:

    app.PerceptionLocalize.localize("apple", -10.0, 10.0, "xarm")
    app.PerceptionLocalize.localize("book", -10.0, 10.0,
                                    '{"accept_score": 0.45, "verify_radius_m": 2.0}')
    app.PerceptionLocalize.localize("apple", -10.0, 10.0,
                                    '{"base": "xarm", "accept_score": 0.45}')

Useful in the shell:

    modules()                                # what is running
    rpcs("PerceptionLocalize")               # what you can call
    describe("PerceptionLocalize.localize")  # the signature and the docs

## Terminal

From any terminal, while a blueprint runs:

    uv run dimos status      # is it up
    uv run dimos stop        # stop it
    uv run dimos log         # its logs

## MCP

`go2-localize` and `xarm-localize-live` include `McpServer`, so an agent can
reach the two skills over MCP while either runs:

    uv run dimos mcp list-tools
    uv run dimos mcp call state
    uv run dimos mcp call localize -a 'objects=book,pen'
    uv run dimos mcp call localize -j '{"objects": "book", "start": -30, "duration": 30}'
    uv run dimos mcp call localize -j '{"objects": "apple", "params": "xarm"}'

### GPU limits for agents

Comma-separated labels share one detector pass, but they do not have constant
peak GPU cost. The segmenter masks every candidate box the detector produces,
so broad or overlapping labels can create a large mask batch and raise
`Remote torch.OutOfMemoryError`. This is especially easy when Rerun and the
rest of the perception stack already occupy the same GPU. The statement that
many labels cost about the same as one applies to detector passes, not peak
segmentation memory. There is not yet a measured safe label-count limit.

On an unmeasured hardware stack:

1. Use `dimos status` to confirm the intended stack and call `state` first.
   You may probe available VRAM and computational resources.
2. Start with one concrete label and the default ten-second window.
3. Add concrete, non-overlapping labels in small batches. Do not begin with a
   batch of broad labels or synonyms.
4. A shorter window reduces total work. Raising `candidate_floor` reduces
   detector proposals and recall. Neither establishes a safe peak-memory
   budget.
5. If a call reports `Remote torch.OutOfMemoryError`, retry with a twice
   smaller batch. Check stack state and leave any restart decision to the
   stack operator.

The client waits 30 seconds for an answer and then gives up. A wide window can
take longer than that. Raise the caller's timeout for each call that needs it:

    uv run dimos mcp call localize -a 'objects=chair' --timeout 300

Setting it on the blueprint does nothing. The timeout is on the side that
waits, not the side that answers.
