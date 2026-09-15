# Perception memory demo blueprints

Two ways to run the perception memory stack on a stream, both fed from a
recording so they work without a robot.

| blueprint | rig | shape |
|---|---|---|
| `xarm-feed` + `xarm-localize-live` | depth camera, tf | port-only module with in-memory memory, feed in a second process |
| `go2-localize-live` | lidar, odom | one blueprint, feeder writes a SQLite store, module reads it |

`LiveLocalizeModule` in `live_localize.py` is the module a robot blueprint
takes. It consumes `color_image`, `depth_image`, `camera_info` and `tf`, keeps a
bounded memory in RAM, embeds the colour feed as it arrives, and answers
`localize` from what it has embedded. It opens no file and does not know
whether its ports carry a robot or a replay. `go2_localize_live.py` is the
older store-backed shape and is kept as the lidar example.

## xarm-localize-live

Terminal 1, the sensors. One coordinator per bus is allowed, so the feed runs
as a plain process publishing on the same topics a robot connection would:

    uv run python -m dimos.perception.memory.demo_blueprints.xarm_feed

The recording is ten minutes. `--seek 427 --duration 76` plays only the window
`tool_localize` uses in its reference command; `--speed`, `--dataset` exist too.

Terminal 2, the blueprint. Start it before the feed if you want the weights
loaded while nothing is arriving:

    uv run dimos run xarm-localize-live

Its stages, in order:

    localize: loading SigLIP, OWLv2 and EdgeTAM weights
    localize: waiting for camera_info
    localize: waiting for the first posed frame of the feed
    localize: ready on N embedded frames

Terminal 3:

    uv run dimos shell
    app.LiveLocalizeModule.state()
    app.LiveLocalizeModule.localize("roll of black tape,book,pen,red marker,yellow sticky notes", 0.0, 76.0)

With the seek above, that call names the reference window once the feed has
played the whole 76 s, and its six answers match `tool_localize` on the same
recording. Rerun shows the camera on the left and, on the right, the tf axes,
the camera frustum, every verified instance as a labelled box under
`world/detections`, and its sightings as image-textured points under
`world/hit_points`.

Configuration of `LiveLocalizeModule`, set on the blueprint:

- `world_frame`: the tf root the camera frame is resolved against.
- `mobile`: room-scale policy and the walking embed rate, off for a parked camera.
- `horizon_s`: how far back `localize` can look. Sets the tf buffer and the
  memory windows.
- `feed_frames`: raw frames kept for the embed tail and the depth pairing.

Memory: the raw feeds are kept raw for `feed_frames`. What `localize` reads is
at index rate: the embedded frames as JPEG, and the depth frame paired with
each of them as lz4, both rolling over `horizon_s`.

## go2-localize-live

Replays `go2_short` into a fresh store forever, lap after lap, with the clock
always moving forward. A background tail embeds the frames as they arrive.
Answers show up in rerun as labelled boxes on top of the lidar map. Nothing is
written back to the original recording.

    uv run dimos run go2-localize-live

Wait for `localize: ready on N embedded frames`, about 15 seconds. Then:

    uv run dimos shell
    app.LocalizeModule.state()
    app.LocalizeModule.localize("chair,table,green plant")

## The two skills

Both modules expose the same two skills.

`state()` says ready or not, and what the module is doing if not. Until it
says ready, `localize` tells you which stage it is in instead of answering.

`localize(objects, start, duration, policy)`.

`objects` is one label, or several split by commas. Several labels share one
detection pass.

`start` and `duration` are seconds, and they name the window this call looks
at. They work like `--from` and `--duration` on `tool_localize`. Positive
`start` counts forward from the first embedded frame. Negative counts back from
the newest frame, like a negative Python index. The default is the last ten
seconds.

    app.LiveLocalizeModule.localize("book", -30.0, 30.0)   # last 30 seconds
    app.LiveLocalizeModule.localize("book", 5.0, 10.0)     # 10 seconds, from 5s in

The window is only what this call examines. What earlier calls proved is
remembered and still answered, so coverage builds up while the cost per call
does not. A wide window over time you have never asked about is the expensive
case: every frame in it needs a detector pass. The same window asked twice is
nearly free.

`policy` is a JSON object. It overrides any field of `LocalizePolicy` for this
one call:

    app.LiveLocalizeModule.localize("book", -10.0, 10.0,
                                    '{"accept_score": 0.45, "verify_radius_m": 2.0}')

Useful in the shell:

    modules()                                # what is running
    rpcs("LiveLocalizeModule")               # what you can call
    describe("LiveLocalizeModule.localize")  # the signature and the docs

## Terminal

From any terminal, while a blueprint runs:

    uv run dimos status      # is it up
    uv run dimos stop        # stop it
    uv run dimos log         # its logs

## MCP

Both blueprints include `McpServer`, so an agent can reach the two skills over
MCP while either runs:

    uv run dimos mcp list-tools
    uv run dimos mcp call state
    uv run dimos mcp call localize -a 'objects=book,pen'
    uv run dimos mcp call localize -j '{"objects": "book", "start": -30, "duration": 30}'

### GPU limits for agents

Comma-separated labels share one OWLv2 detector pass, but they do not have
constant peak GPU cost. EdgeTAM segments every candidate box produced by the
detector, so broad or overlapping labels can create a large mask batch and
raise `Remote torch.OutOfMemoryError`. This is especially easy when Rerun and
the rest of the perception stack already occupy the same GPU. The statement
that many labels cost about the same as one applies to detector passes, not
peak segmentation memory. There is not yet a measured safe label-count limit.

On an unmeasured hardware stack:

1. Use `dimos status` to confirm the intended stack and call `state` first.
   You may probe available VRAM and computational resources.
2. Start with one concrete label and the default ten-second window.
3. Add concrete, non-overlapping labels in small batches. Do not begin with a
   batch of broad labels or synonyms.
4. A shorter window reduces total work. Raising `candidate_floor` reduces
   detector proposals and recall. Neither establishes a safe peak-memory
   budget.
5. If a call reports `Remote torch.OutOfMemoryError`, retry with a twice smaller batch.
   Check stack state and leave any restart decision to the stack operator.

The client waits 30 seconds for an answer and then gives up. A wide window can
take longer than that. The wait belongs to the caller, so raise it there. Per
call, or for every call you make:

    uv run dimos mcp call localize -a 'objects=chair' --timeout 300
    uv run dimos --mcp-timeout 300 mcp call localize -a 'objects=chair'
    MCP_TIMEOUT=300 uv run dimos mcp call localize -a 'objects=chair'

Setting it on the blueprint does nothing. The timeout is on the side that
waits, not the side that answers.
