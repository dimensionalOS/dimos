# go2-localize-live

This blueprint runs the perception memory stack live. It takes the `go2_short`
recording and replays it into a fresh store forever, lap after lap, with the
clock always moving forward. A background tail embeds the frames as they
arrive, so the store keeps growing like a real robot's recorder would. You then
ask it where objects are, and it answers from what it has seen so far. The
answers show up in rerun as labelled boxes, on top of the lidar map. Nothing is
written back to the original recording.

Start it in terminal 1:

    uv run dimos run go2-localize-live

Wait for this line before you ask it anything:

    localize: ready on N embedded frames

It takes about 15 seconds. Most of that is loading SigLIP, OWLv2 and EdgeTAM.

## REPL

Open terminal 2:

    uv run dimos shell

Then:

    modules()                            # what is running
    rpcs("LocalizeModule")               # what you can call
    describe("LocalizeModule.localize")  # the signature and the docs

    app.LocalizeModule.state()           # ready or not, and what it is doing
    app.LocalizeModule.localize("chair")
    app.LocalizeModule.localize("chair,table,green plant")

`localize(objects, start, duration, policy)`.

`objects` is one label, or several split by commas. Several labels share one
detection pass, so 16 labels cost about the same as one.

`start` and `duration` are seconds, and they name the window this call looks
at. They work like `--from` and `--duration` on `tool_localize`. Positive
`start` counts forward from the beginning of the feed. Negative counts back
from the newest frame, like a negative Python index. The default is the last
ten seconds.

    app.LocalizeModule.localize("chair", -30.0, 30.0)   # last 30 seconds
    app.LocalizeModule.localize("chair", 5.0, 10.0)     # 10 seconds, from 5s in

The window is only what this call examines. What earlier calls proved is
remembered and still answered, so coverage builds up while the cost per call
does not.

A wide window over time you have never asked about is the expensive case. Every
frame in it needs a detector pass. The same window asked twice is nearly free.

`policy` is a JSON object. It overrides any field of `LocalizePolicy` for this
one call:

    app.LocalizeModule.localize("chair", -10.0, 10.0,
                                '{"accept_score": 0.45, "verify_radius_m": 2.0}')

## Terminal

From any terminal, while it runs:

    uv run dimos status      # is it up
    uv run dimos stop        # stop it
    uv run dimos log         # its logs

## MCP

The same two calls are skills, so an agent can reach them over MCP:

    uv run dimos mcp list-tools
    uv run dimos mcp call state
    uv run dimos mcp call localize -a 'objects=chair,table'
    uv run dimos mcp call localize -j '{"objects": "chair", "start": -30, "duration": 30}'

### GPU limits for agents

Comma-separated labels share one OWLv2 detector pass, but they do not have
constant peak GPU cost. EdgeTAM segments every candidate box produced by the
detector, so broad or overlapping labels can create a large mask batch and
raise `Remote torch.OutOfMemoryError`. This is especially easy when Rerun and
the rest of the perception stack already occupy the same GPU. The statement
that 16 labels cost about the same as one applies to detector passes, not peak
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

Ask `state` first. Until it says ready, `localize` tells you which stage it is
in instead of answering.
