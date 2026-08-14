# VLN-CE R2R evaluation

This package runs one official VLN-CE R2R episode through the standard
`dimos eval run` command. The checked-in development case uses Habitat's public
`17DRP5sb8fy` MP3D example and official R2R/VLN-CE training episode 515.

The case measures a live agent. Pi remains in the runtime loop for the full
episode and can repeatedly inspect Memory2 observations and call ordinary DimOS
RPCs through `python_exec`. The episode and Pi share a 600-second wall-clock
horizon. The agent ends the route with
`app.VlnceConnection.submit_route()`; point-to-point navigation completion does
not submit the route.

The run specification pins the model and thinking level. The runner passes only
`OPENAI_API_KEY` to Pi and strips credentials from the Python kernel. The
Habitat container runs without network access.

## Run the public case

Install the agent dependencies and build the Pi extension first:

```bash
uv sync --extra agents
npm --prefix packages/pi-code-policy-extension install
npm --prefix packages/pi-code-policy-extension run build
```

Then run:

```bash
uv run dimos eval run \
  dimos/benchmark/vlnce_r2r/cases/mp3d-example-episode-515/evaluation.json \
  --output /tmp/dimos-vlnce-r2r
```

The first run downloads about 68 MB of scene data and 2.6 MB of episode data.
Preparation verifies the pinned archives and required files before launching
the container. Later runs reuse the content-addressed cache.

The evaluator always asks Habitat to record `native-render.mp4`. Rendering is
presentation evidence: a renderer failure appears in `native-render.v1.json`
but does not change official scoring. The container writes the authoritative
`terminal-private/vlnce-result.v1.json`; the host checks its strict result shape
and attempt identity without rescoring it.

## Interpret the result

The condition label is `dimos_geometry_training_scene_development`. The agent
receives the complete public navigability map, RGB and depth observations,
odometry, and DimOS navigation tools. These results are development results on
a training scene, not standard VLN-CE validation, test, or leaderboard results.

The repository stores URLs, identities, instructions, and checksums. It does
not redistribute scene meshes, episode archives, semantic annotations,
reference paths, or trained models. VLN-CE code is MIT licensed; R2R and
Matterport3D-derived assets have separate terms. See
[VLN-CE's license](https://github.com/jacobkrantz/VLN-CE/blob/master/LICENSE)
and the [Matterport terms](https://niessner.github.io/Matterport/).
