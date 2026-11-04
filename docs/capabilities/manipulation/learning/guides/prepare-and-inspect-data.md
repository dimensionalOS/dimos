# Prepare and inspect a recording

Start with a complete dimOS recording directory containing `schema.json` and its
MCAP or SQLite payload. This workflow produces a local LeRobot dataset; it does
not require a running robot. Copy the whole recording directory if conversion
runs on another machine.

## Inspect and convert

After stopping collection, inspect the recording and export its saved episodes:

```bash skip
dimos imitation inspect recordings/session-001
dimos imitation prepare recordings/session-001 --output datasets/session-001
dimos imitation inspect datasets/session-001
```

`inspect` prints a human-readable summary for either a recording or a prepared
dataset. Recordings show episode totals, stream message counts, and quality
metrics with units. Failed and incomplete episodes are always listed. Quality
checks describe the data, not whether the robot completed the physical task.

Use `--verbose` to see every assessed episode, or `--json` for the complete
machine-readable result with unrounded values:

```bash skip
dimos imitation inspect recordings/session-001 --verbose
dimos imitation inspect recordings/session-001 --json
```

Dataset summaries show frame counts, rates, episode lengths, feature shapes and
types, and statistics availability. Redirecting output keeps the readable
format; use `--json` explicitly for scripts. `--json` takes precedence over
`--verbose` when both are supplied.

Preparation requires a new output directory. Without `--output`, it writes to
`~/.local/state/dimos/datasets/<recording-directory-name>` by default.

Before training, view an episode in Rerun:

```bash skip
dimos imitation visualize datasets/session-001 --episode 0
```

The viewer shows camera images, joint states, and actions on a shared timeline.
Use its playback controls to play, pause, and scrub through the episode. Episode
indices start at zero; omitting `--episode` selects the first episode.

Visualization requires a local graphical display and a prepared LeRobot dataset,
not a raw recording. It runs LeRobot's existing viewer in the isolated Python
environment and streams loading progress to the terminal. Unlike inspection,
visualization decodes the episode's frames, so loading can take longer. Dataset
loading is local-only; missing files are not downloaded from Hugging Face.

## Resolve preparation failures

Use the reported episode and feature to locate the problem. Check that image
shapes and joint names match the saved schema and that required source samples
exist. Command streams need an initial target for every declared joint. Do not
substitute measured state for missing commands.

Use a new output path when the target directory already exists. For source
semantics, alignment defaults, and export keys, see
[Recordings and profiles](/docs/capabilities/manipulation/learning/reference/recordings-and-profiles.md).

Preparation imports recorded message types. Prepare trusted recordings and
install custom message packages in the conversion environment.
