# Add a collection blueprint

Use this guide to record a robot supplied by your own Python package.

## 1. Declare the profile

Profiles have no separate registry. `dimos run` discovers Blueprints through the
built-in registry or installed `dimos.blueprints` entry points. The Blueprint
passes a profile to its recorder; the profile name is recording metadata, not
a Blueprint lookup key. Profile validation checks declarations and shared-source
consistency. Recorder wiring checks required inputs; preparation validates the
actual recorded values.

One `CollectionProfile` declares the typed source streams and their dataset
interpretation. A `FeatureSpec` adds a Python `message_type` to the
dataprep feature fields: `stream`, `source_kind`, `field`, `dtype`, `shape`, and `names`.
Several features can project different fields or joint subsets from one source;
the recorder captures that source once.

## 2. Compose the recording stack

Follow the executable [collection API examples](/dimos/imitation/collection/README.md)
to declare a profile. Compose the recorder before producers in `autoconnect()`
so it stops after them. Include your robot, camera producers, and an
`EpisodeMonitorModule.blueprint(instance_name="episodes")` supplying status.

The `collect(profile=...)` factory returns an ordinary blueprint with typed inputs before autoconnect.
It accepts optional `recording=Path(...)` and `format="mcap" | "sqlite"`
blueprint defaults. The output directory can instead be supplied through run
configuration. No recorder subclass is needed.

The Python graph chooses camera producers, devices, and transports. An image
feature does **not** construct a webcam. Add any number of camera features and
matching producers, using normal blueprint remappings when names differ.
Source names must be nonreserved Python identifiers, and message classes must
be importable and support native LCM encoding. The recorder also requires the
reserved `status: In[String]` input carrying a versioned JSON episode document.

## 3. Export and launch the blueprint

Export the blueprint using an installed package entry point:

```toml
# pyproject.toml, for a distribution named vendor-robot
[project.entry-points."dimos.blueprints"]
collect = "vendor_robot.collection:collect"
```

```bash skip
dimos run vendor-robot.collect --daemon \
  --recorder.recording recordings/session-001 \
  --episodes.task "pick up the cup"
dimos imitation control
```

## Use another episode controller

No imitation workflow registration is needed. The TUI uses
`Dimos.connect().find_module_by_spec(EpisodeControlSpec)`. External controllers
can implement the same typed RPCs instead of subclassing our episode monitor.
Exactly one implementation must match; missing or ambiguous matches are errors.
The controller class must be importable in the client.

See [Recordings and profiles](/docs/capabilities/manipulation/learning/reference/recordings-and-profiles.md) for saved schema and export semantics.
