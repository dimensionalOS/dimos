# EpisodeStatus

Add `packages/dimos-generated/custom/dimos_msgs/msg/EpisodeStatus.msg`:

```text
float64 ts
string state
int64 episodes_saved
int64 episodes_discarded
string last_event "init"
string[] task_label
```

Add the type to `packages/dimos-generated/catalog.json`, then prepare and generate locally. Commit the definition and catalog; generated output stays ignored:

```sh
uv run python -m scripts.prepare_builtin_sources
uv run python -m scripts.generate_builtin_messages
uv run python -m scripts.generate_builtin_messages --check
```

Use the generated Python value from the editable checkout or matching message wheel:

```python
from dimos_generated.dimos_msgs.msg import EpisodeStatus
from dimos_cdrgen.registry import encode, decode

status = EpisodeStatus(ts=17.25, state="recording", episodes_saved=2,
                       episodes_discarded=1, last_event="start", task_label=["inspection"])
assert len(status.task_label) <= 1
assert decode(encode(status), EpisodeStatus) == status
```

`ts` is seconds. `task_label=[]` represents None; `[""]` preserves an empty label.
The published Python backend cannot encode bounded sequences. The unbounded sequence
has the same CDR bytes; applications enforce zero or one label before encoding or after decoding.
Counters use signed 64-bit storage. Applications enforce finite timestamps and
`state` (`idle/recording`) and `last_event` (`start/save/discard/init`) values.
Generated values require explicit fields and retain their wire values without Pydantic coercion. The source model's JSON `schema_version` envelope is not a
message field; adding this type does not migrate the robot-learning collector.

The fields follow the [EpisodeStatus model](https://github.com/dimensionalOS/dimos/blob/ef5e5f2710c482fb1d43d43ec50fd73b0fbe1dde/dimos/imitation/collection/episode.py).
See the [built-in message guide](/docs/development/messages-in-repository.md) for
checkout prerequisites and the existing package/consumer workflow.
