# EpisodeStatus

Add `dimos/message_codegen/schemas/dimos_msgs/msg/EpisodeStatus.msg`:

```text
float64 ts
string state
int64 episodes_saved
int64 episodes_discarded
string last_event "init"
string[<=1] task_label
```

Generate and commit the definition together with the generated source:

```sh
uv run python -m scripts.generate_builtin_messages
uv run python -m scripts.generate_builtin_messages --check
```

Use the generated Python value from the editable checkout or matching message wheel:

```python
from dimos_generated.dimos_msgs.msg import EpisodeStatus

status = EpisodeStatus(ts=17.25, state="recording", episodes_saved=2,
                       episodes_discarded=1, last_event="start", task_label=["inspection"])
assert EpisodeStatus.decode(status.encode()) == status
```

`ts` is seconds. `task_label=[]` represents None; `[""]` preserves an empty label.
Counters use signed 64-bit storage. Applications enforce finite timestamps and
`state` (`idle/recording`) and `last_event` (`start/save/discard/init`) values.
Generated values have ordinary message defaults, rather than Pydantic's required
input/coercion rules. The source model's JSON `schema_version` envelope is not a
message field; adding this type does not migrate the robot-learning collector.

The fields follow the [EpisodeStatus model](https://github.com/dimensionalOS/dimos/blob/ef5e5f2710c482fb1d43d43ec50fd73b0fbe1dde/dimos/imitation/collection/episode.py).
See the [built-in message guide](/docs/development/messages-in-repository.md) for
checkout prerequisites and the existing package/consumer workflow.
