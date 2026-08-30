# codecs

Encode/decode payloads for persistent storage. Codecs convert typed Python objects to `bytes` and back, used by backends that store observation data as blobs.

## Protocol

```python
class Codec(Protocol[T]):
    def encode(self, value: T) -> bytes: ...
    def decode(self, data: bytes) -> T: ...
```

## Built-in codecs

| Codec | Type | Notes |
|-------|------|-------|
| `PickleCodec` | Any Python object | Fallback. Uses `HIGHEST_PROTOCOL`. |
| `JpegCodec` | `Image` | Lossy compression via TurboJPEG. ~10-20x smaller. Preserves `frame_id` in header. |
| `LcmCodec` | `DimosMsg` subclasses | Uses `lcm_encode()`/`lcm_decode()`. Zero-copy for LCM message types. |
| `JsonCodec` | Pydantic models, dataclasses | **Opt-in** — never auto-selected. For derived records that must stay readable across releases and from other tools. |

### JsonCodec

A pickle is only readable by the exact Python classes that wrote it. Derived
records that outlive a release — belief, audit trails, anything another tool has
to read — should be stored as JSON instead:

```python
store.stream("belief_observation", BeliefObservation, codec="json")
```

The payload type must declare two things, checked when the codec is constructed
so an unreadable stream fails at open time rather than at decode time:

```python
@dataclass(frozen=True)
class BeliefObservation:
    __pydantic_config__ = ConfigDict(extra="forbid")   # 1. reject unknown fields
    schema_version: str                                 # 2. carry a version
    ...
```

`extra="forbid"` has to live on the type: `TypeAdapter` refuses a `config`
override for models and dataclasses, and silently dropping a field a newer
writer added would turn a schema mismatch into quiet data loss. Pass
`version_field=None` to waive the second requirement.

`codec="lz4+json"` works too, and `codec_for()` never picks `JsonCodec`
automatically — auto-selection would change the on-disk format of existing
streams.

## Auto-selection

`codec_for(payload_type)` picks the right codec:

```python
from dimos.memory.codecs import codec_for

codec_for(Image)        # → JpegCodec(quality=50)
codec_for(SomeLcmMsg)   # → LcmCodec(SomeLcmMsg)   (if has lcm_encode/lcm_decode)
codec_for(dict)         # → PickleCodec()            (fallback)
codec_for(None)         # → PickleCodec()
```

## Writing a new codec

1. Create `dimos/memory/codecs/mycodec.py`:

```python
class MyCodec:
    def encode(self, value: MyType) -> bytes:
        ...

    def decode(self, data: bytes) -> MyType:
        ...
```

2. Add a branch in `codec_for()` in `base.py` to auto-select it for the relevant type.

3. Add a test case to `test_codecs.py` — the grid fixture makes this easy:

```python
@pytest.fixture(params=[..., ("mycodec", MyCodec(), sample_value)])
def codec_case(request):
    ...
```

No base class needed — `Codec` is a protocol. Just implement `encode` and `decode`.
