from pathlib import Path

import pytest

from dimos.memory.codecs.cdr import CdrCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.mcap import McapStore
from dimos.messages.std_msgs.msg import String


@pytest.mark.parametrize("compressed", [False, True])
def test_schema_declared_cdr_channel_decodes_without_importing_metadata(
    tmp_path: Path, compressed: bool
) -> None:
    from mcap.writer import Writer

    codec = Lz4Codec(CdrCodec(String)) if compressed else CdrCodec(String)
    path = tmp_path / "generated.mcap"
    with path.open("wb") as output:
        writer = Writer(output)
        writer.start()
        schema = writer.register_schema(
            name="std_msgs/msg/String", encoding="ros2msg", data=b"string data\n"
        )
        channel = writer.register_channel(
            topic="custom/name",
            message_encoding="lz4+cdr" if compressed else "cdr",
            schema_id=schema,
            metadata={"dimos.payload_type": "untrusted_module.Payload"},
        )
        writer.add_message(
            channel_id=channel,
            log_time=1_000_000_000,
            publish_time=1_000_000_000,
            data=codec.encode(String(data="exact payload")),
        )
        writer.finish()
    with McapStore(path=str(path)) as store:
        message = store.stream(store.list_streams()[0]).order_by("ts").first().data
        assert type(message) is String
        assert message.data == "exact payload"
