"""A separately generated message package records through the actual native process."""

from pathlib import Path
import subprocess
import sys

from dimos_cdrgen.source_package import prepare
import pytest


@pytest.mark.native_e2e
def test_external_generated_schema_survives_native_mcap_round_trip(tmp_path: Path):
    executable = Path(__file__).resolve().parents[3] / "target/debug/dimos-memory-recorder"
    assert executable.is_file(), "Build the native recorder offline before this acceptance test"
    project = tmp_path / "external"
    source = project / "interfaces/acme_msgs/msg/Event.msg"
    source.parent.mkdir(parents=True)
    source.write_text("uint32 count\nstring label\n")
    (project / "pyproject.toml").write_text(
        '[project]\nname="acme_messages"\nversion="1.0.0"\n[tool.dimos.messages]\nsource="interfaces"\nmodule="acme_messages"\n'
    )
    package = prepare(project)
    script = r"""
import sys, socket, subprocess, time
from pathlib import Path
from dimos_cdrgen import registry
registry.initialize((Path(sys.argv[1]),))
from dimos.experimental.memory.cdr_schema import recording_schema
from dimos.experimental.memory.rust_recorder import RustStreamSpec
from dimos.experimental.memory.test_rust_recorder_e2e import _wait_for_log
from dimos.memory.store.cdr_mcap import CdrMcapStore
from dimos.protocol.pubsub.impl.zenohpubsub import ZenohPubSubBase, Topic
from dimos.protocol.service.zenohservice import ZenohSessionPool, ZenohConfig
import json, os
kind=registry.message_types()["acme_msgs/msg/Event"]
original=kind(count=123456, label="external exact 字节")
payload=registry.encode(original)
spec=RustStreamSpec(port="event",name="event",payload_type=kind.__msgtype__,codec="cdr",cdr_schema=recording_schema(kind))
assert spec.source_timestamp is False
with socket.socket() as sock:
 sock.bind(("127.0.0.1",0));port=sock.getsockname()[1]
endpoint=f"tcp/127.0.0.1:{port}"
config={"store":{"kind":"mcap","path":sys.argv[3]},"encoding_threads":1,"streams":[spec.model_dump(exclude_none=True,exclude_defaults=True)]}
launch={"config":config,"topics":{"event":"external/event"},"session":ZenohConfig(mode="peer",connect=[],listen=[endpoint],multicast=False,gossip=False,connect_timeout=0).to_wire()}
process=subprocess.Popen([sys.argv[2]],stdin=subprocess.PIPE,stderr=subprocess.PIPE,stdout=subprocess.DEVNULL,env={**os.environ,"DIMOS_TRANSPORT":"zenoh","RUST_LOG":"info,dimos_memory_recorder=debug"})
pool=ZenohSessionPool();publisher=ZenohPubSubBase(session_pool=pool,mode="client",connect=[endpoint],multicast=False,gossip=False,scouting=False,connect_timeout=5)
try:
 process.stdin.write((json.dumps(launch)+"\n").encode());process.stdin.close()
 _wait_for_log(process,"memory recorder ready");publisher.start();before=time.time();publisher.publish(Topic("external/event"),payload);_wait_for_log(process,"memory recorder batch written")
 process.terminate();assert process.wait(timeout=10)==0
 with CdrMcapStore(path=sys.argv[3]) as store:
  observed=store.stream("event").first();assert before <= observed.ts <= time.time();assert type(observed.data) is kind;assert registry.encode(observed.data)==payload
 from mcap.reader import make_reader
 with open(sys.argv[3],"rb") as source:
  schema,channel,message=next(make_reader(source).iter_messages());assert schema.name==kind.__msgtype__;assert schema.data.decode()==recording_schema(kind)["data"];assert message.data==payload
finally:
 publisher.stop();pool.close_all()
 if process.poll() is None:process.kill();process.wait(timeout=10)
"""
    subprocess.run(
        [
            sys.executable,
            "-c",
            script,
            str(package / "acme_messages_schemas/package"),
            str(executable),
            str(tmp_path / "external.mcap"),
        ],
        check=True,
        timeout=30,
    )
