# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from mcap.reader import make_reader
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from dimos.message_codegen.build import local_cargo_dependency
from dimos.message_codegen.generate import generate
from dimos.message_codegen.native_build import prepare_cpp, write_cmake_toolchain
from dimos.message_codegen.ownership import Dependency
from dimos.protocol.cdr_mcap import CdrMcapWriter


@pytest.fixture(autouse=True)
def restore_reference_types_module():
    # Independent oracle stores own a process-global module. Restore it so they
    # do not invalidate the canonical registry's native class pickle identities.
    previous = sys.modules.get("rosbags.usertypes")
    try:
        yield
    finally:
        if previous is None:
            sys.modules.pop("rosbags.usertypes", None)
        else:
            sys.modules["rosbags.usertypes"] = previous


def test_separate_packages_share_types_and_exchange_cdr(tmp_path, monkeypatch):
    if os.environ.get("DIMOS_NATIVE_ACCEPTANCE") != "1":
        pytest.skip("Set DIMOS_NATIVE_ACCEPTANCE=1 to run the compiled ownership contract")
    root = Path(__file__).resolve().parents[2]
    source = tmp_path / "interfaces/custom_msgs/msg/Reading.msg"
    source.parent.mkdir(parents=True)
    source.write_text("std_msgs/Header header\nfloat64 value\n")
    base, custom = tmp_path / "base", tmp_path / "custom"
    generate([], base, ["std_msgs/msg/Header"], "ownership_base", shared=True)
    generate(
        [source.parents[2]],
        custom,
        ["custom_msgs/msg/Reading"],
        "ownership_custom",
        dependencies=(Dependency.load(base),),
        shared=True,
    )
    monkeypatch.syspath_prepend(str(base / "python"))
    sources = {
        name: Path(path)
        for name, path in json.loads(os.environ.get("DIMOS_NATIVE_SOURCE_DIRS", "{}")).items()
    }
    cache = os.environ.get("DIMOS_NATIVE_TEST_CACHE")
    prefix = prepare_cpp(
        custom,
        cache=Path(cache) if cache else None,
        offline=os.environ.get("DIMOS_OFFLINE") == "1",
        source_dirs=sources or None,
    )
    toolchain = write_cmake_toolchain(prefix, tmp_path / "toolchain.cmake")
    (tmp_path / "dimos_message_build").symlink_to(Path(__file__).parent, target_is_directory=True)
    env = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join(
            [str(tmp_path), str(base / "python"), str(custom / "python")]
        ),
    }
    bootstrap = """
from pathlib import Path
from dimos_message_build import registry
registry.providers = lambda: ()
registry.initialize((Path("base"), Path("custom")), discover=False)
"""
    python = """
from ownership_base.std_msgs.msg import Header
from ownership_base.builtin_interfaces.msg import Time
from ownership_custom.custom_msgs.msg import Reading
from pathlib import Path
h = Header(stamp=Time(sec=0, nanosec=0), frame_id='map')
h.stamp.sec = 17
h.stamp.nanosec = 123456789
msg = Reading(header=h, value=20.5)
assert type(msg.header) is Header
msg.header = h
assert type(registry.decode(registry.encode(msg), Reading).header) is Header
Path('input.cdr').write_bytes(registry.encode(msg))
"""
    subprocess.run([sys.executable, "-c", bootstrap + python], env=env, cwd=tmp_path, check=True)
    consumer = tmp_path / "consumer"
    consumer.mkdir()
    (consumer / "CMakeLists.txt").write_text(f"""cmake_minimum_required(VERSION 3.20)
project(ownership_consumer LANGUAGES CXX)
find_package(ownership_custom CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_17)
target_include_directories(consumer PRIVATE "{root}/native/cpp/include")
target_link_libraries(consumer PRIVATE ownership_custom::messages)
""")
    (consumer / "main.cpp").write_text("""#include <std_msgs/msg/header.hpp>
#include <custom_msgs/msg/reading.hpp>
#include <dimos/native/cdr_codec.hpp>
#include <fstream>
#include <iterator>
#include <type_traits>
static_assert(std::is_same_v<decltype(custom_msgs::msg::Reading{}.header), std_msgs::msg::Header>);
int main() {
 std::ifstream in("input.cdr", std::ios::binary);
 std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
 auto msg = dimos::native::cdr_decode<custom_msgs::msg::Reading>(bytes);
 std_msgs::msg::Header header = msg.header; msg.header = header;
 msg.value += 1;
 auto out = dimos::native::cdr_encode(msg);
 std::ofstream file("cpp.cdr", std::ios::binary);
 file.write(reinterpret_cast<const char*>(out.data()), out.size());
 return file ? 0 : 1;
}
""")
    subprocess.run(
        [
            "cmake",
            "-S",
            str(consumer),
            "-B",
            str(consumer / "build"),
            f"-DCMAKE_TOOLCHAIN_FILE={toolchain}",
        ],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(consumer / "build"), "-j", "2"], check=True)
    subprocess.run([str(consumer / "build/consumer")], cwd=tmp_path, check=True)
    manifest = custom / "rust/Cargo.toml"
    manifest.write_text(
        local_cargo_dependency(manifest.read_text(), "ownership_base").replace(
            "../ownership_base", str(base / "rust")
        )
    )
    (custom / "rust/src/bin").mkdir()
    (
        custom / "rust/src/bin/check.rs"
    ).write_text("""use ownership_custom_messages::custom_msgs::msg::reading::Reading;
use ownership_base_messages::std_msgs::msg::header::Header;
fn main() {
 let wire = std::fs::read("cpp.cdr").unwrap();
 assert_eq!(&wire[..4], &[0, 1, 0, 0]);
 let (mut msg, consumed) = re_cdr::from_bytes::<Reading, re_cdr::LittleEndian>(&wire[4..]).unwrap();
 assert_eq!(consumed, wire.len() - 4);
 let header: Header = msg.header; msg.header = header;
 msg.value += 1.0;
 let mut wire = vec![0, 1, 0, 0];
 wire.extend(re_cdr::to_vec::<_, re_cdr::LittleEndian>(&msg).unwrap());
 std::fs::write("rust.cdr", wire).unwrap();
}
""")
    subprocess.run(
        [
            "cargo",
            "run",
            "--offline",
            "--manifest-path",
            str(custom / "rust/Cargo.toml"),
            "--bin",
            "check",
        ],
        cwd=tmp_path,
        check=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-c",
            bootstrap
            + """
from ownership_base.std_msgs.msg import Header
from ownership_base.builtin_interfaces.msg import Time
from ownership_custom.custom_msgs.msg import Reading
from pathlib import Path
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
msg = registry.decode(Path('rust.cdr').read_bytes(), Reading)
assert type(msg.header) is Header
assert (msg.value, msg.header.frame_id, msg.header.stamp.nanosec) == (22.5, 'map', 123456789)
reference = get_typestore(Stores.ROS2_JAZZY)
reference.register(get_types_from_msg(registry.schema(Reading.__msgtype__), Reading.__msgtype__))
assert reference.deserialize_cdr(registry.encode(msg), Reading.__msgtype__).value == 22.5
""",
        ],
        cwd=tmp_path,
        env=env,
        check=True,
    )
    # Public source manifests are validated before classes are exposed.
    manifest_path = base / "message-package.json"
    installed_manifest = base / "python/ownership_base_schemas/package/message-package.json"
    original = manifest_path.read_text()
    for key, value, error in (
        ("version", "99.0.0", "Dependency version mismatch"),
        ("schemas", {}, "Message schema digest mismatch"),
    ):
        changed = json.loads(original)
        changed[key] = value
        manifest_path.write_text(json.dumps(changed))
        installed_manifest.write_text(json.dumps(changed))
        try:
            result = subprocess.run(
                [sys.executable, "-c", bootstrap],
                env=env,
                cwd=tmp_path,
                capture_output=True,
                text=True,
            )
            assert result.returncode != 0
            assert error in result.stderr
        finally:
            manifest_path.write_text(original)
            installed_manifest.write_text(original)

    # Playback has only the recording: imported dependency schemas must be embedded.
    name = "custom_msgs/msg/Reading"
    schema = json.loads((custom / "schemas.json").read_text())[name]
    recording = tmp_path / "custom.mcap"
    with CdrMcapWriter(recording) as writer:
        writer.write(
            "/reading",
            (tmp_path / "rust.cdr").read_bytes(),
            schema_name=name,
            schema=schema,
            log_time_ns=1,
        )
    with recording.open("rb") as stream:
        [(stored, channel, message)] = list(make_reader(stream).iter_messages())
    assert channel.message_encoding == "cdr"
    assert stored.encoding == "ros2msg"
    reference = get_typestore(Stores.EMPTY)
    reference.register(get_types_from_msg(stored.data.decode(), stored.name))
    decoded = reference.deserialize_cdr(message.data, stored.name)
    assert (decoded.value, decoded.header.frame_id, decoded.header.stamp.nanosec) == (
        22.5,
        "map",
        123456789,
    )

    destination = os.environ.get("DIMOS_OWNERSHIP_EVIDENCE")
    if destination:
        evidence = Path(destination)
        evidence.mkdir(parents=True, exist_ok=True)
        for name in ("input.cdr", "cpp.cdr", "rust.cdr", "custom.mcap"):
            shutil.copy2(tmp_path / name, evidence / name)
        shutil.copytree(custom / "cpp/custom_msgs", evidence / "custom_msgs", dirs_exist_ok=True)
