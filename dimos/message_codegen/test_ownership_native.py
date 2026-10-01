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
import subprocess
import sys

from mcap.reader import make_reader
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from dimos.message_codegen.generate import generate
from dimos.message_codegen.ownership import Dependency
from dimos.protocol.cdr_mcap import CdrMcapWriter


def test_separate_packages_share_types_and_exchange_cdr(tmp_path):
    prefix = os.environ.get("DIMOS_FASTCDR_PREFIX")
    if not prefix:
        pytest.skip("Set DIMOS_FASTCDR_PREFIX to run the compiled ownership contract")
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
    install = tmp_path / "install"
    cmake_prefix = f"{Path(prefix).resolve()};{install}"
    for package in (base, custom):
        subprocess.run(
            [
                "cmake",
                "-S",
                str(package / "cpp"),
                "-B",
                str(package / "build"),
                "-DCMAKE_BUILD_TYPE=Release",
                f"-DCMAKE_PREFIX_PATH={cmake_prefix}",
                f"-DCMAKE_INSTALL_PREFIX={install}",
                f"-DPython_EXECUTABLE={sys.executable}",
            ],
            check=True,
        )
        subprocess.run(["cmake", "--build", str(package / "build"), "-j", "2"], check=True)
        subprocess.run(["cmake", "--install", str(package / "build")], check=True)
    env = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join([str(base / "build"), str(custom / "build")]),
    }
    python = """
from ownership_base.std_msgs.msg import Header
from ownership_custom.custom_msgs.msg import Reading
from pathlib import Path
h = Header(frame_id='map')
h.stamp.sec = 17
h.stamp.nanosec = 123456789
msg = Reading(header=h, value=20.5)
assert type(msg.header) is Header
msg.header = h
assert type(Reading.decode(msg.encode()).header) is Header
Path('input.cdr').write_bytes(msg.encode())
"""
    subprocess.run([sys.executable, "-c", python], env=env, cwd=tmp_path, check=True)
    consumer = tmp_path / "consumer"
    consumer.mkdir()
    (consumer / "CMakeLists.txt").write_text("""cmake_minimum_required(VERSION 3.20)
project(ownership_consumer LANGUAGES CXX)
find_package(ownership_custom CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE ownership_custom::messages)
""")
    (consumer / "main.cpp").write_text("""#include <ownership_base/messages.hpp>
#include <ownership_custom/messages.hpp>
#include <fstream>
#include <iterator>
#include <type_traits>
static_assert(std::is_same_v<decltype(custom_msgs::msg::Reading{}.header), std_msgs::msg::Header>);
int main() {
 std::ifstream in("input.cdr", std::ios::binary);
 std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
 auto msg = dimos::cdr::decode<custom_msgs::msg::Reading>(bytes);
 std_msgs::msg::Header header = msg.header; msg.header = header;
 msg.value += 1;
 auto out = dimos::cdr::encode(msg);
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
            f"-DCMAKE_PREFIX_PATH={cmake_prefix}",
        ],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(consumer / "build"), "-j", "2"], check=True)
    subprocess.run([str(consumer / "build/consumer")], cwd=tmp_path, check=True)
    (custom / "rust/src/bin").mkdir()
    (
        custom / "rust/src/bin/check.rs"
    ).write_text("""use ownership_custom_messages::{codec::Message, custom_msgs::msg::Reading};
use ownership_base_messages::std_msgs::msg::Header;
fn main() {
 let mut msg = Reading::decode(&std::fs::read("cpp.cdr").unwrap()).unwrap();
 let header: Header = msg.header; msg.header = header;
 msg.value += 1.0;
 std::fs::write("rust.cdr", ownership_base_messages::codec::Message::encode(&msg).unwrap()).unwrap();
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
            """
from ownership_base.std_msgs.msg import Header
from ownership_custom.custom_msgs.msg import Reading
from pathlib import Path
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
msg = Reading.decode(Path('rust.cdr').read_bytes())
assert type(msg.header) is Header
assert (msg.value, msg.header.frame_id, msg.header.stamp.nanosec) == (22.5, 'map', 123456789)
reference = get_typestore(Stores.ROS2_JAZZY)
reference.register(get_types_from_msg(Reading.schema, Reading.msg_name))
assert reference.deserialize_cdr(msg.encode(), Reading.msg_name).value == 22.5
""",
        ],
        cwd=tmp_path,
        env=env,
        check=True,
    )
    for mutation in (
        "base.__dimos_version__ = '99.0.0'",
        "base.std_msgs.msg.Header.schema = 'wrong'",
    ):
        result = subprocess.run(
            [
                sys.executable,
                "-c",
                f"import ownership_base as base; {mutation}; import ownership_custom",
            ],
            env=env,
            capture_output=True,
            text=True,
        )
        assert result.returncode != 0
        assert "ImportError" in result.stderr

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
