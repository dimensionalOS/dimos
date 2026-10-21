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

"""Exchange an installed built-in Header through Python, C++ and Rust."""

import json
import os
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory

from dimos_cdrgen.native_build import prepare_cpp, write_cmake_toolchain
from dimos_cdrgen.registry import decode, encode

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header
from dimos.messages_schemas.provider import package_root  # type: ignore[import-untyped]


def main() -> None:
    for tool in ("cmake", "cargo"):
        if shutil.which(tool) is None:
            raise SystemExit(f"Install {tool} before running this native example")
    sources = package_root()
    print(f"Installed native message sources: {sources}", flush=True)
    offline = os.environ.get("DIMOS_OFFLINE") == "1"
    cache = Path(
        os.environ.get("DIMOS_MESSAGE_NATIVE_CACHE", Path.home() / ".cache/dimos/messages")
    )
    prefix = prepare_cpp(sources, cache=cache, offline=offline)
    with TemporaryDirectory(prefix="dimos-builtin-example-") as temporary:
        project = Path(temporary)
        shutil.copytree(Path(__file__).with_name("native"), project, dirs_exist_ok=True)
        toolchain = write_cmake_toolchain(prefix, project / "toolchain.cmake")
        (project / "python.cdr").write_bytes(encode(Header(Time(17, 23), "map")))
        commands = [
            ["cmake", "-S", ".", "-B", "build", f"-DCMAKE_TOOLCHAIN_FILE={toolchain}"],
            ["cmake", "--build", "build", "--parallel", "2"],
            ["build/header_relay", "python.cdr", "cpp.cdr"],
            [
                "cargo",
                "--config",
                "patch.crates-io.dimos-generated-messages.path="
                + json.dumps(str(sources / "rust")),
                "run",
                *(["--offline"] if offline else []),
                "--example",
                "relay",
                "--",
                "cpp.cdr",
                "rust.cdr",
            ],
        ]
        for command in commands:
            subprocess.run(command, cwd=project, check=True)
        value = decode((project / "rust.cdr").read_bytes(), Header)
        assert value == Header(Time(17, 23), "map/cpp/rust"), value
        print("PASS: built-in Header Python -> C++ -> Rust -> Python")


if __name__ == "__main__":
    main()
