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

"""Refresh the pinned, ROS-free schema and parser sources used by message codegen.

Run with the checkout's development Python environment. This maintenance command
requires network access; generation and downstream applications never run it.
"""

from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import tarfile

import requests

ROOT = Path(__file__).resolve().parents[1] / "dimos" / "message_codegen"
SOURCES = (
    (
        "ros2/common_interfaces",
        "a941f14bb318d8d904505ed935ccbb97f24a70a4",
        (
            "std_msgs",
            "geometry_msgs",
            "sensor_msgs",
            "nav_msgs",
            "trajectory_msgs",
            "visualization_msgs",
            "shape_msgs",
        ),
    ),
    ("ros2/rcl_interfaces", "7aa3caf43377ea6ad615bc1040832e2c7566bfbe", ("builtin_interfaces",)),
    ("ros2/geometry2", "f702874b1c8535d6a038230ab2cda0ba5d521ebd", ("tf2_msgs",)),
    ("ros-perception/vision_msgs", "adbf56af9f77e3c8ab2552e25dc99e4a8a77fb27", ("vision_msgs",)),
    ("ros2/rosidl", "85fa592b698b0f665e3120f48fac0d35e2f7d8a4", ("rosidl_adapter",)),
    ("foxglove/schemas", "e86a94e9d4d259cdde71872f18336a7a055bf9ba", ("foxglove_msgs",)),
)


def main() -> None:
    provenance: list[dict[str, object]] = []
    for repository, revision, packages in SOURCES:
        url = f"https://codeload.github.com/{repository}/tar.gz/{revision}"
        response = requests.get(url, timeout=90)
        response.raise_for_status()
        files: dict[str, str] = {}
        with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:gz") as archive:
            for member in archive:
                path = Path(*Path(member.name).parts[1:])
                if (
                    repository == "foxglove/schemas"
                    and path.as_posix() == "schemas/ros2/CompressedVideo.msg"
                ):
                    path = Path("foxglove_msgs/msg/CompressedVideo.msg")
                if not member.isfile() or not path.parts:
                    continue
                if len(path.parts) == 1 and path.name in {"LICENSE", "LICENSE.txt", "NOTICE"}:
                    target = ROOT / "schemas" / "licenses" / repository.replace("/", "_") / path
                elif path.parts[0] not in packages:
                    continue
                elif path.as_posix() == "rosidl_adapter/rosidl_adapter/parser.py":
                    target = ROOT / "_vendor" / "rosidl_parser.py"
                elif path.name in {"LICENSE", "LICENSE.txt", "NOTICE", "package.xml"} or (
                    len(path.parts) == 3 and path.parts[1] == "msg" and path.suffix == ".msg"
                ):
                    target = ROOT / "schemas" / path
                else:
                    continue
                source = archive.extractfile(member)
                assert source is not None
                content = source.read()
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(content)
                files[str(target.relative_to(ROOT))] = hashlib.sha256(content).hexdigest()
        provenance.append(
            {"repository": repository, "revision": revision, "files": dict(sorted(files.items()))}
        )
        print(f"{repository}@{revision}: {len(files)} files")
    (ROOT / "sources.json").write_text(json.dumps(provenance, indent=2) + "\n")


if __name__ == "__main__":
    main()
