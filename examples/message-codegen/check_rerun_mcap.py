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

"""Check native Rerun's semantic and reflection imports, without message packages."""

from collections import Counter
from pathlib import Path
import re
import sys


def main() -> None:
    evidence = Path(sys.argv[1])
    summary = (evidence / "rerun-summary.txt").read_text()
    expected = {
        "/camera/image": "Image:buffer",
        "/camera/compressed": "EncodedImage:blob",
        "/robot/pose": "InstancePoses3D:translations",
        "/tf": "Transform3D:translation",
        "/telemetry": "demo_msgs.msg.Telemetry:message",
    }
    counts: Counter[str] = Counter()
    for line in summary.splitlines():
        match = re.search(r"with (\d+) rows .* - (/[\w/]+) - data columns: (.*)", line)
        if match:
            rows, topic, columns = match.groups()
            assert topic in expected, topic
            assert expected[topic] in columns, (topic, columns)
            counts[topic] += int(rows)
    assert counts == dict.fromkeys(expected, 30), counts
    telemetry = (evidence / "rerun-telemetry.txt").read_text()
    # This must be an Arrow struct with nested fields, not an opaque byte blob.
    for field in ('"temperature": Float64', '"sequence": UInt32', '"hops": List(Int32)'):
        assert field in telemetry, field
    for topic, count in counts.items():
        print(f"Rerun imported {count} {topic} rows as {expected[topic]}.")
    print("Custom telemetry is a structured Arrow value with nested fields.")


if __name__ == "__main__":
    main()
