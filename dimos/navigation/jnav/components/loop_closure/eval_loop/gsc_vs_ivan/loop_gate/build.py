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

"""Assemble the Loop Closure Gate Lab page from the example json files; each example's ICP clouds go to clouds/*.txt.
usage: build.py <out dir> <loop_examples_NAME.json>..."""

import base64
import json
from pathlib import Path
import struct
import sys

HERE = Path(__file__).parent
OUT = Path(sys.argv[1])
(OUT / "clouds").mkdir(parents=True, exist_ok=True)
for old in (OUT / "clouds").glob("*.*"):
    old.unlink()
examples = []
for path in map(Path, sys.argv[2:]):
    name = path.stem.removeprefix("loop_examples_")
    for e in json.loads(path.read_text())["examples"]:
        source, target = (
            base64.b64decode(e.pop("source_b64")),
            base64.b64decode(e.pop("target_b64")),
        )
        # base64 of: u32 source count, u32 target count, then float32 xyz (source points, target points); artifacts serve text, not binary
        e["clouds"] = f"clouds/{name}_{e['source_id']}_{e['target_id']}.txt"
        (OUT / e["clouds"]).write_bytes(
            base64.b64encode(
                struct.pack("<II", len(source) // 12, len(target) // 12) + source + target
            )
        )
        if e.get("extended"):
            x = e["extended"]
            source, target = (
                base64.b64decode(x.pop("source_b64")),
                base64.b64decode(x.pop("target_b64")),
            )
            x["clouds"] = f"clouds/{name}_{e['source_id']}_{e['target_id']}_ext.txt"
            (OUT / x["clouds"]).write_bytes(
                base64.b64encode(
                    struct.pack("<II", len(source) // 12, len(target) // 12) + source + target
                )
            )
        examples.append(e)
names = sorted({path.stem.removeprefix("loop_examples_") for path in map(Path, sys.argv[2:])})
head = (
    (HERE / "page_head.html")
    .read_text()
    .replace(
        "<title>Loop Closure Gate Lab</title>",
        f"<title>{' + '.join(names)} Loop Inspection</title>",
    )
)
body = (
    (HERE / "page_body.html")
    .read_text()
    .replace("__DATA__", json.dumps({"examples": examples}).replace("</", "<\\/"))
)
(OUT / "loop_gate_lab.html").write_text(head + body)
clouds = sum(p.stat().st_size for p in (OUT / "clouds").glob("*.txt"))
print(
    len(examples),
    "examples",
    round(len(head + body) / 1e6, 2),
    "MB page,",
    round(clouds / 1e6, 1),
    "MB clouds",
)
