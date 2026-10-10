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

"""Host-only acceptance checks; never copy this grader-side script into the robot image."""

import argparse
import json
from pathlib import Path


def verify(root: Path) -> None:
    for mode in ("normal", "blocked", "forgery"):
        directory = root / mode
        result = json.loads((directory / "result.json").read_text())
        manifest = json.loads((directory / "episode.json").read_text())
        assert result["run"] == manifest["run"]
        assert result["episode"] == manifest["episode"]
        assert result["case"] == "robosuite_lift_cube"
        assert result["error"] == "", result
        assert result["score"] == 0.0, result
        assert result["attempts"] >= 2
        events = [
            json.loads(line) for line in (directory / "boundary.jsonl").read_text().splitlines()
        ]
        assert any(event["accepted"] for event in events)
        if mode != "normal":
            assert any(not event["accepted"] for event in events)
            assert any(event["reason"] == "replay" for event in events)
        print(f"{mode}: trusted hold-baseline score=0; attempts={result['attempts']}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path)
    verify(parser.parse_args().results)
