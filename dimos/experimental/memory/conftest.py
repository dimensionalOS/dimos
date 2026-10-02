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

from __future__ import annotations

from collections.abc import Callable
import json
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

from mcap.reader import make_reader
import pytest

from dimos.constants import DIMOS_PROJECT_ROOT


@pytest.fixture(scope="module")
def native_mcap_writer() -> Callable[[Path, Path, list[dict[str, str]]], None]:
    """Run the test-only offline adapter through the production Rust encoder/writer."""

    def write(directory: Path, artifact: Path, streams: list[dict[str, str]]) -> None:
        (directory / "config.json").write_text(
            json.dumps(
                {
                    "store": {"kind": "mcap", "path": str(artifact)},
                    "encoding_threads": 2,
                    "streams": streams,
                }
            )
        )
        result = subprocess.run(
            [
                "cargo",
                "test",
                "-p",
                "dimos-memory-recorder",
                "--lib",
                "write_interop_fixture",
                "--",
                "--ignored",
            ],
            cwd=DIMOS_PROJECT_ROOT,
            env={**os.environ, "DIMOS_MCAP_INTEROP_DIR": str(directory)},
            capture_output=True,
            text=True,
            timeout=600,
        )
        assert result.returncode == 0, result.stdout + result.stderr

    return write


@pytest.fixture(scope="module")
def foxglove_validator() -> Callable[[Path], None]:
    """Exercise the same schema parser and CDR reader as the Foxglove viewer."""

    def validate(artifact: Path) -> None:
        with TemporaryDirectory(dir=artifact.parent) as temporary, artifact.open("rb") as source:
            directory = Path(temporary)
            reader = make_reader(source)
            summary = reader.get_summary()
            assert summary is not None
            remaining = {c.topic for c in summary.channels.values() if c.message_encoding == "cdr"}
            expected = sorted(remaining)
            samples = []
            for schema, channel, message in reader.iter_messages():
                if channel.topic not in remaining:
                    continue
                assert schema is not None
                schema_path = directory / f"{channel.id}.msg"
                payload_path = directory / f"{channel.id}.cdr"
                schema_path.write_bytes(schema.data)
                payload_path.write_bytes(message.data)
                samples.append(
                    {
                        "topic": channel.topic,
                        "schema": str(schema_path),
                        "payload": str(payload_path),
                    }
                )
                remaining.remove(channel.topic)
                if not remaining:
                    break
            assert expected and not remaining
            manifest = directory / "samples.json"
            manifest.write_text(json.dumps(samples))
            result = subprocess.run(
                [
                    "deno",
                    "run",
                    "--no-config",
                    "--no-lock",
                    f"--allow-read={directory}",
                    str(Path(__file__).with_name("foxglove_decode.ts")),
                    str(manifest),
                ],
                capture_output=True,
                text=True,
                timeout=120,
            )
            assert result.returncode == 0, result.stdout + result.stderr
            assert sorted(json.loads(result.stdout)) == expected

    return validate
