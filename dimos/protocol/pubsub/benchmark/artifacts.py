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

"""Versioned transport benchmark artifact storage."""

from __future__ import annotations

from collections.abc import Iterable
import csv
from datetime import datetime, timezone
import gzip
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
from typing import Any, cast

from dimos.protocol.pubsub.benchmark.metrics import (
    aggregate_summaries,
    compare_to_ros,
    summarize_trial,
)
from dimos.protocol.pubsub.benchmark.model import SCHEMA_VERSION, MessageSample, TrialRecord


def _command_output(argv: list[str]) -> str | None:
    try:
        completed = subprocess.run(argv, check=True, capture_output=True, text=True, timeout=10)
    except (FileNotFoundError, subprocess.SubprocessError):
        return None
    return completed.stdout.strip()


def _read_text(path: str) -> str | None:
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def collect_manifest(
    *, suite: str, seed: int, command: list[str], benchmark_image: str
) -> dict[str, Any]:
    packages: dict[str, str | None] = {}
    for package in ("dimos", "eclipse-zenoh", "plotly", "psutil"):
        try:
            packages[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            packages[package] = None
    return {
        "schema_version": SCHEMA_VERSION,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "suite": suite,
        "seed": seed,
        "command": command,
        "git_revision": _command_output(["git", "rev-parse", "HEAD"]),
        "platform": platform.platform(),
        "kernel": platform.release(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "cpu_count": os.cpu_count(),
        "cpu_governor": _read_text("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
        "net_core_rmem_default": _read_text("/proc/sys/net/core/rmem_default"),
        "net_core_rmem_max": _read_text("/proc/sys/net/core/rmem_max"),
        "container_runtime": _command_output(
            ["docker", "version", "--format", "{{.Server.Version}}"]
        ),
        "benchmark_image": benchmark_image,
        "benchmark_image_id": _command_output(
            ["docker", "image", "inspect", "--format", "{{.Id}}", benchmark_image]
        ),
        "packages": packages,
    }


def collect_container_manifest(image: str) -> dict[str, Any]:
    """Probe versions inside the exact image used for emulated trials."""
    script = """
import importlib.metadata
import json
import os
import subprocess

from rclpy.utilities import get_rmw_implementation_identifier

def version(package):
    try:
        return importlib.metadata.version(package)
    except importlib.metadata.PackageNotFoundError:
        return None

def debian_version(package):
    result = subprocess.run(
        ["dpkg-query", "--show", "--showformat=${Version}", package],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout or None

print(json.dumps({
    "ros_distro": os.environ.get("ROS_DISTRO"),
    "rmw_identifier": get_rmw_implementation_identifier(),
    "rmw_zenoh_cpp": debian_version("ros-jazzy-rmw-zenoh-cpp"),
    "zenoh_cpp_vendor": debian_version("ros-jazzy-zenoh-cpp-vendor"),
    "native_zenoh": version("eclipse-zenoh"),
    "dimos": version("dimos"),
    "python": os.sys.version,
}, sort_keys=True))
"""
    completed = subprocess.run(
        [
            "docker",
            "run",
            "--rm",
            "--env",
            "RMW_IMPLEMENTATION=rmw_zenoh_cpp",
            image,
            "/app/.venv/bin/python",
            "-c",
            script,
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=60.0,
    )
    return cast("dict[str, Any]", json.loads(completed.stdout))


class ArtifactWriter:
    """Owns one campaign directory and writes append-only trial artifacts."""

    def __init__(self, output_dir: Path, manifest: dict[str, Any]) -> None:
        self.output_dir = output_dir
        self.samples_dir = output_dir / "samples"
        self.logs_dir = output_dir / "logs"
        self.topology_dir = output_dir / "topology"
        for path in (self.samples_dir, self.logs_dir, self.topology_dir):
            path.mkdir(parents=True, exist_ok=True)
        manifest_path = output_dir / "manifest.json"
        if manifest_path.exists():
            existing = json.loads(manifest_path.read_text())
            for key in ("schema_version", "suite", "seed", "trials"):
                if existing.get(key) != manifest.get(key):
                    raise ValueError(f"Cannot resume: manifest {key} does not match")
        else:
            self._write_json(manifest_path, manifest)

    @staticmethod
    def _write_json(path: Path, value: Any) -> None:
        path.write_text(json.dumps(value, indent=2, sort_keys=True, default=str) + "\n")

    @staticmethod
    def _append_jsonl(path: Path, values: Iterable[dict[str, Any]]) -> None:
        with gzip.open(path, "at", encoding="utf-8") as stream:
            for value in values:
                stream.write(json.dumps(value, sort_keys=True, default=str) + "\n")

    def write_trial(self, record: TrialRecord) -> dict[str, Any]:
        summary = summarize_trial(record)
        self._append_jsonl(
            self.samples_dir / f"{record.spec.trial_id}.jsonl.gz",
            (sample.to_dict() for sample in record.samples),
        )
        self._append_jsonl(self.output_dir / "trials.jsonl.gz", (summary,))
        return summary

    def prepare_retry(self, trial_id: str) -> None:
        """Remove artifacts for one failed/incomplete trial before rerunning it."""
        sample_path = self.samples_dir / f"{trial_id}.jsonl.gz"
        if sample_path.exists():
            sample_path.unlink()
        topology_path = self.topology_dir / trial_id
        if topology_path.exists():
            shutil.rmtree(topology_path)
        for log_path in self.logs_dir.glob(f"{trial_id}-*.log"):
            log_path.unlink()

    def finalize(self, summaries: list[dict[str, Any]], *, seed: int) -> None:
        with gzip.open(self.output_dir / "trials.jsonl.gz", "wt", encoding="utf-8") as stream:
            for summary in summaries:
                stream.write(json.dumps(summary, sort_keys=True, default=str) + "\n")
        self._write_json(self.output_dir / "summary.json", summaries)
        self._write_json(
            self.output_dir / "aggregates.json",
            aggregate_summaries(summaries, seed=seed),
        )
        self._write_json(
            self.output_dir / "comparisons.json",
            compare_to_ros(summaries, seed=seed),
        )
        columns = sorted({key for summary in summaries for key in summary if key != "usage"})
        with (self.output_dir / "summary.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(summaries)


def load_summaries(run_dir: Path) -> list[dict[str, Any]]:
    path = run_dir / "summary.json"
    if not path.exists():
        raise FileNotFoundError(f"Benchmark summary does not exist: {path}")
    data = json.loads(path.read_text())
    if not isinstance(data, list):
        raise ValueError(f"Benchmark summary must contain a JSON list: {path}")
    return data


def load_trial_summaries(run_dir: Path) -> list[dict[str, Any]]:
    """Load durable per-trial checkpoints, including an interrupted campaign."""
    path = run_dir / "trials.jsonl.gz"
    if not path.exists():
        return []
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream]


def load_samples(path: Path) -> list[MessageSample]:
    samples = []
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        for line in stream:
            samples.append(MessageSample(**json.loads(line)))
    return samples
