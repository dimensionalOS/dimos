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

"""Offline MCP service exposing CodePolicy over one frozen memory cutoff."""

from __future__ import annotations

import math
from pathlib import Path
import time

from dimos.agents.code_policy_core import (
    CodePolicySessionConfig,
    FrozenMemoryEnvironment,
)
from dimos.agents.code_policy_server import CodePolicyMcpServer
from dimos.benchmark.short_horizon_qa.models import CutoffRecord, FrozenMemoryManifest
from dimos.benchmark.short_horizon_qa.prepare import (
    DERIVED_NAME,
    MANIFEST_NAME,
)


def load_bundle(
    bundle: Path,
    cutoff_seconds: float | None = None,
    *,
    progress: float | None = None,
) -> tuple[FrozenMemoryManifest, CutoffRecord, Path, Path]:
    """Validate a prepared bundle and resolve one exact configured cutoff."""
    if (cutoff_seconds is None) == (progress is None):
        raise ValueError("Select exactly one cutoff in seconds or normalized progress")
    bundle = bundle.resolve()
    manifest_path = bundle / MANIFEST_NAME
    if not manifest_path.is_file():
        raise FileNotFoundError(manifest_path)
    manifest = FrozenMemoryManifest.model_validate_json(manifest_path.read_text(encoding="utf-8"))
    source_path = Path(manifest.source_path)
    derived_path = bundle / DERIVED_NAME
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    if not derived_path.is_file():
        raise FileNotFoundError(derived_path)

    if progress is not None:
        matches = [
            cutoff
            for cutoff in manifest.cutoffs
            if cutoff.normalized_progress is not None
            and math.isclose(cutoff.normalized_progress, progress, rel_tol=0.0, abs_tol=1e-12)
        ]
        requested = f"progress {progress}"
        available = ", ".join(
            str(item.normalized_progress)
            for item in manifest.cutoffs
            if item.normalized_progress is not None
        )
    else:
        assert cutoff_seconds is not None
        matches = [
            cutoff
            for cutoff in manifest.cutoffs
            if math.isclose(cutoff.cutoff_seconds, cutoff_seconds, rel_tol=0.0, abs_tol=1e-9)
        ]
        requested = f"cutoff {cutoff_seconds}s"
        available = ", ".join(str(item.cutoff_seconds) for item in manifest.cutoffs)
    if len(matches) != 1:
        raise ValueError(
            f"Requested {requested} is not unique in the bundle. Available: {available}"
        )

    source_stat = source_path.stat()
    if (
        source_stat.st_size != manifest.source_size_bytes
        or source_stat.st_mtime_ns != manifest.source_mtime_ns
    ):
        raise ValueError(f"Source recording identity changed: {source_path}")
    return manifest, matches[0], source_path, derived_path


def frozen_qa_config(
    source_path: Path,
    derived_path: Path,
    cutoff: CutoffRecord,
) -> CodePolicySessionConfig:
    """Build the module-independent session configuration for one cutoff."""
    return CodePolicySessionConfig(
        environment=FrozenMemoryEnvironment(
            recording_path=str(source_path),
            derived_recording_path=str(derived_path),
            memory_cutoff_timestamp=cutoff.cutoff_timestamp,
        )
    )


def serve_bundle(
    bundle: Path,
    cutoff_seconds: float | None = None,
    *,
    progress: float | None = None,
) -> None:
    """Run the frozen QA MCP endpoint until interrupted."""
    _, cutoff, source_path, derived_path = load_bundle(bundle, cutoff_seconds, progress=progress)
    server = CodePolicyMcpServer(frozen_qa_config(source_path, derived_path, cutoff))
    server.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
