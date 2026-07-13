# Copyright 2025-2026 Dimensional Inc.
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

"""Fixtures for the skill-verification suite.

Each test module declares a module-level ``SKILL_ID`` matching its manifest
entry; the ``skill_output`` fixture then hands it a cleaned per-skill directory
under test_outputs/ and records which artifacts it produced. After the session,
``pytest_sessionfinish`` renders test_outputs/REVIEW.md.
"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
from typing import TYPE_CHECKING

import matplotlib
import pytest

from dimos.skills.skill_verification._review import (
    OUTPUTS_ROOT,
    ReviewCollector,
    SkillOutput,
    skills_by_id,
)

# Headless rendering for every test in this suite.
matplotlib.use("Agg")

if TYPE_CHECKING:
    from collections.abc import Iterator

_COLLECTOR_KEY = "_skill_verify_collector"
_CLEANED_KEY = "_skill_verify_cleaned"


def _collector(config: pytest.Config) -> ReviewCollector:
    collector = getattr(config, _COLLECTOR_KEY, None)
    if collector is None:
        collector = ReviewCollector()
        setattr(config, _COLLECTOR_KEY, collector)
        setattr(config, _CLEANED_KEY, set())
    return collector


@pytest.fixture
def skill_output(request: pytest.FixtureRequest) -> Iterator[SkillOutput]:
    """A cleaned per-skill output directory under test_outputs/.

    Reads ``SKILL_ID`` from the requesting test module. The directory is wiped
    once per session on first use so a rerun doesn't mix old and new artifacts.
    """
    skill_id = getattr(request.module, "SKILL_ID", None)
    if skill_id is None:
        raise RuntimeError(f"{request.module.__name__} must define module-level SKILL_ID")
    known = skills_by_id()
    if skill_id not in known:
        raise RuntimeError(f"SKILL_ID {skill_id!r} is not in manifest.yaml")

    _collector(request.config)  # ensures the collector + cleaned set exist
    out_dir = OUTPUTS_ROOT / skill_id
    cleaned: set[str] = getattr(request.config, _CLEANED_KEY)
    if skill_id not in cleaned:
        if out_dir.exists():
            shutil.rmtree(out_dir)
        cleaned.add(skill_id)
    out_dir.mkdir(parents=True, exist_ok=True)

    out = SkillOutput(skill_id=skill_id, dir=out_dir)
    yield out
    _collector(request.config).merge(out)


@pytest.fixture(scope="session")
def recording_path() -> Path | None:
    """A real Rerun recording to drive real-data tiers, or None.

    Resolution order: $SKILL_VERIFY_RRD, then chinaOffice.rrd at the repo root.
    """
    env = os.environ.get("SKILL_VERIFY_RRD")
    if env and Path(env).is_file():
        return Path(env)
    from dimos.constants import DIMOS_PROJECT_ROOT

    default = DIMOS_PROJECT_ROOT / "chinaOffice.rrd"
    return default if default.is_file() else None


def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    collector = getattr(session.config, _COLLECTOR_KEY, None)
    if collector is None:
        return
    # Only the xdist controller (or a non-xdist run) writes the summary.
    if hasattr(session.config, "workerinput"):
        return
    path = collector.write()
    reporter = session.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(f"\n[skill-verification] review index: {path}")
