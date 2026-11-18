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

"""Turn a session's frozen program into an Agent Skill folder (the
agentskills.io format): SKILL.md, the program and a runner under scripts/,
the program API, hypothesis and notes under references/, the validation
record and a few keyframes under assets/."""

from __future__ import annotations

from collections import Counter
import os
from pathlib import Path
import re
import shutil
import tempfile
import textwrap
from typing import Any

from pydantic import BaseModel
import yaml

from dimos.autoresearch.session import Session, TrialRecord
from dimos.manipulation.code_policy.api import API_REFERENCE, API_VERSION

# The agent harness's user skills folder (userSkillsDir in dimos/agents/gateway/config/paths.ts).
USER_SKILLS_DIR = (
    Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    / "dimos"
    / "agent"
    / "skills"
)

NAME_MAX = 64
DESCRIPTION_MAX = 1024
COMPATIBILITY_MAX = 500
BODY_MAX_LINES = 500
KEYFRAMES = 4
FAILURE_LINES = 5
_NAME = re.compile(r"[a-z0-9]+(-[a-z0-9]+)*")
_FRONTMATTER = re.compile(r"---\n(.*?)\n---\n(.*)", re.DOTALL)
_FIELDS = {"name", "description", "license", "compatibility", "metadata", "allowed-tools"}
# The metadata key naming the program API version the program was written against.
API_KEY = "code-policy-api"

# scripts/run.py: runs the program next to it with the task from assets/validation.json.
_RUN_PY = '''\
"""Run this skill's program once on the DimOS stack running on this machine.

Use the DimOS environment's Python. Options are those of
python -m dimos.manipulation.code_policy.runner; --task defaults to the
validated task.
"""

import json
from pathlib import Path
import sys

from dimos.manipulation.code_policy.runner import main

skill = Path(__file__).resolve().parent.parent
task = json.loads((skill / "assets" / "validation.json").read_text())["task"]
sys.exit(main([str(skill / "scripts" / "program.py"), "--task", task, *sys.argv[1:]]))
'''


class SplitTally(BaseModel):
    passed: int
    trials: int


class TrialSummary(BaseModel):
    index: int
    success: bool
    stage: str
    note: str
    duration_s: float


class Validation(BaseModel):
    """``assets/validation.json``: what the program was validated on and how it scored."""

    task: str
    success_rule: str
    candidate: int
    dev: SplitTally
    heldout: SplitTally
    heldout_trials: list[TrialSummary]
    keyframes_from: str
    model: str
    session: str
    dimos_commit: str
    blueprint: str
    scene_source: str


def skill_name(text: str) -> str:
    """A valid skill name from free text: lowercase words joined by hyphens,
    cut to 64 characters; ``autoresearch-program`` when nothing is left."""
    slug = re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")
    return slug[:NAME_MAX].rstrip("-") or "autoresearch-program"


def export_skill(
    session: Session, root: Path, name: str | None = None, *, force: bool = False
) -> Path:
    """Write the frozen candidate of ``session`` as the skill folder
    ``root/<name>`` and return it. ``name`` defaults to a slug of the task.
    Raises ``ValueError`` before a candidate is frozen, when the session has no
    held-out score or a held-out pass rate of 0 (unless ``force``), or when the
    result breaks the format; ``FileExistsError`` when the folder is there."""
    candidate = session.frozen()
    if candidate is None:
        raise ValueError(f"session {session.name} has no frozen candidate; freeze one first")
    info = session.info()
    records = session.results(candidate)
    heldout = [record for record in records if record.split == "heldout"]
    dev = [record for record in records if record.split == "dev"]
    if not force and not info.heldout_rate:
        tally = _tally(heldout)
        score = (
            f"has no held-out score yet ({tally.trials} of "
            f"{info.budget.heldout_trials} held-out trials run)"
            if info.heldout_rate is None
            else f"passed {tally.passed}/{tally.trials} held-out trials (pass rate 0.00)"
        )
        raise ValueError(
            f"candidate {candidate} of session {session.name} {score}; "
            "pass force=True (--force) to export it anyway"
        )
    name = name or skill_name(info.task)
    target = root / name
    if target.exists():
        raise FileExistsError(f"{target} exists; remove it or pass another name")
    shown = _keyframe_trial(records)
    validation = Validation(
        task=info.task,
        success_rule=info.success_rule or "unknown",
        candidate=candidate,
        dev=_tally(dev),
        heldout=_tally(heldout),
        heldout_trials=[
            TrialSummary(
                index=r.index,
                success=r.success,
                stage=r.stage,
                note=r.note,
                duration_s=r.duration_s,
            )
            for r in heldout
        ],
        keyframes_from=f"{shown.split}-{shown.index:02d}" if shown else "",
        model=info.model,
        session=str(session.dir.resolve()),
        dimos_commit=info.dimos_commit or "unknown",
        blueprint=info.blueprint or "unknown",
        scene_source=info.scene_source or "unknown",
    )
    notes = [path.read_text() for path in session.library()]
    root.mkdir(parents=True, exist_ok=True)
    # Built beside the target and moved in once valid, so a failed export leaves nothing behind.
    with tempfile.TemporaryDirectory(dir=root, prefix=".export-") as staging:
        skill = Path(staging) / name
        for folder in ("scripts", "references", "assets/keyframes"):
            (skill / folder).mkdir(parents=True)
        (skill / "SKILL.md").write_text(_skill_md(name, validation, records, bool(notes)))
        shutil.copyfile(session.program_path(candidate), skill / "scripts" / "program.py")
        (skill / "scripts" / "run.py").write_text(_RUN_PY)
        (skill / "references" / "api.md").write_text(_api_md())
        (skill / "references" / "hypothesis.md").write_text(session.hypothesis(candidate))
        if notes:
            (skill / "references" / "findings.md").write_text(_findings(notes))
        (skill / "assets" / "validation.json").write_text(validation.model_dump_json(indent=2))
        if shown:
            frames = session.candidate_dir(candidate) / "trials" / validation.keyframes_from
            for frame in _spread(sorted((frames / "keyframes").glob("*.png")), KEYFRAMES):
                shutil.copyfile(frame, skill / "assets" / "keyframes" / frame.name)
        validate_skill(skill)
        skill.rename(target)
    return target


def validate_skill(directory: Path) -> dict[str, Any]:
    """Check ``directory/SKILL.md`` against the agentskills.io naming and length
    rules, and that its metadata names the program API version; return its
    frontmatter. Raises ``ValueError`` on the first violation."""
    match = _FRONTMATTER.fullmatch((directory / "SKILL.md").read_text())
    if match is None:
        raise ValueError(f"{directory}/SKILL.md does not start with a --- frontmatter block")
    front = yaml.safe_load(match.group(1))
    body = match.group(2)
    if unknown := set(front) - _FIELDS:
        raise ValueError(f"unknown frontmatter fields {sorted(unknown)}")
    name = front.get("name")
    if not isinstance(name, str) or len(name) > NAME_MAX or not _NAME.fullmatch(name):
        raise ValueError(f"name {name!r} is not 1-{NAME_MAX} of a-z, 0-9 and single hyphens")
    if name != directory.name:
        raise ValueError(f"name {name!r} differs from the folder name {directory.name!r}")
    description = front.get("description")
    if not isinstance(description, str) or not 1 <= len(description) <= DESCRIPTION_MAX:
        raise ValueError(f"description must be 1-{DESCRIPTION_MAX} characters")
    compatibility = front.get("compatibility")
    if "compatibility" in front and not (
        isinstance(compatibility, str) and 1 <= len(compatibility) <= COMPATIBILITY_MAX
    ):
        raise ValueError(f"compatibility must be 1-{COMPATIBILITY_MAX} characters")
    metadata = front.get("metadata", {})
    if not isinstance(metadata, dict) or not all(
        isinstance(key, str) and isinstance(value, str) for key, value in metadata.items()
    ):
        raise ValueError("metadata must map string keys to string values")
    if API_KEY not in metadata:
        raise ValueError(f"metadata has no {API_KEY!r}, the program API version")
    if len(body.splitlines()) >= BODY_MAX_LINES:
        raise ValueError(f"the body must be under {BODY_MAX_LINES} lines")
    return dict(front)


def _skill_md(name: str, v: Validation, records: list[TrialRecord], has_findings: bool) -> str:
    task = " ".join(v.task.split())
    robot = v.blueprint
    heldout_rate = _rate(v.heldout)
    rule = v.success_rule
    front = {
        "name": name,
        "description": textwrap.shorten(
            f"Runs a DimOS robot arm program, found and validated in simulation by an "
            f'autoresearch session, for the task "{task}". Use it when asked to do this task '
            f"with a matching DimOS stack running (blueprint: {robot}). Success rule: {rule}. "
            f"Held-out pass rate: {heldout_rate}.",
            DESCRIPTION_MAX,
            placeholder="...",
        ),
        "compatibility": textwrap.shorten(
            f"DimOS with program API {API_VERSION} (dimos.manipulation.code_policy) and a "
            f"running stack that has the manipulation planner, the scene module and the "
            f"obstacle map (validated on blueprint: {robot}). Run with the DimOS environment's "
            f"Python.",
            COMPATIBILITY_MAX,
            placeholder="...",
        ),
        "metadata": {
            "robot": robot,
            "validated-in": "simulation",
            "heldout-pass-rate": heldout_rate,
            "heldout-trials": str(v.heldout.trials),
            "dev-pass-rate": _rate(v.dev),
            "model": v.model,
            "session": v.session,
            "dimos-commit": v.dimos_commit,
            "scene-source": v.scene_source,
            API_KEY: API_VERSION,
        },
    }
    failures = Counter((r.stage, r.note) for r in records if not r.success)
    failure_lines = [
        f"- {count} trial{'s' if count > 1 else ''} stopped at `{stage}`: {note or 'no note'}"
        for (stage, note), count in failures.most_common(FAILURE_LINES)
    ] or ["- none seen in validation"]
    frames = f"camera frames from trial {v.keyframes_from}" if v.keyframes_from else "empty"
    findings = (
        ["- `references/findings.md`: the session's notes on what failed and why."]
        if has_findings
        else []
    )
    body = [
        f"# {task}",
        "",
        f'A program a DimOS autoresearch session wrote and froze for "{task}". '
        f"Success rule: {rule}.",
        "",
        "## Run it",
        "",
        f"A DimOS stack like the one it was validated on (blueprint: {robot}) must already run "
        "on this machine's bus. From this folder, with the DimOS environment's Python:",
        "",
        "```bash",
        'python scripts/run.py [--task "..."] [--out DIR] [--timeout S]',
        "```",
        "",
        "It prints how the program ended (completed, raised or timeout) and the trace path. "
        "`trace.jsonl` has one line per robot call; `keyframes/` holds camera frames before "
        "and after each motion. Completed is not a judged success: check the last keyframes.",
        "",
        "## Validation",
        "",
        f"Simulation only. Blueprint: {robot}. Objects found by: {v.scene_source}. "
        f"Held-out: {v.heldout.passed}/{v.heldout.trials} passed. "
        f"Dev: {v.dev.passed}/{v.dev.trials} passed. "
        f"Candidate {v.candidate}, written by {v.model}, dimos commit {v.dimos_commit}.",
        "",
        "## Known failures",
        "",
        *failure_lines,
        "",
        "## Files",
        "",
        "- `scripts/program.py`: the program, `run(robot, task)`.",
        "- `scripts/run.py`: runs it once on the running stack.",
        f"- `references/api.md`: the program API, version {API_VERSION}.",
        "- `references/hypothesis.md`: what the program was meant to fix and where it may fail.",
        *findings,
        "- `assets/validation.json`: the task, the success rule and every held-out trial.",
        f"- `assets/keyframes/`: {frames}.",
    ]
    dumped = yaml.safe_dump(front, sort_keys=False, allow_unicode=True, width=1_000_000)
    return f"---\n{dumped}---\n" + "\n".join(body) + "\n"


def _api_md() -> str:
    return f"# Program API {API_VERSION}\n\n```text\n{API_REFERENCE}```\n"


def _findings(notes: list[str]) -> str:
    sections = []
    for note in notes:
        title, _, text = note.partition("\n")
        sections.append(f"## {title.removeprefix('# ')}\n\n{text.strip()}\n")
    return "# Findings\n\n" + "\n".join(sections)


def _keyframe_trial(records: list[TrialRecord]) -> TrialRecord | None:
    """A passing held-out trial, else a passing dev trial, else the last trial run."""
    for split in ("heldout", "dev"):
        for record in records:
            if record.split == split and record.success:
                return record
    return records[-1] if records else None


def _spread(frames: list[Path], count: int) -> list[Path]:
    """Up to ``count`` frames spaced evenly from the first to the last."""
    if len(frames) <= count:
        return frames
    return [frames[round(i * (len(frames) - 1) / (count - 1))] for i in range(count)]


def _tally(records: list[TrialRecord]) -> SplitTally:
    return SplitTally(passed=sum(r.success for r in records), trials=len(records))


def _rate(tally: SplitTally) -> str:
    return f"{tally.passed / tally.trials:.2f}" if tally.trials else "unknown"
