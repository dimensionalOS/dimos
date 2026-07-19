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

"""Static-typing regression harness for the pure-module surface (T4).

Mechanism (spec: dimos/pure/tasks/t4-typing.md §harness): one batched mypy
subprocess over every fixture in test_typing_fixtures/, repo config, cold
cache; each fixture's diagnostics must match its inline markers exactly, in
both directions.

Marker DSL — same-line comments in fixtures, one marker per line:

    # E[code]: substring   expect an error with that code whose message
                           contains the substring
    # R: type              expect note `Revealed type is "type"`, compared
                           EXACTLY after stripping the fixture's own module
                           prefix from the revealed string

Errors and reveal-notes are matched; other notes (Liskov explanations,
protocol conflict dumps) are ignored. Any unmarked error or reveal, or any
unmatched marker, fails the fixture's case.

The fixtures are invisible to the repo mypy gate (path matches the
`.*/test_.` exclude in pyproject) and to pytest collection (not named
test_*.py); test_gate_excludes_fixtures pins the former.
"""

import os
from pathlib import Path
import re
import subprocess
import sys

import pytest

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
FIXTURES_DIR = HERE / "test_typing_fixtures"
FIXTURES_REL = "dimos/pure/test_typing_fixtures"
CASES = sorted(p.name for p in FIXTURES_DIR.glob("case_*.py"))

_DIAG_RE = re.compile(
    r"^(?P<path>[^:]+):(?P<line>\d+): (?P<sev>error|note): (?P<msg>.*?)(?:  \[(?P<code>[\w-]+)\])?$"
)
_E_MARK_RE = re.compile(r"# E\[(?P<code>[\w-]+)\]: (?P<sub>.+?)\s*$")
_R_MARK_RE = re.compile(r"# R: (?P<type>.+?)\s*$")
_REVEAL_PREFIX = 'Revealed type is "'


def _run_mypy(paths):
    cmd = [
        sys.executable,
        "-m",
        "mypy",
        "--config-file",
        "pyproject.toml",
        "--cache-dir",
        os.devnull,
        "--no-error-summary",
        "--no-color-output",
        *paths,
    ]
    return subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True, timeout=300)


@pytest.fixture(scope="session")
def diagnostics():
    """One mypy run over all fixtures -> {fixture filename: [(line, sev, code, msg)]}."""
    proc = _run_mypy([f"{FIXTURES_REL}/{c}" for c in CASES])
    assert proc.returncode in (0, 1), f"mypy did not run cleanly:\n{proc.stdout}\n{proc.stderr}"
    per_file = {c: [] for c in CASES}
    strays = []
    for raw in proc.stdout.splitlines():
        m = _DIAG_RE.match(raw)
        if not m:
            continue
        name = Path(m["path"]).name
        entry = (int(m["line"]), m["sev"], m["code"], m["msg"])
        if m["path"].replace(os.sep, "/").startswith(FIXTURES_REL) and name in per_file:
            per_file[name].append(entry)
        elif m["sev"] == "error":  # companion notes may point at typing.py; errors may not
            strays.append(raw)
    per_file["__strays__"] = strays
    return per_file


def _expected(case: str):
    """Parse markers -> (errors: {line: (code, substring)}, reveals: {line: type})."""
    errors: dict[int, tuple[str, str]] = {}
    reveals: dict[int, str] = {}
    for lineno, text in enumerate((FIXTURES_DIR / case).read_text().splitlines(), start=1):
        if e := _E_MARK_RE.search(text):
            errors[lineno] = (e["code"], e["sub"])
        elif r := _R_MARK_RE.search(text):
            reveals[lineno] = r["type"]
    return errors, reveals


def _actual(case: str, diags):
    """Split a fixture's diagnostics -> (errors: {line: (code, msg)}, reveals: {line: type})."""
    module_prefix = f"dimos.pure.test_typing_fixtures.{case.removesuffix('.py')}."
    errors: dict[int, tuple[str, str]] = {}
    reveals: dict[int, str] = {}
    problems: list[str] = []
    for line, sev, code, msg in diags:
        if sev == "error":
            if line in errors:
                problems.append(f"line {line}: multiple errors (restructure the fixture)")
            errors[line] = (code or "", msg)
        elif msg.startswith(_REVEAL_PREFIX):
            reveals[line] = msg[len(_REVEAL_PREFIX) : -1].replace(module_prefix, "")
    return errors, reveals, problems


@pytest.mark.parametrize("case", CASES)
def test_fixture_expectations(case, diagnostics):
    exp_errors, exp_reveals = _expected(case)
    act_errors, act_reveals, problems = _actual(case, diagnostics[case])

    for line, revealed in sorted(act_reveals.items()):
        want = exp_reveals.pop(line, None)
        if want is None:
            problems.append(f"line {line}: unmarked reveal {revealed!r}")
        elif revealed != want:
            problems.append(f"line {line}: revealed {revealed!r}, marker says {want!r}")
    for line, want in sorted(exp_reveals.items()):
        problems.append(f"line {line}: marker expects reveal {want!r}, mypy emitted none")

    for line, (code, msg) in sorted(act_errors.items()):
        want = exp_errors.pop(line, None)
        if want is None:
            problems.append(f"line {line}: unmarked error [{code}] {msg}")
        else:
            want_code, want_sub = want
            if code != want_code:
                problems.append(f"line {line}: error code [{code}], marker says [{want_code}]")
            if want_sub not in msg:
                problems.append(f"line {line}: {want_sub!r} not in message {msg!r}")
    for line, (want_code, want_sub) in sorted(exp_errors.items()):
        problems.append(f"line {line}: marker expects [{want_code}] {want_sub!r}, no error emitted")

    assert not problems, f"{case}:\n  " + "\n  ".join(problems)


def test_only_fixture_diagnostics(diagnostics):
    """typing.py (analyzed as an import) and everything else must stay silent."""
    assert diagnostics["__strays__"] == []


def test_gate_excludes_fixtures():
    """Recursive discovery (what the gate does) must not see the fixtures.

    If this fails, the deliberate type errors in test_typing_fixtures/ are
    about to break the repo mypy gate: the dir no longer matches the
    `.*/test_.` exclude in pyproject.toml.
    """
    proc = _run_mypy([FIXTURES_REL])
    out = proc.stdout + proc.stderr
    assert proc.returncode == 2 and "no .py[i] files" in out, out


def test_fixture_format_stability():
    """ruff format must be a no-op on fixtures — markers ride their lines."""
    proc = subprocess.run(
        [sys.executable, "-m", "ruff", "format", "--check", str(FIXTURES_DIR)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr


def test_runtime_surface():
    """The typing module imports at runtime; stubs raise, protocols subscript."""
    import dimos.pure.typing as pure_typing

    for name in pure_typing.__all__:
        assert hasattr(pure_typing, name)
    assert pure_typing.Stateless[int, str] is not None  # structural aliases work
    surface = pure_typing.EngineSurface()
    with pytest.raises(NotImplementedError):
        surface.over()
    with pytest.raises(NotImplementedError):
        _ = surface.i
    with pytest.raises(NotImplementedError):
        _ = surface.o
