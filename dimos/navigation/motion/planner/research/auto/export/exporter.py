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

"""Export a self-contained autoresearch lab from this package.

    python -m dimos.navigation.motion.planner.research.auto.export <dest>

The output is a standalone research workspace: the referee copied as
`referee/`, the current rust planner seeded as `candidate/`, the bench/
gate harness, generated lock files (content hashes computed from the
exported bytes), warmed caches, a venv, and a git repo — ready for
`/evo:discover`. See templates/README.md.tmpl for what the lab looks like
from the inside.

Templates are copied byte-for-byte; everything that varies per export lives
in GENERATED files (locks, gates.json, README, PROVENANCE, pyproject), so
the executable harness is reviewable here in-repo.
"""

from __future__ import annotations

from datetime import date
from importlib import metadata
import json
from pathlib import Path
import shutil
import string
import subprocess
import sys

from . import caches, locks

HERE = Path(__file__).resolve().parent
PLANNER = HERE.parents[2]  # dimos/navigation/motion/planner
PACKAGE = PLANNER / "referee"  # the copyable referee unit (relative imports throughout)
RUST = PLANNER / "rust"  # the production crate, seeded as the lab's candidate/

# Baseline of the seeded candidate, measured on the source package's battery
# (56 worlds, gen 40 seed 0) at kit-authoring time. gold/consistency are
# deterministic; speed/referee carry ordinary timing noise. PROVENANCE.md
# labels these as measured-at-source numbers; step 6 of a new lab is
# reproducing them with its own `./eval`.
BASELINE = {
    "baseline_referee": 106.66,
    "baseline_gold": 0.9629,
    "baseline_consistency": 0.9473,
    "baseline_speed": 0.89,
    "gold_headroom": 3.7,
}

REFEREE_EXCLUDE = {
    "__pycache__",
    ".se2_cache.pkl",
    ".gen_cache.pkl",
    ".gitignore",
    "README.md",
    "test_gold.py",
}

BENCH_TEMPLATES = [
    "_evo_inline.py",
    "battery.py",
    "check_rules.py",
    "ext_invariants.py",
    "extcheck",
    "fitness.py",
    "generalization.py",
    "parity",
    "quality.py",
    "run",
]
EXECUTABLE = {"run", "eval", "parity", "extcheck", "bench_guard.py"}


def _render(name: str, dest: Path, values: dict[str, object]) -> None:
    tmpl = string.Template((HERE / "templates" / name).read_text(encoding="utf-8"))
    dest.write_text(tmpl.substitute({k: str(v) for k, v in values.items()}), encoding="utf-8")


def _git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True, check=True
    ).stdout.strip()


def say(msg: str) -> None:
    print(f"export: {msg}", file=sys.stderr)


def run(
    dest: Path,
    *,
    force: bool = False,
    no_venv: bool = False,
    no_warm: bool = False,
    no_build: bool = False,
) -> None:
    dest = dest.resolve()

    # -- preflight ---------------------------------------------------------
    if dest.exists() and any(dest.iterdir()):
        if not force:
            sys.exit(f"{dest} is not empty (--force to overwrite)")
        shutil.rmtree(dest)
    repo = PACKAGE
    while not (repo / ".git").exists() and repo.parent != repo:
        repo = repo.parent
    try:
        source_commit = _git(repo, "rev-parse", "HEAD")
        dirty = _git(
            repo,
            "status",
            "--porcelain",
            "--untracked-files=no",
            "--",
            str(PACKAGE.relative_to(repo)),
        )
    except (subprocess.CalledProcessError, OSError):
        source_commit, dirty = "unknown", ""
    if dirty and not force:
        sys.exit("the source package has uncommitted changes — commit them or --force:\n" + dirty)

    # -- copy the referee --------------------------------------------------
    say("copying referee/")
    (dest / "referee").mkdir(parents=True)
    for path in sorted(PACKAGE.rglob("*")):
        rel = path.relative_to(PACKAGE)
        if (
            path.is_dir()
            or any(part in REFEREE_EXCLUDE for part in rel.parts)
            or rel.name in REFEREE_EXCLUDE
            or rel.name.startswith("test_")
        ):
            continue
        out = dest / "referee" / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, out)

    # -- seed the candidate ------------------------------------------------
    say("seeding candidate/ from rust/")
    shutil.copytree(
        RUST, dest / "candidate", ignore=shutil.ignore_patterns("target", "__pycache__")
    )

    # -- harness + docs ----------------------------------------------------
    say("laying down bench/ and lab files")
    (dest / "bench").mkdir()
    (dest / ".evo" / "cache").mkdir(parents=True)
    for name in BENCH_TEMPLATES:
        shutil.copy2(HERE / "templates" / name, dest / "bench" / name)
    shutil.copy2(HERE / "templates" / "eval", dest / "eval")
    shutil.copy2(HERE / "templates" / "gitignore", dest / ".gitignore")
    shutil.copy2(HERE / "templates" / "bench_guard.py", dest / ".evo" / "bench_guard.py")
    shutil.copy2(HERE / "templates" / "bench.env", dest / ".evo" / "bench.env")
    for name in EXECUTABLE:
        for p in (dest / "bench" / name, dest / name, dest / ".evo" / name):
            if p.exists():
                p.chmod(p.stat().st_mode | 0o755)

    deps = {name: metadata.version(name) for name in ("numpy", "scipy", "pydantic")}
    values: dict[str, object] = {
        **BASELINE,
        "fitness_max": 26,
        "export_date": date.today().isoformat(),
        "source_repo": str(repo),
        "source_commit": source_commit,
        "dirty_note": " (package DIRTY at export)" if dirty else "",
        "planner_sha256": "",  # filled below, after candidate/ exists
        "referee_sha256": "",  # filled below, after referee.lock
        "dep_versions": ", ".join(f"{k} {v}" for k, v in deps.items()),
        "cache_note": "not warmed (--no-warm)",
        "pinned_deps": "\n".join(f'    "{k}=={v}",' for k, v in deps.items()),
    }
    _render("gates.json.tmpl", dest / "gates.json", values)
    _render("README.md.tmpl", dest / "README.md", values)
    _render("pyproject.toml.tmpl", dest / "pyproject.toml", values)
    # Pin the lab's python to the exporting interpreter's minor version:
    # referee.lock pins live co_code hashes, and CPython bytecode changes
    # across minor versions -- a lab synced under a different python would
    # fail ext_invariants for what is really an interpreter mismatch.
    (dest / ".python-version").write_text(f"{sys.version_info.major}.{sys.version_info.minor}\n")
    locks.write_frozen(dest)

    # -- venv --------------------------------------------------------------
    python = sys.executable
    if not no_venv:
        say("uv sync")
        subprocess.run(["uv", "sync", "--quiet"], cwd=dest, check=True)
        python = str(dest / ".venv" / "bin" / "python")

    # -- caches ------------------------------------------------------------
    if no_warm:
        cache_note = "not warmed (--no-warm); first battery will be slow"
    else:
        say("warming caches")
        cache_note = caches.warm(dest, python, PACKAGE)
    values["cache_note"] = cache_note

    # -- locks (order matters: referee -> harness -> trust) ----------------
    say("generating locks")
    locks.write_referee_lock(dest, python, str(repo), source_commit)
    locks.write_harness_lock(dest)
    locks.write_trust_lock(dest)
    lock = json.loads((dest / ".evo" / "referee.lock").read_text())
    values["referee_sha256"] = _tree_hash_of(dest)
    values["planner_sha256"] = _planner_fingerprint(dest)
    _render("PROVENANCE.md.tmpl", dest / "PROVENANCE.md", values)
    assert lock["files"], "referee.lock has no files"

    # -- smoke -------------------------------------------------------------
    for name, cmd in (
        ("bench_guard", ["python3", str(dest / ".evo" / "bench_guard.py"), str(dest)]),
        ("check_rules", ["python3", str(dest / "bench" / "check_rules.py")]),
    ):
        say(f"smoke: {name}")
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    if not no_build:
        say("smoke: cargo build + 2-world battery")
        subprocess.run(
            ["bash", str(dest / "bench" / "run"), "--gen", "2", "--seed", "0", "--no-curated"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        crumb = dest / ".ext" / "summary_g2_s0_c0.json"
        s = json.loads(crumb.read_text())
        assert s["dq"] == 0 and s["fitness"] > 0, f"smoke battery failed: {s}"
        say(f"smoke battery ok: fitness {s['fitness']}/{s['fitness_max']} on {s['worlds']} worlds")

    # -- git ---------------------------------------------------------------
    say("git init + initial commit")
    subprocess.run(["git", "init", "-q", "-b", "master"], cwd=dest, check=True)
    subprocess.run(["git", "add", "-A"], cwd=dest, check=True)
    subprocess.run(
        [
            "git",
            "-c",
            "user.name=autoresearch-export",
            "-c",
            "user.email=autoresearch-export@localhost",
            "commit",
            "-q",
            "-m",
            f"lab exported from {repo.name} @ {source_commit[:9]}\n\n"
            f"baseline: referee {BASELINE['baseline_referee']}/111, "
            f"gold {BASELINE['baseline_gold']}, dq 0",
        ],
        cwd=dest,
        check=True,
    )
    say(f"done: {dest}")
    say(
        "next: cd there, `uv sync` if skipped, `./eval`, then /evo:discover "
        "(gates.json has the benchmark + gate commands)"
    )


def _tree_hash_of(dest: Path) -> str:
    import hashlib

    h = hashlib.sha256()
    for path in sorted((dest / "referee").rglob("*.py")):
        h.update(path.relative_to(dest).as_posix().encode())
        h.update(path.read_bytes())
    return h.hexdigest()


def _planner_fingerprint(dest: Path) -> str:
    import hashlib

    h = hashlib.sha256()
    paths = sorted((dest / "candidate" / "src").rglob("*.rs"))
    paths += [
        p
        for p in (dest / "candidate" / "Cargo.toml", dest / "candidate" / "Cargo.lock")
        if p.exists()
    ]
    for path in paths:
        h.update(path.relative_to(dest).as_posix().encode())
        h.update(path.read_bytes())
    return h.hexdigest()
