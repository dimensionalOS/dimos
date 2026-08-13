#!/usr/bin/env python3
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

"""Static enforcement of the README's rules, plus the anti-gaming rules the
scoring itself cannot see.

Mostly a grep over `candidate/src/**.rs` and `candidate/Cargo.toml`, plus a
check that the off-limits local referee copy is byte-identical to what this
lab is pinned to. Deliberately conservative: a planner that genuinely needs
one of these constructs should say so and get the rule relaxed on purpose,
rather than smuggle it past a gate.

This is anti-cheat enforcement of the HONESTY rules (determinism, honest
inputs, no cross-call state), not correctness testing of the algorithm --
the crate's behavior is a black box, gated through the referee.

Stdlib only -- gates run under bare `python3`, outside the lab's venv.
"""

from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from quality import LAB, evo_root  # noqa: E402

SRC = LAB / "candidate" / "src"
CARGO = LAB / "candidate" / "Cargo.toml"
# Generated at export time from the exported bytes -- the hashes of the
# frozen python surface travel beside the code they pin.
FROZEN_JSON = Path(__file__).resolve().parent / "frozen.json"

# (regex, why). Applied to every .rs under candidate/src except the frozen
# files, with comments stripped.
BANNED = [
    # Threading is NOT banned here, deliberately.
    #
    # avoid_ms is time.process_time(), which sums CPU across every thread in
    # the process. So threading buys nothing for free: eight threads are
    # charged eight threads' worth of CPU. A candidate that is genuinely
    # cheaper in TOTAL cpu and happens to express that with threads has earned
    # it, and a static grep refusing that is over-restriction, not safety.
    #
    # What actually constrains threading is determinism, and it still bites:
    # parallel float reductions are order-dependent, and `deterministic_across
    # _calls` (the crate's own tests) plus `check_determinism`
    # (ext_invariants.py) demand BIT-identical output across repeated calls.
    # That is enforcement by consequence rather than by pattern-match, and it
    # cannot be evaded by spelling a crate name differently.
    #
    # Note the bans below on `thread_local!` / `static mut` / OnceCell stay.
    # They are about persistent state ACROSS calls -- the memoization cheat
    # this harness had to close twice -- not about threading.
    #
    # Deterministic. Same inputs must give bit-identical output.
    (r"\bHashMap\b|\bHashSet\b|\bRandomState\b",
     "hash container with nondeterministic iteration order -- use BTreeMap/BTreeSet/Vec"),
    (r"\brand::|\bthread_rng\b|\bSmallRng\b|\bgetrandom\b", "randomness (determinism rule)"),
    (r"\bstd::time\b|\bInstant\b|\bSystemTime\b", "time-dependence (determinism rule)"),
    # Honest inputs only. plan() sees the cloud, pose, goal and embodiment.
    # Nothing else exists.
    (r"\bstd::fs\b|\bFile::|\bread_to_string\b|\binclude_str!|\binclude_bytes!|\binclude!",
     "filesystem access -- the planner may only read its plan() arguments. "
     "`include!` is banned with the rest: it is how a build script would smuggle "
     "generated tables into the binary"),
    (r"\bstd::env\b|\benv::var\b|\benv!\(", "environment access (honest-inputs rule)"),
    (r"\bstd::net\b|\bTcpStream\b", "network access (honest-inputs rule)"),
    # Reaching back into the interpreter that is judging us. planner.rs is
    # called from inside the referee's Python process; a pyo3 token there can
    # import referee.scenarios and read se2_path()'s cached gold path
    # outright, or walk the stack to judge()'s live Scenario.
    (r"\bpyo3\b|\bPython\b|\bPyModule\b|\bwith_gil\b|\bPy<|\bBound<|\bPyResult\b",
     "pyo3 outside python.rs -- that is a live handle on the referee's own "
     "interpreter, which is where the gold answers live"),
    (r"_getframe|\bPyO3_|\bpyo3::", "interpreter introspection (honest-inputs rule)"),
    # Escaping the process. avoid_ms is CPU time, which excludes children
    # and excludes blocking, so a candidate that forks a helper and waits on
    # it reports ~0 ms and collects the entire speed pillar.
    (r"\bstd::process\b|\bCommand::new\b|\bstd::os::|\bfork\b",
     "subprocess/OS escape -- CPU-time scoring excludes children, so this "
     "fakes the speed pillar"),
    (r"\bstd::io\b", "io -- the planner has nothing legitimate to read or write"),
    # Cross-call memoization. avoid_ms is min-of-repeats over identical plan()
    # calls, so a cache keyed on the input reports ~0 ms and hands over the
    # speed pillar without planning anything faster. plan() is a free
    # function, so such a cache needs module-level mutable state.
    (r"\bstatic\s+mut\b|\bthread_local!|\blazy_static\b|\bonce_cell\b|\bOnceCell\b|\bOnceLock\b",
     "module-level mutable state -- reads as a cross-call cache, which fakes the speed pillar"),
    (r"\bMutex\b|\bRwLock\b|\bAtomicUsize\b|\bAtomicU64\b|\bAtomicBool\b",
     "shared mutable state -- same concern, plus it implies threading"),
    (r"\bunsafe\b", "unsafe -- not needed here, and it is the only route to `static mut`"),
]

# Interface frozen. Belt and braces alongside the hash freeze.
REQUIRED = [
    (CARGO, r'name = "dimos_motion2_target"', "the [lib] name (frozen interface)"),
]

ALLOWED_DEPS = {"pyo3", "numpy"}


def strip_comments(text: str) -> str:
    """Rules apply to code, not to prose about the rules."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return "\n".join(re.sub(r"//.*$", "", line) for line in text.splitlines())


def frozen_hashes(problems: list[str]) -> dict[str, str]:
    if not FROZEN_JSON.exists():
        problems.append(f"frozen.json missing at {FROZEN_JSON} -- the frozen python "
                        f"surface is unpinned; re-export the lab")
        return {}
    return json.loads(FROZEN_JSON.read_text(encoding="utf-8"))


def check_frozen(problems: list[str], frozen: dict[str, str]) -> None:
    for rel, want in frozen.items():
        path = LAB / rel
        if not path.exists():
            problems.append(f"{rel} is missing -- it is frozen (interface rule)")
            continue
        got = hashlib.sha256(path.read_bytes()).hexdigest()
        if got != want:
            problems.append(
                f"{rel} changed -- it is frozen (the python surface and the "
                f"module layout are not the optimization target)\n"
                f"      expected {want}\n      got      {got}"
            )


def check_source(problems: list[str], frozen: dict[str, str]) -> None:
    for path in sorted(SRC.rglob("*.rs")):
        rel = path.relative_to(LAB).as_posix()
        if rel in frozen:
            continue  # frozen by hash above; pyo3 use there is its purpose
        code = strip_comments(path.read_text(encoding="utf-8"))
        for lineno, line in enumerate(code.splitlines(), 1):
            for pattern, why in BANNED:
                if re.search(pattern, line):
                    problems.append(f"{rel}:{lineno}: {why}\n      {line.strip()}")


def check_required(problems: list[str]) -> None:
    for path, pattern, why in REQUIRED:
        if not path.exists():
            problems.append(f"{path.relative_to(LAB)} is missing -- {why}")
        elif not re.search(pattern, path.read_text(encoding="utf-8")):
            problems.append(f"{path.relative_to(LAB)}: lost {why}")


def check_deps(problems: list[str]) -> None:
    """Keep the dependency surface at pyo3/numpy. A new crate is the easy way
    to smuggle in threading, a hasher, or a clock."""
    text = CARGO.read_text(encoding="utf-8")
    block = re.search(r"^\[dependencies\]$(.*?)(?=^\[|\Z)", text, flags=re.M | re.S)
    if not block:
        problems.append("candidate/Cargo.toml: no [dependencies] section found")
        return
    for line in block.group(1).splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name = line.split("=", 1)[0].strip().strip('"')
        if name and name not in ALLOWED_DEPS:
            problems.append(
                f"candidate/Cargo.toml: dependency {name!r} is not in the allowlist "
                f"{sorted(ALLOWED_DEPS)} -- ask before adding one"
            )


def check_no_build_script(problems: list[str]) -> None:
    """No build script, by any route.

    cargo runs `candidate/build.rs` automatically if the file merely exists --
    no manifest entry required -- with arbitrary code, full filesystem access,
    and the referee's gold cache one `open()` away. It would then write a
    generated source file for planner.rs to `include!`. None of the other
    checks see any of this: check_source globs only `src/**.rs`, and
    check_deps parses only `[dependencies]`.

    There is no legitimate build step for this crate, so the rule is simply:
    there is no build script."""
    strays = [p for p in (LAB / "candidate").glob("*.rs")]
    if strays:
        for p in strays:
            problems.append(
                f"{p.relative_to(LAB)}: cargo auto-detects build.rs and runs it with full "
                f"filesystem access during the build -- there is no legitimate build step "
                f"for this crate (honest-inputs rule)"
            )
    text = CARGO.read_text(encoding="utf-8")
    stripped = "\n".join(
        line for line in text.splitlines() if not line.strip().startswith("#")
    )
    if re.search(r"^\s*build\s*=", stripped, flags=re.M):
        problems.append("candidate/Cargo.toml: a `build = ` key declares a build script")
    if re.search(r"^\s*\[build-dependencies\]", stripped, flags=re.M):
        problems.append("candidate/Cargo.toml: [build-dependencies] implies a build script")


def check_referee(problems: list[str]) -> None:
    """The referee is off-limits. It is a local copy under referee/, so
    "off-limits" is enforced by content hash: every file must match
    .evo/referee.lock exactly, and nothing may appear beside the judge."""
    lock_path = evo_root() / ".evo" / "referee.lock"
    if not lock_path.exists():
        problems.append(f"referee lock missing at {lock_path}")
        return
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    want: dict[str, str] = lock.get("files", {})
    if not want:
        problems.append("referee.lock has no `files` block -- the referee is unpinned")
        return
    seen: set[str] = set()
    for rel, expected in sorted(want.items()):
        path = LAB / rel
        seen.add(rel)
        if not path.exists():
            problems.append(f"{rel} is missing -- it is part of the referee")
            continue
        got = hashlib.sha256(path.read_bytes()).hexdigest()
        if got != expected:
            problems.append(
                f"{rel} does not match referee.lock -- the referee has been modified.\n"
                f"      Scores judged by different referee code are not comparable. If the "
                f"change was intentional, re-export the lab (or update the lock), re-run "
                f"the baseline, and say so in project.md."
            )
    # Stray-file ban: nothing untracked should ever appear beside the judge --
    # a stray .py shadows imports, a stray .pkl is a cache poisoning vector.
    for path in sorted((LAB / "referee").rglob("*")):
        if path.is_dir() or "__pycache__" in path.parts:
            continue
        rel = path.relative_to(LAB).as_posix()
        if rel not in seen:
            problems.append(f"{rel}: stray file inside the referee package -- nothing "
                            f"should appear beside the judge")


def main() -> None:
    problems: list[str] = []
    frozen = frozen_hashes(problems)
    check_frozen(problems, frozen)
    check_source(problems, frozen)
    check_required(problems)
    check_deps(problems)
    check_no_build_script(problems)
    check_referee(problems)
    if problems:
        print(f"RULES FAILED ({len(problems)}):", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        sys.exit(1)
    print("rules ok: deterministic, no persistent cross-call state, honest inputs, "
          "frozen python surface, dependency allowlist, referee hash-pinned + no strays "
          "(threading is permitted -- avoid_ms charges total CPU across threads, "
          "and bit-identical output is enforced by the determinism gates)")


if __name__ == "__main__":
    main()
