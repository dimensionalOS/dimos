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

"""Drive an eval suite across a pool of worker containers, one case per exec.

Runs on the host with only Python and the docker CLI; dimos never has to be
installed here. The pool is the ``worker`` service of docker/evals/compose.yaml:

    docker compose -f docker/evals/compose.yaml up -d --scale worker=4
    python docker/evals/dispatch.py --suite dimos.evals.suites.typesafe_nav \\
        --agent dimos.evals.agents.typesafe_policy \\
        --set scene_json=dimos/evals/suites/scenes/apartment_detections.json

Every container gets one thread that pulls the next unclaimed case from a
queue and execs ``run_case.py run`` for it; that boots a dimos + DimSim inside
the container, runs the case, tears them down, and writes the runner's
artifacts under ``/state/pool/<run-id>/cases/<case-id>/``. When the queue
drains, ``run_case.py collect`` merges everything into one ``results.jsonl``
and ``summary.json`` next to them.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

RUN_CASE = "/app/docker/evals/run_case.py"
STATE = "/state/pool"
COMPOSE_FILE = Path(__file__).with_name("compose.yaml")


def docker(*args: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["docker", *args], text=True, **kwargs)  # type: ignore[call-overload]


def pool(compose: Path, workers: int | None) -> list[str]:
    """Container ids of the worker service, scaled to ``workers`` when given."""
    if workers:
        docker(
            "compose", "-f", str(compose), "up", "-d", "--scale", f"worker={workers}", check=True
        )
    out = docker(
        "compose", "-f", str(compose), "ps", "-q", "worker", capture_output=True, check=True
    )
    ids = [c for c in out.stdout.split() if c]
    if not ids:
        sys.exit(
            f"no running worker containers; start them with: docker compose -f {compose} up -d --scale worker=N"
        )
    return ids


def list_cases(container: str, suite: str, tags: str, limit: int) -> list[str]:
    out = docker(
        "exec",
        container,
        "python",
        RUN_CASE,
        "list",
        "--suite",
        suite,
        "--tags",
        tags,
        "--limit",
        str(limit),
        capture_output=True,
        check=True,
    )
    return json.loads(out.stdout)


class Pool:
    def __init__(self, containers: list[str], run_dir: str, forwarded: list[str]) -> None:
        self.containers = containers
        self.run_dir = run_dir
        self.forwarded = forwarded
        self.todo: queue.Queue[str] = queue.Queue()
        self.done: dict[str, int] = {}
        self.lock = threading.Lock()
        self.running: dict[str, subprocess.Popen[str]] = {}

    def worker(self, slot: int, container: str) -> None:
        name = f"worker-{slot}"
        while True:
            try:
                case_id = self.todo.get_nowait()
            except queue.Empty:
                return
            out = f"{self.run_dir}/cases/{case_id}"
            command = [
                "docker",
                "exec",
                container,
                "python",
                RUN_CASE,
                "run",
                *self.forwarded,
                "--case",
                case_id,
                "--out",
                out,
            ]
            started = time.monotonic()
            log(f"{name} start  {case_id}")
            proc = subprocess.Popen(
                command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
            )
            with self.lock:
                self.running[container] = proc
            output, _ = proc.communicate()
            with self.lock:
                self.running.pop(container, None)
                self.done[case_id] = proc.returncode
            tail = output.strip().splitlines()[-1] if output.strip() else ""
            log(
                f"{name} finish {case_id} exit={proc.returncode} {time.monotonic() - started:.0f}s  {tail}"
            )
            if proc.returncode:
                log(output, end="")

    def run(self, case_ids: list[str]) -> None:
        for case_id in case_ids:
            self.todo.put(case_id)
        threads = [
            threading.Thread(target=self.worker, args=(slot, c), daemon=True, name=f"worker-{slot}")
            for slot, c in enumerate(self.containers)
        ]
        for t in threads:
            t.start()
        try:
            for t in threads:
                while t.is_alive():
                    t.join(0.5)
        except KeyboardInterrupt:
            log("interrupted: stopping in-flight cases")
            with self.lock:
                running = dict(self.running)
            for container, proc in running.items():
                # The exec client dying does not kill the in-container process.
                docker("exec", container, "pkill", "-TERM", "-f", RUN_CASE)
                proc.terminate()
            raise


def log(message: str, end: str = "\n") -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", end=end, flush=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--suite", required=True, help="suite module, e.g. dimos.evals.suites.typesafe_nav"
    )
    parser.add_argument(
        "--agent", required=True, help="agent module, e.g. dimos.evals.agents.typesafe_policy"
    )
    parser.add_argument(
        "--set", action="append", default=[], metavar="FIELD=VALUE", help="agent field override"
    )
    parser.add_argument("--allow", default=None, help="comma-separated tool allowlist")
    parser.add_argument("--tags", default="", help="comma-separated tag filter")
    parser.add_argument("--limit", type=int, default=0, help="run at most N cases")
    parser.add_argument(
        "--workers", type=int, default=None, help="scale the pool to N containers first"
    )
    parser.add_argument(
        "--compose", type=Path, default=COMPOSE_FILE, help="compose file of the pool"
    )
    parser.add_argument(
        "--containers",
        nargs="*",
        default=None,
        help="use these containers instead of the compose pool",
    )
    parser.add_argument(
        "--run-id", default=None, help="name of this run under /state/pool (default: timestamp)"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="exercise the plumbing without launching simulators"
    )
    args = parser.parse_args(argv)

    containers = args.containers or pool(args.compose, args.workers)
    case_ids = list_cases(containers[0], args.suite, args.tags, args.limit)
    if not case_ids:
        sys.exit("the selection matched no cases")

    run_id = args.run_id or time.strftime("run-%Y%m%d-%H%M%S")
    run_dir = f"{STATE}/{run_id}"
    manifest = {
        "schema_version": 1,
        "suite": args.suite,
        "agent": args.agent,
        "set": args.set,
        "allow": args.allow,
        "selection": {"tags": args.tags, "limit": args.limit},
        "cases": case_ids,
        "workers": len(containers),
        "dry_run": args.dry_run,
    }
    docker("exec", containers[0], "mkdir", "-p", run_dir, check=True)
    docker(
        "exec",
        "-i",
        containers[0],
        "sh",
        "-c",
        f"cat > {run_dir}/manifest.json",
        input=json.dumps(manifest, indent=2),
        check=True,
    )

    forwarded = ["--suite", args.suite, "--agent", args.agent]
    for item in args.set:
        forwarded += ["--set", item]
    if args.allow is not None:
        forwarded += ["--allow", args.allow]
    if args.dry_run:
        forwarded.append("--dry-run")

    log(f"{len(case_ids)} cases over {len(containers)} workers -> {run_dir}")
    workers = Pool(containers, run_dir, forwarded)
    workers.run(case_ids)

    log("collecting")
    collected = docker("exec", containers[0], "python", RUN_CASE, "collect", "--run-dir", run_dir)
    failed = sum(1 for code in workers.done.values() if code)
    if failed:
        log(f"{failed} case exec(s) exited non-zero; see the output above")
    return 1 if collected.returncode or failed else 0


if __name__ == "__main__":
    sys.exit(main())
