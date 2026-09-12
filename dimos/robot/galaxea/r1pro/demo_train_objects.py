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

"""Persistent local collection, ACT training and held-out physics evaluation."""

import argparse
from collections import Counter
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any


def write_json(path: Path, value: dict[str, Any]) -> None:
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def run_pipeline(args: argparse.Namespace) -> None:
    root = Path(__file__).resolve().parents[4]
    job = args.output.resolve()
    job.mkdir(parents=True, exist_ok=True)
    status: dict[str, Any] = dict(
        pid=os.getpid(), stage="starting", started=time.time(), ready=False
    )
    (job / "pid").write_text(str(os.getpid()))
    env = os.environ.copy()
    env.update(
        PYTHONPATH=str(root),
        MUJOCO_GL="egl",
        OPENBLAS_NUM_THREADS="1",
        OMP_NUM_THREADS="4",
        MKL_NUM_THREADS="4",
        HF_HUB_OFFLINE="1",
        WANDB_MODE="disabled",
    )
    env.pop("DISPLAY", None)
    native = [sys.executable]
    learned = [
        "uv",
        "run",
        "--offline",
        "--frozen",
        "--project",
        "dimos/imitation/policy/lerobot/python",
        "--with-editable",
        ".",
        "--with",
        "mujoco==3.10.0",
        "python",
    ]
    collection = job / "collection"
    dataset = job / "dataset"
    initialization = job / "initialization"
    training = job / "training"
    artifact = job / "policy"
    contract = dict(
        layouts=args.layouts,
        steps=args.steps,
        scene_package=str(args.scene_package.resolve()),
        source=str(args.source.resolve()),
        seed=args.seed,
    )
    contract_path = job / "contract.json"
    if contract_path.exists() and json.loads(contract_path.read_text()) != contract:
        raise ValueError("Cannot reuse a pipeline directory with a different contract")
    write_json(contract_path, contract)

    def run(stage: str, command: list[str], *, require_success: bool = True) -> None:
        done = job / (stage + ".done")
        if done.exists():
            return
        status.update(stage=stage, updated=time.time())
        write_json(job / "status.json", status)
        (job / "stage").write_text(stage)
        if shutil.disk_usage(job).free < 12 * 1024**3:
            raise RuntimeError("Less than 12 GiB free; stopping before the next stage")
        with (job / (stage + ".log")).open("a") as log:
            result = subprocess.run(
                command,
                cwd=root,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=False,
            )
        if require_success and result.returncode != 0:
            raise RuntimeError(f"{stage} exited {result.returncode}; inspect its log")
        done.write_text(str(result.returncode))

    try:
        run(
            "collect",
            [
                *native,
                "-m",
                "dimos.robot.galaxea.r1pro.demo_collect_objects",
                "--output",
                str(collection),
                "--scene-package",
                str(args.scene_package),
                "--start-seed",
                str(args.seed),
                "--layouts",
                str(args.layouts),
                "--choices",
                "2",
                "--occupied-max",
                "3",
                "--image-stride",
                "2",
            ],
        )
        manifest = json.loads((collection / "manifest.json").read_text())
        accepted = len(manifest["episodes"])
        attempted = accepted + len(manifest["rejected"])
        shapes = Counter(row["shape"] for row in manifest["episodes"])
        coverage = dict(accepted=accepted, attempted=attempted, shapes=dict(shapes))
        write_json(job / "coverage.json", coverage)
        if (
            accepted < max(12, args.layouts)
            or accepted / max(1, attempted) < 0.75
            or any(shapes[s] < max(3, args.layouts // 6) for s in ("bottle", "box", "cylinder"))
        ):
            raise RuntimeError(
                "Teacher coverage gate failed; inspect rejected samples before training"
            )
        run(
            "convert",
            [
                *learned,
                "-m",
                "dimos_lerobot.prepare_r1pro_dataset",
                "--source",
                str(collection),
                "--output",
                str(dataset),
            ],
        )
        run(
            "initialize",
            [
                *learned,
                "-m",
                "dimos_lerobot.prepare_object_act",
                "--source",
                str(args.source),
                "--dataset",
                str(dataset),
                "--output",
                str(initialization),
            ],
        )
        # Keep paired target choices from one layout entirely on one side of
        # the diagnostic loss split. Physical test scenes use separate seeds.
        seeds = list(dict.fromkeys(row["seed"] for row in manifest["episodes"]))
        held_out = set(seeds[-max(1, len(seeds) // 10) :])
        n_eval = sum(row["seed"] in held_out for row in manifest["episodes"])
        eval_fraction = (n_eval - 0.25) / accepted
        write_json(
            job / "split.json",
            dict(
                train_seeds=[s for s in seeds if s not in held_out],
                validation_seeds=sorted(held_out),
                validation_episodes=n_eval,
                eval_fraction=eval_fraction,
            ),
        )
        run(
            "train",
            [
                *learned,
                "-m",
                "lerobot.scripts.lerobot_train",
                "--dataset.repo_id=local/r1pro-object-packing",
                "--dataset.root=" + str(dataset),
                f"--dataset.eval_split={eval_fraction}",
                "--policy.path=" + str(initialization),
                "--policy.device=cuda",
                "--policy.push_to_hub=false",
                "--policy.optimizer_lr=.00005",
                "--policy.optimizer_lr_backbone=.00001",
                "--steps=" + str(args.steps),
                "--batch_size=32",
                "--num_workers=6",
                "--env_eval_freq=0",
                "--eval_steps=2500",
                "--max_eval_samples=512",
                "--log_freq=200",
                "--save_freq=2500",
                "--wandb.enable=false",
                "--output_dir=" + str(training),
            ],
        )
        run(
            "export",
            [
                *learned,
                "-m",
                "dimos_lerobot.prepare_r1pro_deployment",
                "--source",
                str(training / "checkpoints/last/pretrained_model"),
                "--output",
                str(artifact),
                "--objects",
                "--action-steps",
                "20",
            ],
        )
        evaluation = [
            "-m",
            "dimos_lerobot.demo_object_packing",
            "--artifact",
            str(artifact),
            "--scene-package",
            str(args.scene_package),
            "--no-viewer",
        ]
        run(
            "evaluate-single",
            learned
            + evaluation
            + [
                "--output",
                str(job / "eval-single"),
                "--start-seed",
                "200000",
                "--layouts",
                "12",
                "--occupied-max",
                "3",
                "--single-pick",
            ],
            require_success=False,
        )
        run(
            "evaluate-sequences",
            learned
            + evaluation
            + ["--output", str(job / "eval-sequences"), "--start-seed", "210000", "--layouts", "8"],
            require_success=False,
        )
        single = json.loads((job / "eval-single/result.json").read_text())
        sequences = json.loads((job / "eval-sequences/result.json").read_text())
        status.update(
            stage="evaluated",
            single_successes=single["successes"],
            single_total=single["total"],
            sequence_successes=sequences["successes"],
            sequence_total=sequences["total"],
            requires_native_validation=True,
            ready=False,
        )
        (job / "exit-code").write_text("0")
    except Exception as exc:
        status.update(stage="failed", error=str(exc))
        (job / "exit-code").write_text("1")
        raise
    finally:
        status["updated"] = time.time()
        write_json(job / "status.json", status)
        (job / "stage").write_text(str(status["stage"]))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--scene-package", type=Path, required=True)
    parser.add_argument("--layouts", type=int, default=64)
    parser.add_argument("--steps", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=110000)
    parser.add_argument("--background", action="store_true")
    args = parser.parse_args()
    if args.layouts < 8 or args.steps < 1 or args.seed < 0:
        parser.error("Use at least eight layouts, positive training steps and a nonnegative seed")
    if args.background:
        args.output.mkdir(parents=True, exist_ok=True)
        with (args.output / "supervisor.log").open("a") as log:
            child = subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "dimos.robot.galaxea.r1pro.demo_train_objects",
                    *[arg for arg in sys.argv[1:] if arg != "--background"],
                ],
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        print(f"Started persistent ACT job {child.pid}: {args.output}")
        return
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "pipeline.lock").open("a") as lease:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("This training pipeline is already running") from None
        run_pipeline(args)


if __name__ == "__main__":
    main()
