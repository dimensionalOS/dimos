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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Command line entrypoints for spatial benchmark generation."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

from dimos.benchmark.spatial.corpus_loader import SpatialCorpusLoader, SpatialCorpusSelection
from dimos.benchmark.spatial.models import MapVariant, Predicate
from dimos.benchmark.spatial.smoke import (
    PilotSourceError,
    SmokeGateError,
    generate_smoke_corpus,
    run_pilot_generation,
)
from dimos.benchmark.spatial.validation import validate_release, write_validation_report
from dimos.benchmark.spatial.viewer import (
    RealViserReadOnlyBoundary,
    SpatialCorpusViserView,
    ViserReadOnlyBoundary,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="spatial-benchmark")
    subparsers = parser.add_subparsers(dest="command", required=True)
    smoke = subparsers.add_parser("generate-smoke")
    smoke.add_argument("--output", required=True, type=Path)
    pilot = subparsers.add_parser("generate-pilot")
    pilot.add_argument("--output", required=True, type=Path)
    pilot.add_argument("--smoke-root", required=True, type=Path)
    pilot.add_argument("--source-root", type=Path)
    pilot.add_argument("--workers", type=int)
    validate = subparsers.add_parser("validate-release")
    validate.add_argument("--root", required=True, type=Path)
    view = subparsers.add_parser("view")
    view.add_argument("--root", required=True, type=Path)
    view.add_argument("--scene-id")
    view.add_argument("--trajectory-id")
    view.add_argument("--question-id")
    view.add_argument("--predicate", choices=[predicate.value for predicate in Predicate])
    view.add_argument("--variant", choices=[variant.value for variant in MapVariant])
    view.add_argument("--instance-id")
    view.add_argument(
        "--public-only",
        "--no-oracle",
        dest="public_only",
        action="store_true",
        help="Hide all private oracle context and render only agent-visible evidence.",
    )
    view.add_argument("--host", default="127.0.0.1")
    view.add_argument("--port", default=8080, type=int)
    view.add_argument("--once", action="store_true")
    args = parser.parse_args(argv)
    if args.command == "generate-smoke":
        report = generate_smoke_corpus(args.output)
        print(
            f"smoke complete={report.complete} report={args.output / 'smoke_validation_report.json'}"
        )
        return 0 if report.complete else 1
    if args.command == "generate-pilot":
        try:
            run_pilot_generation(
                args.output, args.smoke_root, args.source_root, workers=args.workers
            )
        except SmokeGateError as error:
            print(str(error))
            return 2
        except PilotSourceError as error:
            print(str(error))
            return 3
        print(f"pilot generation permitted after smoke gate: {args.output}")
        return 0
    if args.command == "validate-release":
        validation_report = validate_release(args.root)
        path = write_validation_report(args.root, validation_report)
        print(f"release complete={validation_report.complete} report={path}")
        return 0 if validation_report.complete else 1
    if args.command == "view":
        return _view_command(args)
    return 2


def _view_command(args: argparse.Namespace) -> int:
    oracle_root = args.root / "__oracle_disabled__" if args.public_only else args.root / "oracle"
    loader = SpatialCorpusLoader(args.root, oracle_root=oracle_root)
    selection = SpatialCorpusSelection(
        scene_id=args.scene_id,
        trajectory_id=args.trajectory_id,
        question_id=args.question_id,
        predicate=Predicate(args.predicate) if args.predicate is not None else None,
        variant=MapVariant(args.variant) if args.variant is not None else None,
        instance_id=args.instance_id,
    )
    if args.once:
        boundary = ViserReadOnlyBoundary()
    else:
        try:
            boundary = RealViserReadOnlyBoundary(host=args.host, port=args.port)
        except ImportError as error:
            print(str(error))
            return 4
    view = SpatialCorpusViserView(loader, boundary)
    instance = view.start_qa_review(selection)
    counts = _draw_counts(boundary.commands)
    url = getattr(boundary, "url", f"http://{args.host}:{args.port}")
    print(
        "viewer "
        f"url={url} scene_id={instance.scene.scene_id} "
        f"trajectory_id={instance.trajectory.trajectory_id} "
        f"question_id={instance.question.question_id} "
        f"instance_id={instance.instance.instance_id} variant={instance.instance.variant.value} "
        f"commands={len(boundary.commands)} counts={counts}"
    )
    if not args.once and isinstance(boundary, RealViserReadOnlyBoundary):
        try:
            boundary.block_forever()
        except KeyboardInterrupt:
            return 0
    return 0


def _draw_counts(commands: Sequence[object]) -> str:
    counts: dict[str, int] = {}
    for command in commands:
        kind = getattr(command, "kind", "unknown")
        counts[str(kind)] = counts.get(str(kind), 0) + 1
    return ",".join(f"{kind}:{count}" for kind, count in sorted(counts.items()))


if __name__ == "__main__":
    raise SystemExit(main())
