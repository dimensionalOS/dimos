from __future__ import annotations

import argparse
from pathlib import Path

from dimos.experimental.dogops.config_loader import (
    DEFAULT_MANIFEST,
    DEFAULT_MISSION,
    DEFAULT_POLICY,
    DEFAULT_SITE,
    load_dogops_config,
)
from dimos.experimental.dogops.dashboard import serve_dashboard
from dimos.experimental.dogops.mission_engine import run_offline_simulation
from dimos.experimental.dogops.report import render_report_markdown
from dimos.experimental.dogops.store import DogOpsStore


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="dogops", description="DogOps offline CLI")
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate = subparsers.add_parser("validate", help="Validate DogOps YAML configs")
    _add_config_args(validate)

    simulate = subparsers.add_parser("simulate", help="Run the offline DogOps mission")
    _add_config_args(simulate)
    simulate.add_argument("--out", default=".dogops/runs/latest")

    report = subparsers.add_parser("report", help="Regenerate a report from a run directory")
    report.add_argument("--run", default=".dogops/runs/latest")
    report.add_argument("--out", default=None)

    serve = subparsers.add_parser("serve", help="Serve a local dashboard for a run directory")
    serve.add_argument("--run", default=".dogops/runs/latest")
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--port", type=int, default=8765)

    return parser


def _add_config_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--site", default=str(DEFAULT_SITE))
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--mission", default=str(DEFAULT_MISSION))
    parser.add_argument("--policy", default=str(DEFAULT_POLICY))


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "validate":
        config = load_dogops_config(args.site, args.manifest, args.mission, args.policy)
        print(
            "validated "
            f"site={config.site.site_id} "
            f"manifest={config.manifest.manifest_id} "
            f"mission={config.mission.mission_id}"
        )
        return 0

    if args.command == "simulate":
        state = run_offline_simulation(
            site=args.site,
            manifest=args.manifest,
            mission=args.mission,
            policy=args.policy,
            out=args.out,
        )
        print(f"run_id={state.run.id}")
        print(f"state={getattr(state.run.state, 'value', state.run.state)}")
        print(f"report={Path(args.out) / 'report.md'}")
        return 0

    if args.command == "report":
        store = DogOpsStore.load_existing(args.run)
        state = store.state
        assert state is not None
        content = render_report_markdown(state)
        out_path = Path(args.out) if args.out else Path(args.run) / "report.md"
        out_path.write_text(content, encoding="utf-8")
        store.write_report(state.run.id)
        print(f"report={out_path}")
        return 0

    if args.command == "serve":
        serve_dashboard(args.run, host=args.host, port=args.port)
        return 0

    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
