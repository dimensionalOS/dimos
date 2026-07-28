"""Offline CLI for inspecting or explicitly sending selected Go2 candidate frames."""

from __future__ import annotations

import argparse
import base64
from collections.abc import Callable, Sequence
import json
from pathlib import Path
import sys
from typing import Any, TextIO

from dimos.perception.target_verification import (
    CandidateEvidenceBundle,
    OpenAIResponsesVisionVerifier,
    PersistedCandidateLoader,
    VisionVerifier,
)

VerifierFactory = Callable[[str], VisionVerifier]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Load selected Go2 VisualMemory frames locally. No image is uploaded "
            "unless --send is explicitly provided."
        )
    )
    parser.add_argument("--memory", required=True, help="Path to visual_memory.pkl")
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--target", required=True, help="Natural-language target description")
    parser.add_argument(
        "--frame",
        action="append",
        required=True,
        dest="frame_ids",
        help="Allowlisted VisualMemory frame ID; repeat one to three times",
    )
    parser.add_argument("--send", action="store_true", help="Upload only selected frames")
    parser.add_argument("--model", default="gpt-5.6-terra")
    parser.add_argument("--output", help="Optional path for sanitized JSON result")
    return parser


def _default_verifier_factory(model: str) -> VisionVerifier:
    return OpenAIResponsesVisionVerifier(model=model)


def _sanitized_bundle(bundle: CandidateEvidenceBundle) -> dict[str, Any]:
    return {
        "candidate_id": bundle.candidate_id,
        "target_description": bundle.target_description,
        "selected_frames": [
            {
                "frame_id": view.frame_id,
                "width": view.width,
                "height": view.height,
                "mime_type": view.mime_type,
                "encoded_bytes": len(base64.b64decode(view.jpeg_base64)),
            }
            for view in bundle.views
        ],
    }


def run(
    argv: Sequence[str] | None = None,
    *,
    verifier_factory: VerifierFactory = _default_verifier_factory,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        bundle = PersistedCandidateLoader(args.memory).load(
            candidate_id=args.candidate_id,
            target_description=args.target,
            frame_ids=args.frame_ids,
        )
        payload = {
            "mode": "send_requested" if args.send else "dry_run",
            "upload_authorized": bool(args.send),
            "model": args.model if args.send else None,
            **_sanitized_bundle(bundle),
        }
        if args.send:
            verification = verifier_factory(args.model).verify(bundle)
            payload["verification"] = verification.model_dump(mode="json")

        serialized = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
        stdout.write(serialized)
        if args.output:
            output_path = Path(args.output)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(serialized, encoding="utf-8")
        return 0
    except (OSError, ValueError) as exc:
        stderr.write(f"offline verification failed: {exc}\n")
        return 2


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
