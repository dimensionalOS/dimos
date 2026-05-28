from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import json
import os
import shutil
import struct
from typing import Any, Literal
import zlib

from pydantic import Field

from dimos.experimental.dogops.models import DogOpsModel


RouteActionKind = Literal[
    "scan_tags",
    "scan_qr",
    "capture_image",
    "gemini_inspect_image",
    "inspect_asset",
    "verify_work_order",
    "wait",
    "operator_prompt",
]


class EditableRouteAction(DogOpsModel):
    id: str
    kind: RouteActionKind
    label: str | None = None
    required: bool = True
    timeout_s: float = 5.0
    args: dict[str, Any] = Field(default_factory=dict)


class RouteActionResult(DogOpsModel):
    ok: bool
    state: Literal["completed", "failed", "skipped"] = "completed"
    note: str = ""
    payload: dict[str, Any] = Field(default_factory=dict)
    evidence: list[dict[str, Any]] = Field(default_factory=list)


ScanZoneHandler = Callable[[str], str | dict[str, Any]]
CaptureImageHandler = Callable[[dict[str, Any]], dict[str, Any] | None]


def execute_route_action(
    action: EditableRouteAction,
    *,
    run_dir: str | Path,
    route_run_id: str,
    waypoint_id: str,
    route_id: str | None = None,
    target_id: str | None = None,
    pose: dict[str, Any] | None = None,
    scan_zone_handler: ScanZoneHandler | None = None,
    capture_image_handler: CaptureImageHandler | None = None,
) -> RouteActionResult:
    if action.kind == "scan_tags":
        expected = [int(tag) for tag in action.args.get("expected", [])]
        return RouteActionResult(
            ok=True,
            note=f"tag scan demo result: {len(expected)} expected",
            payload={"expected_tag_ids": expected, "detected_tag_ids": expected, "source": "demo"},
            evidence=[
                {
                    "kind": "tag_detection",
                    "path": None,
                    "mime_type": "application/json",
                    "metadata": {
                        "source": "demo",
                        "expected_tag_ids": expected,
                        "detected_tag_ids": expected,
                    },
                }
            ],
        )
    if action.kind == "scan_qr":
        expected = [str(item) for item in action.args.get("expected", [])]
        zone_id = _scan_zone_id_for_action(action, target_id=target_id, waypoint_id=waypoint_id)
        if scan_zone_handler is not None and zone_id:
            scan_result = _call_scan_zone(scan_zone_handler, zone_id)
            if scan_result.get("ok") is True:
                return _qr_result_from_scan_zone(expected, scan_result)
            return RouteActionResult(
                ok=False,
                state="failed",
                note=f"QR scan zone failed: {scan_result.get('error') or 'scan_zone_failed'}",
                payload={"source": "scan_zone", "zone_id": zone_id, "scan_zone": scan_result},
            )
        payloads = expected
        if not payloads and action.args.get("payload"):
            payloads = [str(action.args["payload"])]
        if not payloads:
            return RouteActionResult(
                ok=False,
                state="failed",
                note="QR scan configured without expected payloads",
                payload={"source": "not_configured"},
            )
        return RouteActionResult(
            ok=True,
            note=f"QR scan demo result: {len(payloads)} payloads",
            payload={"expected_payloads": payloads, "detected_payloads": payloads, "source": "demo"},
            evidence=[
                {
                    "kind": "qr_detection",
                    "path": None,
                    "mime_type": "application/json",
                    "metadata": {
                        "source": "demo",
                        "expected_payloads": payloads,
                        "detected_payloads": payloads,
                    },
                }
            ],
        )
    if action.kind == "capture_image":
        image_path, source, mime_type, capture_metadata = _capture_image_path(
            run_dir=Path(run_dir),
            route_run_id=route_run_id,
            waypoint_id=waypoint_id,
            action=action,
            capture_image_handler=capture_image_handler,
        )
        return RouteActionResult(
            ok=True,
            note=(
                "live Go2 camera image captured"
                if source == "go2_camera_live"
                else (
                    "configured Go2 camera image captured"
                    if source == "go2_camera_configured"
                    else "placeholder dog-camera image captured"
                )
            ),
            payload={"source": source, "path": str(image_path)},
            evidence=[
                {
                    "kind": "image",
                    "path": str(image_path),
                    "mime_type": mime_type,
                    "metadata": {
                        "source": source,
                        "route_run_id": route_run_id,
                        "route_id": route_id,
                        "waypoint_id": waypoint_id,
                        "action_id": action.id,
                        "target": action.args.get("target") or target_id or waypoint_id,
                        "pose": pose,
                        **capture_metadata,
                    },
                }
            ],
        )
    if action.kind == "gemini_inspect_image":
        return _execute_gemini_inspect_image(
            action,
            run_dir=Path(run_dir),
            route_run_id=route_run_id,
            waypoint_id=waypoint_id,
            route_id=route_id,
            target_id=target_id,
            pose=pose,
        )
    if action.kind == "wait":
        return RouteActionResult(
            ok=True,
            note=f"wait completed ({float(action.args.get('seconds', 0.0)):.1f}s demo)",
            payload={"seconds": float(action.args.get("seconds", 0.0)), "source": "demo"},
        )
    if action.kind in {"inspect_asset", "verify_work_order", "operator_prompt"}:
        return RouteActionResult(
            ok=True,
            note=f"{action.kind} completed by deterministic demo handler",
            payload={"source": "demo", **action.args},
        )
    return RouteActionResult(ok=False, state="failed", note=f"unsupported action: {action.kind}")


def _scan_zone_id_for_action(
    action: EditableRouteAction,
    *,
    target_id: str | None,
    waypoint_id: str,
) -> str:
    for key in ("zone_id", "target_id", "target"):
        value = action.args.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return (target_id or waypoint_id).strip()


def _call_scan_zone(handler: ScanZoneHandler, zone_id: str) -> dict[str, Any]:
    raw = handler(zone_id)
    if isinstance(raw, str):
        payload = json.loads(raw)
    else:
        payload = raw
    if not isinstance(payload, dict):
        raise ValueError("scan_zone handler returned a non-object payload")
    return payload


def _qr_result_from_scan_zone(
    expected: list[str],
    scan_result: dict[str, Any],
) -> RouteActionResult:
    detected_payloads = [str(item) for item in scan_result.get("package_ids") or []]
    detected_tag_ids = [int(tag) for tag in scan_result.get("visible_tag_ids") or []]
    zone_id = str(scan_result.get("zone_id") or "")
    source = str(scan_result.get("source") or "scan_zone")
    return RouteActionResult(
        ok=True,
        note=(
            f"QR scan used scan_zone {zone_id} via {source}: "
            f"{len(detected_payloads)} package payloads"
        ),
        payload={
            "expected_payloads": expected,
            "detected_payloads": detected_payloads,
            "detected_tag_ids": detected_tag_ids,
            "source": "scan_zone",
            "scan_zone_source": source,
            "zone_id": zone_id,
            "scan_zone": scan_result,
        },
        evidence=[
            {
                "kind": "qr_detection",
                "path": None,
                "mime_type": "application/json",
                "metadata": {
                    "source": "scan_zone",
                    "scan_zone_source": source,
                    "zone_id": zone_id,
                    "expected_payloads": expected,
                    "detected_payloads": detected_payloads,
                    "detected_tag_ids": detected_tag_ids,
                    "scan_zone": scan_result,
                },
            }
        ],
    )


def _execute_gemini_inspect_image(
    action: EditableRouteAction,
    *,
    run_dir: Path,
    route_run_id: str,
    waypoint_id: str,
    route_id: str | None,
    target_id: str | None,
    pose: dict[str, Any] | None,
) -> RouteActionResult:
    from dimos.experimental.dogops.gemini_vision import inspect_images_with_gemini
    from dimos.experimental.dogops.route_run_store import RouteRunStore

    store = RouteRunStore(run_dir)
    current = store.image_evidence_for_route_run_waypoint(
        route_run_id=route_run_id,
        waypoint_id=waypoint_id,
    )
    if current is None or not current.get("path"):
        return RouteActionResult(
            ok=False,
            state="failed",
            note="Gemini inspect needs a preceding capture_image action at this waypoint",
            payload={
                "source": "gemini",
                "status": "image_missing",
                "waypoint_id": waypoint_id,
                "recommendation": "add capture_image before gemini_inspect_image",
            },
        )

    require_baseline = bool(action.args.get("require_baseline", False))
    baseline_policy = str(action.args.get("baseline_policy") or "same_waypoint_latest_previous")
    baseline = store.latest_image_evidence_for_waypoint(
        waypoint_id=waypoint_id,
        exclude_route_run_id=route_run_id,
        route_id=route_id,
        target_id=str(action.args.get("target") or target_id or ""),
        pose=pose,
        baseline_policy=baseline_policy,
    )
    if baseline is None and require_baseline:
        return RouteActionResult(
            ok=False,
            state="failed",
            note="Gemini inspect required a baseline image but none was available",
            payload={
                "source": "gemini",
                "status": "baseline_missing",
                "baseline_policy": baseline_policy,
                "waypoint_id": waypoint_id,
            },
        )

    model = str(action.args.get("model") or "gemini-2.5-flash")
    max_inline = int(action.args.get("max_image_bytes_inline") or 20_000_000)
    prompt = str(
        action.args.get("prompt")
        or "Inspect this waypoint for physical changes, safety issues, package changes, and work-order evidence."
    )
    baseline_path = baseline.get("path") if baseline else None
    gemini = inspect_images_with_gemini(
        current_image_path=str(current["path"]),
        current_mime_type=str(current.get("mime_type") or ""),
        current_evidence_id=str(current["evidence_id"]),
        baseline_image_path=str(baseline_path) if baseline_path else None,
        baseline_mime_type=str(baseline.get("mime_type") or "") if baseline else None,
        baseline_evidence_id=str(baseline.get("evidence_id")) if baseline else None,
        prompt=prompt,
        model=model,
        max_image_bytes_inline=max_inline,
        route_context={
            "route_id": route_id,
            "route_run_id": route_run_id,
            "waypoint_id": waypoint_id,
            "target_id": target_id,
            "baseline_policy": baseline_policy,
            "baseline_match": baseline.get("baseline_match") if baseline else None,
        },
    )
    payload = {
        "source": "gemini",
        "status": gemini.status,
        "model": model,
        "current_evidence_id": current["evidence_id"],
        "baseline_policy": baseline_policy,
        "baseline_match": baseline.get("baseline_match") if baseline else "none",
        "baseline_evidence_id": baseline.get("evidence_id") if baseline else None,
        "baseline_route_run_id": baseline.get("route_run_id") if baseline else None,
        "baseline_created_at": baseline.get("created_at") if baseline else None,
        "waypoint_id": waypoint_id,
        "action_id": action.id,
    }
    if not gemini.ok or gemini.inspection is None:
        payload["message"] = gemini.message
        return RouteActionResult(
            ok=True,
            state="skipped",
            note=gemini.message,
            payload=payload,
        )

    analysis_path = _write_gemini_analysis(
        run_dir=run_dir,
        route_run_id=route_run_id,
        waypoint_id=waypoint_id,
        action_id=action.id,
        payload={
            "schema_version": 1,
            "analysis": gemini.inspection.model_dump(mode="json"),
            "metadata": payload,
        },
    )
    analysis = gemini.inspection.model_dump(mode="json")
    return RouteActionResult(
        ok=True,
        state="completed",
        note=analysis.get("summary") or "Gemini image inspection completed",
        payload={**payload, "analysis": analysis},
        evidence=[
            {
                "kind": "gemini_vision_analysis",
                "path": str(analysis_path),
                "mime_type": "application/json",
                "metadata": {
                    **payload,
                    "changed": analysis.get("changed"),
                    "summary": analysis.get("summary"),
                    "change_summary": analysis.get("change_summary"),
                    "severity": analysis.get("severity"),
                    "confidence": analysis.get("confidence"),
                    "possible_incident": analysis.get("possible_incident"),
                },
            }
        ],
    )


def _write_gemini_analysis(
    *,
    run_dir: Path,
    route_run_id: str,
    waypoint_id: str,
    action_id: str,
    payload: dict[str, Any],
) -> Path:
    evidence_dir = run_dir / "route_runs" / route_run_id / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    path = evidence_dir / f"{waypoint_id}-{action_id}-gemini.json"
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def _capture_image_path(
    *,
    run_dir: Path,
    route_run_id: str,
    waypoint_id: str,
    action: EditableRouteAction,
    capture_image_handler: CaptureImageHandler | None = None,
) -> tuple[Path, str, str, dict[str, Any]]:
    evidence_dir = run_dir / "route_runs" / route_run_id / "evidence"
    if capture_image_handler is not None:
        captured = capture_image_handler(
            {
                "evidence_dir": evidence_dir,
                "route_run_id": route_run_id,
                "waypoint_id": waypoint_id,
                "action_id": action.id,
                "action": action.model_dump(mode="json"),
            }
        )
        if captured:
            return (
                Path(str(captured["path"])),
                str(captured.get("source") or "go2_camera_live"),
                str(captured.get("mime_type") or "image/png"),
                dict(captured.get("metadata") or {}),
            )
    configured = action.args.get("image_path") or os.environ.get("DOGOPS_GO2_CAMERA_IMAGE_PATH")
    if configured:
        source = Path(str(configured)).expanduser()
        if source.exists() and source.is_file():
            evidence_dir.mkdir(parents=True, exist_ok=True)
            suffix = source.suffix or ".img"
            destination = evidence_dir / f"{waypoint_id}-{action.id}{suffix}"
            shutil.copyfile(source, destination)
            return destination, "go2_camera_configured", _mime_type_for_suffix(suffix), {}
    return (
        _write_placeholder_image(
            run_dir=run_dir,
            route_run_id=route_run_id,
            waypoint_id=waypoint_id,
            action=action,
        ),
        "demo_placeholder",
        "image/png",
        {},
    )


def _mime_type_for_suffix(suffix: str) -> str:
    return {
        ".jpg": "image/jpeg",
        ".jpeg": "image/jpeg",
        ".png": "image/png",
        ".webp": "image/webp",
        ".svg": "image/svg+xml",
    }.get(suffix.lower(), "application/octet-stream")


def _write_placeholder_image(
    *,
    run_dir: Path,
    route_run_id: str,
    waypoint_id: str,
    action: EditableRouteAction,
) -> Path:
    evidence_dir = run_dir / "route_runs" / route_run_id / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    path = evidence_dir / f"{waypoint_id}-{action.id}.png"
    label = f"target={action.args.get('target') or waypoint_id}; action={action.label or action.id}"
    _write_placeholder_png(path, label=label)
    return path


def _write_placeholder_png(path: Path, *, label: str) -> None:
    width = 640
    height = 360
    label_seed = sum(label.encode("utf-8")) % 80
    rows = []
    for y in range(height):
        row = bytearray()
        for x in range(width):
            aisle = abs(x - width // 2) < 58 + y // 8
            stripe = abs(x - (width // 2 - 95 - y // 9)) < 4 or abs(x - (width // 2 + 95 + y // 9)) < 4
            machine = (60 < x < 210 or 430 < x < 585) and 80 < y < 260
            if stripe:
                color = (242, 196, 63)
            elif aisle:
                base = 90 + (y * 70 // height)
                color = (base, base + 5, base + 10)
            elif machine:
                color = (55 + label_seed // 3, 75, 88)
            else:
                color = (27, 39 + label_seed // 8, 52)
            row.extend(color)
        rows.append(b"\x00" + bytes(row))
    raw = b"".join(rows)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"tEXt", b"Description\x00DogOps demo placeholder camera frame")
        + _png_chunk(b"IDAT", zlib.compress(raw, level=9))
        + _png_chunk(b"IEND", b"")
    )


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    checksum = zlib.crc32(kind + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", checksum)
