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

"""Build the raw_april_tags + april_tags streams into a recording's mem2.db and
record the outcome in summary.json. --summary recomputes only the result section.

Usage:
  python dimos/navigation/jnav/components/loop_closure/gsc_pgo/add_april.py --rec PATH
      [--summary] [--output PATH] [--camera color_image] [--intrinsics PATH]
      [--tag-size 0.10] [--dict DICT_APRILTAG_36h11] [--dynamic 17]
"""

import argparse
import json
from pathlib import Path
from typing import Any

from dimos.navigation.jnav.utils import recording_db as rdb
from dimos.navigation.jnav.utils.apriltag_agreement import (
    VISIT_GAP_S,
    split_visits,
)
from dimos.navigation.jnav.utils.apriltags import (
    ensure_april_streams,
    gate_params,
    load_intrinsics_json,
)

RAW_STREAM = "raw_april_tags"
FILTERED_STREAM = "april_tags"
SUMMARY_NAME = "summary.json"
MIN_REVISITS = 2  # a tag seen on fewer visits than this carries no agreement signal
DEFAULT_MARKER_LENGTH_M = 0.10
DEFAULT_DICTIONARY = "DICT_APRILTAG_36h11"


def _parse_dynamic(raw: str | None) -> list[int]:
    if not raw:
        return []
    return sorted({int(token) for token in raw.replace(",", " ").split()})


def _times_by_tag(store: Any, stream_name: str) -> dict[int, list[float]]:
    by_tag: dict[int, list[float]] = {}
    for observation in store.stream(stream_name):
        by_tag.setdefault(int(observation.tags["marker_id"]), []).append(float(observation.ts))
    return by_tag


def summarize(store: Any) -> dict[str, Any]:
    """Per-tag raw detections + filtered visits from the existing streams, flagging
    never-revisited tags. Prints the table and returns the result for summary.json."""
    streams = set(store.list_streams())
    raw_available = RAW_STREAM in streams
    raw = _times_by_tag(store, RAW_STREAM) if raw_available else {}
    filtered = _times_by_tag(store, FILTERED_STREAM) if FILTERED_STREAM in streams else {}
    tag_ids = sorted(set(raw) | set(filtered))

    print(
        f"  {'tag':>5} {'raw':>6} {'filtered':>9} {'revisits':>9}   (visit gap {VISIT_GAP_S:.0f}s)"
    )
    tags: list[dict[str, Any]] = []
    not_revisited: list[int] = []
    for tag_id in tag_ids:
        raw_count = len(raw.get(tag_id, []))
        filtered_times = sorted(filtered.get(tag_id, []))
        visits = len(split_visits(filtered_times, gap_s=VISIT_GAP_S)) if filtered_times else 0
        revisited = visits >= MIN_REVISITS
        if not revisited:
            not_revisited.append(tag_id)
        tags.append(
            {
                "tag_id": tag_id,
                "raw": raw_count if raw_available else None,
                "filtered": len(filtered_times),
                "revisits": visits,
                "revisited": revisited,
            }
        )
        raw_display = raw_count if raw_available else "-"
        flag = "  <-- NOT revisited" if not revisited else ""
        print(f"  {tag_id:>5} {raw_display!s:>6} {len(filtered_times):>9} {visits:>9}{flag}")

    print(
        f"  totals: {len(tag_ids)} tags | {len(tag_ids) - len(not_revisited)} revisited"
        f" (>={MIN_REVISITS} visits) | {len(not_revisited)} not revisited"
    )
    if not_revisited:
        print(f"  NOT revisited: {not_revisited}")
    if not raw_available:
        print(f"  (no '{RAW_STREAM}' yet — raw counts N/A; run without --summary to build it)")

    return {
        "visit_gap_s": VISIT_GAP_S,
        "min_revisits": MIN_REVISITS,
        "all_unfiltered_tag_ids": sorted(raw) if raw_available else sorted(filtered),
        "total_tags": len(tag_ids),
        "revisited": len(tag_ids) - len(not_revisited),
        "not_revisited": not_revisited,
        "raw_available": raw_available,
        "tags": tags,
    }


def _update_summary_json(
    path: Path,
    *,
    filter_parameters: dict[str, Any] | None = None,
    result: dict[str, Any] | None = None,
) -> Path:
    """Merge the april_tags section into summary.json, preserving every other key."""
    data: dict[str, Any] = json.loads(path.read_text()) if path.exists() else {}
    section: dict[str, Any] = data.get("april_tags", {})
    if filter_parameters is not None:
        section["filter_parameters"] = filter_parameters
    if result is not None:
        section["result"] = result
    data["april_tags"] = section
    path.write_text(json.dumps(data, indent=2))
    return path


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--rec", type=Path, required=True, help="recording dir or mem2.db path")
    parser.add_argument("--camera", default="color_image")
    parser.add_argument("--intrinsics", type=Path, default=None)
    parser.add_argument("--tag-size", type=float, default=None, help="marker length (m)")
    parser.add_argument("--dict", dest="dictionary", default=None)
    parser.add_argument(
        "--summary",
        action="store_true",
        help="read-only on streams: recompute and write ONLY april_tags.result (a subset of"
        " the full run, which also rebuilds streams + filter_parameters)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="summary.json path to write (default: <recording>/summary.json) — pick another"
        " location to avoid overwriting an existing one",
    )
    parser.add_argument(
        "--dynamic",
        default=None,
        help="comma/space-separated tag ids on moving objects — kept in raw, dropped from filtered",
    )
    args = parser.parse_args()

    recording = args.rec.expanduser()
    db_path = recording if recording.name == "mem2.db" else recording / "mem2.db"
    if not db_path.exists():
        parser.error(f"no mem2.db at {db_path}")
    store = rdb.store(db_path)
    summary_path = args.output.expanduser() if args.output else (db_path.parent / SUMMARY_NAME)
    print(f"=== {db_path.parent.name} ===")

    if args.summary:
        result = summarize(store)
        _update_summary_json(summary_path, result=result)
        print(f"   updated {summary_path} april_tags.result")
        return

    # Rebuild both streams and overwrite filter_parameters + result.
    dynamic_tags = _parse_dynamic(args.dynamic)
    intrinsics_path = (args.intrinsics or (db_path.parent / "camera_intrinsics.json")).expanduser()
    config = load_intrinsics_json(intrinsics_path)
    marker_length = (
        args.tag_size
        if args.tag_size is not None
        else config.get("marker_length", DEFAULT_MARKER_LENGTH_M)
    )
    dictionary = args.dictionary or config.get("dictionary", DEFAULT_DICTIONARY)
    ensure_april_streams(
        store,
        config["intrinsics"],
        config["distortion"],
        image_stream=args.camera,
        marker_length=marker_length,
        dictionary=dictionary,
        raw_stream=RAW_STREAM,
        filtered_stream=FILTERED_STREAM,
        exclude_tags=dynamic_tags,
        force=True,
    )
    filter_parameters = {
        "gates": gate_params(),
        "marker_length_m": marker_length,
        "dictionary": dictionary,
        "camera_stream": args.camera,
        "raw_stream": RAW_STREAM,
        "filtered_stream": FILTERED_STREAM,
        "dynamic_tags_excluded": dynamic_tags,
    }
    result = summarize(store)
    _update_summary_json(summary_path, filter_parameters=filter_parameters, result=result)
    print(
        f"   updated {summary_path} april_tags (filter_parameters + result); dynamic={dynamic_tags}"
    )


if __name__ == "__main__":
    main()
