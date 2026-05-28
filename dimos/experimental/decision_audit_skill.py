#!/usr/bin/env python3
# Drop-in Guide decision audit skill.
# Claude calls `log_nav_decision` BEFORE every navigate_with_text so we
# capture the grounding evidence (query, tier matched, similarity/confidence,
# chosen target). The trail is written to:
#   1. A JSONL file at assets/output/drop_in_guide/nav_trace.jsonl
#   2. A Rerun TextDocument entity at /audit/nav_decisions  (renders as a
#      panel in the Rerun viewer alongside the 3D world)
#   3. A short Rerun TextLog entity at /audit/event  (timeline-style line)
# And exposed via `recent_nav_decisions()` for in-conversation queries.
#
# This is the thesis carrier: every navigation choice becomes legible.

from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Any

from dimos.agents.annotation import skill
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_OUT_DIR = DIMOS_PROJECT_ROOT / "assets" / "output" / "drop_in_guide"
_TRACE_PATH = _OUT_DIR / "nav_trace.jsonl"

# Rerun integration is intentionally disabled here — calling rr.log() from
# within the dimOS fork-server worker hangs when no recording stream is
# attached (the worker process can't share the bridge's stream). We render
# the audit panel in video post-production from the JSONL trace instead.
# Marker file `nav_trace.jsonl` is the source of truth.


def _fmt_panel(entries: list[dict[str, Any]], max_items: int = 5) -> str:
    """Render the audit panel as a markdown document (used for export, not
    live Rerun rendering)."""
    if not entries:
        return "# Drop-in Guide — Decision Audit\n\n_no decisions yet_"
    lines = ["# Drop-in Guide — Decision Audit", ""]
    now = time.time()
    for e in entries[-max_items:][::-1]:
        age_s = max(0.0, now - float(e["ts"]))
        age = f"{age_s:.0f}s ago" if age_s < 60 else f"{age_s/60:.1f}m ago"
        lines.append(
            f"- **{e['query']}** → `{e['target']}`  "
            f"_(tier: {e['matched_tier']}, conf {e['confidence']:.2f}, {age})_"
        )
    return "\n".join(lines)


class DecisionAuditSkill(Module):
    """Audit trail for nav decisions — the system's grounded-evidence record."""

    _trace: list[dict[str, Any]] = []

    @rpc
    def start(self) -> None:
        super().start()
        os.makedirs(_OUT_DIR, exist_ok=True)
        self._trace = []
        if _TRACE_PATH.exists():
            try:
                _TRACE_PATH.unlink()
            except OSError:
                pass
        logger.info(f"DecisionAuditSkill: trace at {_TRACE_PATH}")

    @rpc
    def stop(self) -> None:
        super().stop()

    def render_panel_markdown(self) -> str:
        """Render the current audit trail as a markdown panel. Useful for
        exporting to video overlays in post-production. Not exposed as an
        LLM skill — call directly when generating demo assets.
        """
        return _fmt_panel(self._trace)

    @skill
    def log_nav_decision(
        self,
        query: str,
        matched_tier: str,
        confidence: float,
        target: str,
    ) -> str:
        """Log a navigation decision for the audit panel. Call this RIGHT
        BEFORE `navigate_with_text` (and BEFORE `speak`) so the audit panel
        renders grounding evidence in real time.

        Args:
            query: The original natural-language target the user asked for
                   (e.g. "the printer", "kitchen").
            matched_tier: How you resolved it. Must be one of:
                          "tagged" (matched a tag_location from priming),
                          "visual" (Qwen-VL detected it in current camera frame),
                          "semantic" (matched the spatial memory vector search),
                          "unresolved" (no match — about to ask operator for help).
            confidence: Your confidence in this match, 0.0 to 1.0. For tagged
                        matches use 0.9+ unless ambiguous. For semantic use
                        the cosine similarity if known, else 0.5.
            target: The canonical name of the destination you're navigating to
                    (e.g. "printer", "kitchen"). May differ from query if you
                    resolved an alias.
        """
        entry = {
            "ts": time.time(),
            "query": query.strip(),
            "matched_tier": matched_tier.strip().lower(),
            "confidence": float(confidence),
            "target": target.strip(),
        }
        self._trace.append(entry)
        try:
            with open(_TRACE_PATH, "a") as f:
                f.write(json.dumps(entry) + "\n")
        except OSError as e:
            logger.warning(f"failed to write nav_trace: {e}")

        return (
            f"Logged: query='{entry['query']}' tier={entry['matched_tier']} "
            f"confidence={entry['confidence']:.2f} target='{entry['target']}'"
        )

    @skill
    def recent_nav_decisions(self) -> str:
        """Recall the most recent navigation decisions you logged. Useful
        when someone asks "why did you go there?" or "show me your reasoning".
        Returns up to 5 most recent log entries.
        """
        if not self._trace:
            return "No navigation decisions logged yet."
        recent = self._trace[-5:]
        lines = []
        for e in recent:
            lines.append(
                f"- '{e['query']}' -> {e['target']} "
                f"(via {e['matched_tier']}, confidence {e['confidence']:.2f})"
            )
        return "Recent decisions:\n" + "\n".join(lines)
