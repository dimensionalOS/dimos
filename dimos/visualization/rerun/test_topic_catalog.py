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

from typing import Any

from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.visualization.rerun.topic_catalog import (
    LcmTopicCatalog,
    classify_lcm_topic_renderability,
    normalized_lcm_topic,
    topic_type_name,
)


class _RenderableMsg:
    msg_name = "test.RenderableMsg"

    def to_rerun(self) -> Any:
        return None


class _PlainMsg:
    msg_name = "test.PlainMsg"


def test_typed_channel_helpers_normalize_name_and_type() -> None:
    topic = Topic("/camera/color", _RenderableMsg)  # type: ignore[arg-type]

    assert str(topic) == "/camera/color#test.RenderableMsg"
    assert normalized_lcm_topic(topic) == "/camera/color"
    assert topic_type_name(topic) == "test.RenderableMsg"


def test_catalog_tracks_liveness_rate_bandwidth_and_selection() -> None:
    catalog = LcmTopicCatalog(freshness_window_s=1.0, stats_window_s=2.0)
    topic = Topic("/camera/color", _RenderableMsg)  # type: ignore[arg-type]

    catalog.observe(topic, b"1234", renderability="renderable", render_reason="native", now=10.0)
    catalog.observe(topic, b"12", renderability="renderable", render_reason="native", now=11.0)

    [entry] = catalog.snapshot(staged_topics={"/camera/color"}, logging_topics=set(), now=11.5)
    assert entry.live is True
    assert entry.selected is True
    assert entry.logging is False
    assert entry.message_count == 2
    assert entry.rate_hz == 1.0
    assert entry.bandwidth_bps == 3.0
    assert entry.average_message_size_bytes == 3.0

    [stale] = catalog.snapshot(now=13.0)
    assert stale.live is False


def test_renderability_classification_reports_converter_status() -> None:
    renderable = Topic("/renderable", _RenderableMsg)  # type: ignore[arg-type]
    plain = Topic("/plain", _PlainMsg)  # type: ignore[arg-type]
    unknown = Topic("/raw", None)

    assert classify_lcm_topic_renderability(renderable, entity_path="world/renderable") == (
        "renderable",
        "native to_rerun()",
    )
    assert classify_lcm_topic_renderability(plain, entity_path="world/plain") == (
        "unsupported",
        "no Rerun converter",
    )
    assert classify_lcm_topic_renderability(unknown, entity_path="world/raw") == (
        "unknown",
        "unknown message type",
    )


def test_visual_override_can_render_or_suppress_topic() -> None:
    plain = Topic("/plain", _PlainMsg)  # type: ignore[arg-type]

    assert classify_lcm_topic_renderability(
        plain,
        entity_path="world/plain",
        visual_override={"world/plain": lambda _: None},
    ) == ("renderable", "visual converter")
    assert classify_lcm_topic_renderability(
        plain,
        entity_path="world/plain",
        visual_override={"world/plain": None},
    ) == ("unsupported", "suppressed by visual override")
