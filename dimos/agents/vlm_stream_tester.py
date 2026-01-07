# Copyright 2025-2026 Dimensional Inc.
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

import threading
import time

from langchain_core.messages import AIMessage, HumanMessage

from dimos.core import Module, rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class VlmStreamTester(Module):
    """Smoke-test VLMAgent with replayed images and stream queries."""

    color_image: In[Image]
    query_stream: Out[HumanMessage]
    answer_stream: In[AIMessage]

    rpc_calls: list[str] = [
        "VLMAgent.query_image",
    ]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        prompt: str = "What do you see?",
        num_queries: int = 10,
        query_interval_s: float = 2.0,
        max_image_age_s: float = 1.5,
        max_image_gap_s: float = 1.5,
    ) -> None:
        super().__init__()
        self._prompt = prompt
        self._num_queries = num_queries
        self._query_interval_s = query_interval_s
        self._max_image_age_s = max_image_age_s
        self._max_image_gap_s = max_image_gap_s
        self._latest_image: Image | None = None
        self._latest_image_wall_ts: float | None = None
        self._last_image_wall_ts: float | None = None
        self._max_gap_seen_s = 0.0
        self._answer_count = 0
        self._stop_event = threading.Event()
        self._worker: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.color_image.subscribe(self._on_image))  # type: ignore[arg-type]
        self._disposables.add(self.answer_stream.subscribe(self._on_answer))  # type: ignore[arg-type]
        self._worker = threading.Thread(target=self._run_queries, daemon=True)
        self._worker.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        super().stop()

    def _on_image(self, image: Image) -> None:
        now = time.time()
        if self._last_image_wall_ts is not None:
            gap = now - self._last_image_wall_ts
            if gap > self._max_gap_seen_s:
                self._max_gap_seen_s = gap
        self._last_image_wall_ts = now
        self._latest_image_wall_ts = now
        self._latest_image = image

    def _on_answer(self, msg: AIMessage) -> None:
        self._answer_count += 1
        logger.info(
            "VLMAgent stream answer",
            count=self._answer_count,
            content=msg.content,
        )

    def _run_queries(self) -> None:
        try:
            while not self._stop_event.is_set() and self._latest_image is None:
                time.sleep(0.05)

            self._run_stream_queries()
            self._run_rpc_queries()
        except Exception as exc:
            logger.exception("VlmStreamTester query loop failed", error=str(exc))
        finally:
            if self._max_gap_seen_s > self._max_image_gap_s:
                logger.warning(
                    "Image stream gap exceeded threshold",
                    max_gap_s=self._max_gap_seen_s,
                    threshold_s=self._max_image_gap_s,
                )

    def _run_stream_queries(self) -> None:
        for idx in range(self._num_queries):
            if self._stop_event.is_set():
                break
            if self._latest_image is None:
                logger.warning("No image available for stream query.")
                break

            image_age = None
            if self._latest_image_wall_ts is not None:
                image_age = time.time() - self._latest_image_wall_ts
                if image_age > self._max_image_age_s:
                    logger.warning(
                        "Latest image is stale",
                        age_s=image_age,
                        max_age_s=self._max_image_age_s,
                    )

            logger.info("Sending stream query", index=idx + 1, total=self._num_queries)
            self.query_stream.publish(
                HumanMessage(content=f"{self._prompt} (stream query {idx + 1}/{self._num_queries})")
            )
            time.sleep(self._query_interval_s)

    def _run_rpc_queries(self) -> None:
        rpc_query = None
        try:
            rpc_query = self.get_rpc_calls("VLMAgent.query_image")
        except Exception as exc:
            logger.warning("RPC query_image lookup failed", error=str(exc))
            return

        for idx in range(self._num_queries):
            if self._stop_event.is_set():
                break
            if self._latest_image is None:
                logger.warning("No image available for RPC query.")
                break

            image_age = None
            if self._latest_image_wall_ts is not None:
                image_age = time.time() - self._latest_image_wall_ts
                if image_age > self._max_image_age_s:
                    logger.warning(
                        "Latest image is stale",
                        age_s=image_age,
                        max_age_s=self._max_image_age_s,
                    )

            logger.info("Sending RPC query", index=idx + 1, total=self._num_queries)
            try:
                response = rpc_query(
                    self._latest_image,
                    f"{self._prompt} (rpc query {idx + 1}/{self._num_queries})",
                )
                logger.info(
                    "VLMAgent RPC answer",
                    query_index=idx + 1,
                    image_age_s=image_age,
                    content=response,
                )
            except Exception as exc:
                logger.warning("RPC query_image failed", error=str(exc))
            time.sleep(self._query_interval_s)


vlm_stream_tester = VlmStreamTester.blueprint

__all__ = ["VlmStreamTester", "vlm_stream_tester"]
