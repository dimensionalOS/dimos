"""DimOS glue: Qwen VL via the Alibaba *China* DashScope endpoint.

DimOS's ``QwenVlModel`` hardcodes the international endpoint
(``dashscope-intl.aliyuncs.com``), which returns 401 for a mainland-China Model
Studio key. This subclass points at the China endpoint and defaults to a
current grounding model. Override the model with ``PAWTRACK_VLM_MODEL``.

``qwen-vl-max`` is the default because it returns accurate pixel-space bounding
boxes; the ``qwen3-vl-*`` models return 0-1000 normalized coordinates that the
DimOS bbox parser does not rescale. Imports DimOS; not unit tested.
"""

from __future__ import annotations

import functools
import os

from openai import OpenAI

from dimos.models.vl.qwen import QwenVlModel, QwenVlModelConfig

_CHINA_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
_DEFAULT_MODEL = "qwen-vl-max"


class QwenChinaVlModelConfig(QwenVlModelConfig):
    """Qwen VL config defaulting to a current China-endpoint model."""

    model_name: str = os.getenv("PAWTRACK_VLM_MODEL", _DEFAULT_MODEL)


class QwenChinaVlModel(QwenVlModel):
    """QwenVlModel that talks to the Alibaba China DashScope endpoint."""

    config: QwenChinaVlModelConfig

    @functools.cached_property
    def _client(self) -> OpenAI:
        api_key = self.config.api_key or os.getenv("ALIBABA_API_KEY")
        if not api_key:
            raise ValueError("Alibaba API key must be set in ALIBABA_API_KEY.")
        return OpenAI(base_url=_CHINA_BASE_URL, api_key=api_key)
