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

"""Synchronous HTTP client for benchmark runtime sidecars."""

from __future__ import annotations

import time

from dimos_runtime_protocol import (
    EpisodeResetRequest,
    EpisodeResetResponse,
    HealthResponse,
    RuntimeDescription,
    ScoreOutput,
    StepRequest,
    StepResponse,
)
import requests


class RuntimeSidecarClient:
    """Minimal request/response client for the v1 runtime sidecar protocol."""

    def __init__(self, base_url: str, *, timeout_s: float = 5.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s

    def health(self) -> HealthResponse:
        response = requests.get(f"{self.base_url}/health", timeout=self.timeout_s)
        _raise_for_status(response)
        return HealthResponse.model_validate(response.json())

    def wait_until_healthy(
        self, *, timeout_s: float = 10.0, poll_s: float = 0.05
    ) -> HealthResponse:
        deadline = time.monotonic() + timeout_s
        last_error: requests.RequestException | ValueError | None = None
        while time.monotonic() < deadline:
            try:
                return self.health()
            except (requests.RequestException, ValueError) as exc:
                last_error = exc
                time.sleep(poll_s)
        raise TimeoutError(f"runtime sidecar did not become healthy: {last_error}")

    def describe(self) -> RuntimeDescription:
        response = requests.get(f"{self.base_url}/describe", timeout=self.timeout_s)
        _raise_for_status(response)
        return RuntimeDescription.model_validate(response.json())

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        response = requests.post(
            f"{self.base_url}/reset",
            json=request.model_dump(mode="json"),
            timeout=self.timeout_s,
        )
        _raise_for_status(response)
        return EpisodeResetResponse.model_validate(response.json())

    def step(self, request: StepRequest) -> StepResponse:
        response = requests.post(
            f"{self.base_url}/step",
            json=request.model_dump(mode="json"),
            timeout=self.timeout_s,
        )
        _raise_for_status(response)
        return StepResponse.model_validate(response.json())

    def score(self) -> ScoreOutput:
        response = requests.get(f"{self.base_url}/score", timeout=self.timeout_s)
        _raise_for_status(response)
        return ScoreOutput.model_validate(response.json())

    def payload(self, data_ref: str) -> bytes:
        path = data_ref if data_ref.startswith("/") else f"/{data_ref}"
        response = requests.get(f"{self.base_url}{path}", timeout=self.timeout_s)
        _raise_for_status(response)
        return response.content


def _raise_for_status(response: requests.Response) -> None:
    try:
        response.raise_for_status()
    except requests.HTTPError as exc:
        body = response.text.strip()
        if body:
            raise requests.HTTPError(f"{exc}; response body: {body}", response=response) from exc
        raise
