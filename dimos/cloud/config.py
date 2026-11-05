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

from pathlib import Path

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from dimos.core.global_config import ENV_FILE


class CloudConfig(BaseSettings):
    """Cloud authentication and transfers; environment prefix DIMOS_CLOUD_."""

    url: str = "https://api.dimensional.org"
    api_key: str | None = Field(default=None, repr=False)
    http_timeout: float = 60.0
    upload_codec: str = "lz4"
    upload_retries: int = 2
    upload_chunk_mb: int | None = None
    upload_quiet_s: float = 30.0
    staging_dir: Path | None = None

    model_config = SettingsConfigDict(
        env_prefix="DIMOS_CLOUD_",
        env_file=ENV_FILE,
        env_file_encoding="utf-8",
        extra="ignore",
    )
