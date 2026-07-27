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

"""Tests for hosted-data authentication primitives."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from dimos.hosted_data.auth import (
    RepositoryAccessPolicy,
    create_signed_download_url,
    verify_download_signature,
)


def test_repository_access_policy_scopes_tokens(tmp_path: Path) -> None:
    token = "developer-secret"
    path = tmp_path / "acl.json"
    path.write_text(
        json.dumps(
            {
                "principals": [
                    {
                        "name": "alice",
                        "token_sha256": hashlib.sha256(token.encode()).hexdigest(),
                        "read": ["shared/*"],
                        "write": ["alice/replays"],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    policy = RepositoryAccessPolicy.from_file(path)

    assert policy.authorize(token, mode="write", owner="alice", repository="replays")
    assert policy.authorize(token, mode="read", owner="alice", repository="replays")
    assert policy.authorize(token, mode="read", owner="shared", repository="go2")
    assert not policy.authorize(token, mode="write", owner="shared", repository="go2")
    assert not policy.authorize("wrong", mode="read", owner="alice", repository="replays")


def test_signed_download_url_is_path_bound_and_expires() -> None:
    url = create_signed_download_url(
        "https://cdn.example/api/v1/repositories/alice/demo/objects/" + "a" * 64,
        secret="signing-secret",
        expires_in_seconds=60,
        now=1000.0,
    )
    parsed = urlsplit(url)
    query = parse_qs(parsed.query)

    assert verify_download_signature(
        parsed.path,
        secret="signing-secret",
        expires=query["expires"][0],
        signature=query["signature"][0],
        now=1059.0,
    )
    assert not verify_download_signature(
        parsed.path + "x",
        secret="signing-secret",
        expires=query["expires"][0],
        signature=query["signature"][0],
        now=1059.0,
    )
    assert not verify_download_signature(
        parsed.path,
        secret="signing-secret",
        expires=query["expires"][0],
        signature=query["signature"][0],
        now=1061.0,
    )
