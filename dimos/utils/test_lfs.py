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

"""Smoke tests for the dimos Git LFS server (lfs.dimensionalos.com).

These talk to the LFS batch API directly (no `git lfs` toolchain), so when CI
fails the diagnostic is "the LFS server returned X" rather than the looser
"git lfs pull failed". The fixture is a tracked documentation asset; runtime
data is served by the pinned Hugging Face dataset instead.
"""

import hashlib
import urllib.request

import pytest
import requests

LFS_URL = "https://lfs.dimensionalos.com/dimensionalOS/dimos"

# Known fixture: tracked documentation SVG pointer.
KNOWN_OID = "566d6c7009d0e3d2e8a6da49bdaa4e865c44a5b6e3dc10082bf1ac200bb9681b"
KNOWN_SIZE = 333223


def _batch(operation: str, oid: str, size: int, *, auth=None):
    return requests.post(
        f"{LFS_URL}/objects/batch",
        json={
            "operation": operation,
            "transfers": ["basic"],
            "objects": [{"oid": oid, "size": size}],
        },
        headers={
            "Accept": "application/vnd.git-lfs+json",
            "Content-Type": "application/vnd.git-lfs+json",
        },
        auth=auth,
        timeout=15,
    )


@pytest.mark.self_hosted
def test_anonymous_download_returns_presigned_url():
    """An unauthenticated batch download request returns a presigned S3 URL."""
    response = _batch("download", KNOWN_OID, KNOWN_SIZE)
    response.raise_for_status()

    obj = response.json()["objects"][0]
    assert obj["oid"] == KNOWN_OID
    assert "actions" in obj, f"no download action — server response: {obj}"

    href = obj["actions"]["download"]["href"]
    assert "dimos-github-lfs" in href, href


@pytest.mark.self_hosted
def test_anonymous_upload_is_forbidden():
    """An unauthenticated upload returns 401/403 — only repo collaborators can push."""
    response = _batch("upload", "0" * 64, 1)
    assert response.status_code in (401, 403), response.text


@pytest.mark.self_hosted
def test_known_object_roundtrip():
    """Fetching the known fixture via giftless yields bytes whose SHA matches the pointer."""
    response = _batch("download", KNOWN_OID, KNOWN_SIZE)
    response.raise_for_status()
    href = response.json()["objects"][0]["actions"]["download"]["href"]

    with urllib.request.urlopen(href, timeout=30) as r:
        body = r.read()

    assert hashlib.sha256(body).hexdigest() == KNOWN_OID
    assert len(body) == KNOWN_SIZE
