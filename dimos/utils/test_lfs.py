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
"git lfs pull failed". The end-to-end smudge test lives in test_data.py.
"""

import hashlib
import urllib.request

import pytest
import requests

LFS_URL = "https://lfs.dimensionalos.com/dimensionalOS/dimos"

# Known fixture: cafe.jpg.tar.gz pointer in dev. SHA + size match the LFS
# pointer file at data/.lfs/cafe.jpg.tar.gz.
KNOWN_OID = "b8cf30439b41033ccb04b09b9fc8388d18fb544d55b85c155dbf85700b9e7603"
KNOWN_SIZE = 136165


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


@pytest.mark.slow
def test_anonymous_download_returns_presigned_url():
    """An unauthenticated batch download request returns a presigned S3 URL."""
    response = _batch("download", KNOWN_OID, KNOWN_SIZE)
    response.raise_for_status()

    obj = response.json()["objects"][0]
    assert obj["oid"] == KNOWN_OID
    assert "actions" in obj, f"no download action — server response: {obj}"

    href = obj["actions"]["download"]["href"]
    assert href.startswith("https://dimos-github-lfs.s3"), href


@pytest.mark.slow
def test_anonymous_upload_is_forbidden():
    """An unauthenticated upload returns 403 — only repo collaborators can push."""
    response = _batch("upload", "0" * 64, 1)
    assert response.status_code == 403, response.text


@pytest.mark.slow
def test_known_object_roundtrip():
    """Fetching the known fixture via giftless yields bytes whose SHA matches the pointer."""
    response = _batch("download", KNOWN_OID, KNOWN_SIZE)
    response.raise_for_status()
    href = response.json()["objects"][0]["actions"]["download"]["href"]

    with urllib.request.urlopen(href, timeout=30) as r:
        body = r.read()

    assert hashlib.sha256(body).hexdigest() == KNOWN_OID
    assert len(body) == KNOWN_SIZE
