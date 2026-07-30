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

from pathlib import Path

import pytest

from dimos.utils.model_artifacts import (
    MOBILECLIP2_S4,
    ModelArtifact,
    materialize_model_artifacts,
    resolve_model_artifact,
    resolve_model_family_artifact,
    verify_model_artifact,
)


def test_resolve_model_artifact_downloads_anonymously_from_pinned_revision(
    mocker,
    tmp_path: Path,
) -> None:
    cached = tmp_path / "cached.pt"
    download = mocker.patch(
        "dimos.utils.model_artifacts.hf_hub_download",
        return_value=str(cached),
    )

    result = resolve_model_artifact(MOBILECLIP2_S4)

    assert result == cached
    download.assert_called_once_with(
        repo_id="apple/MobileCLIP2-S4",
        filename="mobileclip2_s4.pt",
        revision="6ded45853e73bf3d0d5ccd245c3493cdd694e015",
        token=False,
    )


def test_resolve_model_artifact_accepts_local_file(tmp_path: Path) -> None:
    local_model = tmp_path / "custom.pt"
    local_model.write_bytes(b"model")

    result = resolve_model_artifact(MOBILECLIP2_S4, local_model)

    assert result == local_model


def test_resolve_model_artifact_uses_historical_name_in_local_directory(tmp_path: Path) -> None:
    local_model = tmp_path / "MobileCLIP2-S4.pt"
    local_model.write_bytes(b"model")

    result = resolve_model_artifact(MOBILECLIP2_S4, tmp_path)

    assert result == local_model


def test_resolve_model_artifact_rejects_missing_local_override(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="Local model artifact does not exist"):
        resolve_model_artifact(MOBILECLIP2_S4, tmp_path)


def test_resolve_model_family_artifact_accepts_custom_local_model(tmp_path: Path) -> None:
    local_model = tmp_path / "custom.pt"
    local_model.write_bytes(b"model")

    result = resolve_model_family_artifact({}, "custom.pt", tmp_path)

    assert result == local_model


def test_resolve_model_family_artifact_rejects_unpinned_remote_model() -> None:
    with pytest.raises(ValueError, match="no pinned canonical artifact"):
        resolve_model_family_artifact({}, "custom.pt")


def test_materialize_model_artifacts_preserves_repository_path(
    mocker,
    tmp_path: Path,
) -> None:
    source = tmp_path / "cache" / "model.pth"
    source.parent.mkdir()
    source.write_bytes(b"model")
    artifact = ModelArtifact(
        repo_id="canonical/model",
        filename="checkpoints/model.pth",
        revision="0123456789abcdef0123456789abcdef01234567",
        sha256="9372c470eeadd5ec07579c4b7dc7a19a7335c70c60080f53f6f8e08d81270791",
        license_id="Example",
    )
    resolver = mocker.patch(
        "dimos.utils.model_artifacts.resolve_model_artifact",
        return_value=source,
    )
    verifier = mocker.patch("dimos.utils.model_artifacts.verify_model_artifact")
    destination = tmp_path / "image"

    materialize_model_artifacts(destination, [artifact])

    target = destination / "checkpoints" / "model.pth"
    assert target.read_bytes() == b"model"
    resolver.assert_called_once_with(artifact)
    verifier.assert_called_once_with(target, artifact)


def test_verify_model_artifact_accepts_exact_content(tmp_path: Path) -> None:
    model = tmp_path / "model.pt"
    model.write_bytes(b"exact model")
    artifact = ModelArtifact(
        repo_id="canonical/model",
        filename="model.pt",
        revision="0123456789abcdef0123456789abcdef01234567",
        sha256="b7d1f2dd431219baf3f0db1317463664d238cdedccee53d864612a99318602fc",
        license_id="Example",
    )

    verify_model_artifact(model, artifact)


def test_verify_model_artifact_rejects_changed_content(tmp_path: Path) -> None:
    model = tmp_path / "model.pt"
    model.write_bytes(b"changed")
    artifact = ModelArtifact(
        repo_id="canonical/model",
        filename="model.pt",
        revision="0123456789abcdef0123456789abcdef01234567",
        sha256="16f3a135a5b866cde978edc6a82b7258edba1e2f1787bc4d9ab94806b1718150",
        license_id="Example",
    )

    with pytest.raises(ValueError, match="checksum mismatch"):
        verify_model_artifact(model, artifact)
