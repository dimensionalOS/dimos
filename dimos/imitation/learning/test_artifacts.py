from moto import mock_aws
import pytest

from dimos.imitation.learning.artifacts import Artifacts, seal, verify
from dimos.imitation.learning.protocol import LearningError


def dataset(root):
    for name in ("meta/info.json", "data/chunk-000/data.parquet", "videos/camera/episode.mp4"):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"unit-test artifact " + name.encode())


def test_complete_manifest_and_tamper(tmp_path):
    dataset(tmp_path)
    sealed = seal(tmp_path, {"source_revision": 3})
    assert verify(tmp_path) == sealed
    (tmp_path / "data/chunk-000/data.parquet").write_bytes(b"tampered")
    with pytest.raises(LearningError):
        verify(tmp_path)


def test_missing_video_and_reseal_rejected(tmp_path):
    with pytest.raises(LearningError):
        seal(tmp_path, {})
    dataset(tmp_path)
    seal(tmp_path, {})
    with pytest.raises(LearningError):
        seal(tmp_path, {})


@mock_aws
def test_remote_publish_readback_and_restore(tmp_path, monkeypatch):
    monkeypatch.setenv("AWS_ACCESS_KEY_ID", "testing")
    monkeypatch.setenv("AWS_SECRET_ACCESS_KEY", "testing")
    monkeypatch.setenv("DIMOS_LEARNING_S3_BUCKET", "learning-unit")
    monkeypatch.delenv("DIMOS_LEARNING_S3_ENDPOINT", raising=False)
    root = tmp_path / "source"
    root.mkdir()
    dataset(root)
    seal(root, {"revision": 7})
    artifacts = Artifacts()
    artifacts.client.create_bucket(Bucket=artifacts.bucket)
    publication = artifacts.publish(root, "org", "dataset")
    assert artifacts.verify_publication(publication)["lineage"]["revision"] == 7
    assert artifacts.publish(root, "org", "dataset") == publication
    target = artifacts.restore(publication, tmp_path / "restored")
    assert verify(root) == verify(target)
    artifacts.client.put_object(
        Bucket=artifacts.bucket, Key=publication["prefix"] + "meta/info.json", Body=b"corrupt"
    )
    with pytest.raises(LearningError):
        artifacts.verify_publication(publication)
