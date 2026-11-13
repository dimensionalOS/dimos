"""Immutable, checksum-verified S3-compatible publication."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any

import boto3
from botocore.exceptions import ClientError

from dimos.imitation.learning.media import sha256
from dimos.imitation.learning.protocol import LearningError


def inventory(root: Path) -> list[dict[str, Any]]:
    return [
        {"path": str(p.relative_to(root)), "bytes": p.stat().st_size, "sha256": sha256(p)}
        for p in sorted(root.rglob("*"))
        if p.is_file() and p.name != "manifest.json"
    ]


def seal(root: Path, lineage: dict[str, Any], kind: str = "dataset") -> dict[str, Any]:
    if kind == "dataset":
        for directory in ("meta", "data", "videos"):
            if not any(p.is_file() for p in (root / directory).rglob("*")):
                raise LearningError(
                    422, "incomplete_dataset", f"Dataset requires nonempty {directory}"
                )
        if not (root / "meta/info.json").is_file():
            raise LearningError(422, "incomplete_dataset", "Native dataset metadata missing")
    (root / "lineage.json").write_text(json.dumps(lineage, sort_keys=True, indent=2))
    manifest = {"version": 1, "kind": kind, "files": inventory(root), "lineage": lineage}
    path = root / "manifest.json"
    if path.exists():
        raise LearningError(409, "immutable_output", "Output is already sealed")
    path.write_text(json.dumps(manifest, sort_keys=True, indent=2))
    verify(root)
    return manifest


def verify(root: Path) -> dict[str, Any]:
    manifest = json.loads((root / "manifest.json").read_text())
    expected = {x["path"]: x for x in manifest["files"]}
    actual = {x["path"]: x for x in inventory(root)}
    if expected != actual:
        raise LearningError(
            409, "checksum_mismatch", "Artifact inventory differs from sealed manifest"
        )
    return manifest


class Artifacts:
    def __init__(self):
        self.bucket = os.environ.get("DIMOS_LEARNING_S3_BUCKET", "")
        self.endpoint = os.environ.get("DIMOS_LEARNING_S3_ENDPOINT")
        if not self.bucket:
            raise LearningError(503, "storage_unconfigured", "DIMOS_LEARNING_S3_BUCKET required")
        self.client = boto3.client(
            "s3",
            endpoint_url=self.endpoint,
            region_name=os.environ.get("AWS_DEFAULT_REGION", "us-east-1"),
        )

    def publish(self, root: Path, org_id: str, resource_id: str) -> dict[str, Any]:
        manifest = verify(root)
        prefix = f"learning/{org_id}/{resource_id}/"
        # Manifest is uploaded LAST, after read-back of every object.
        files = [root / entry["path"] for entry in manifest["files"]] + [root / "manifest.json"]
        for path in files:
            key = prefix + str(path.relative_to(root))
            checksum = sha256(path)
            try:
                self.client.put_object(
                    Bucket=self.bucket,
                    Key=key,
                    Body=path.read_bytes(),
                    Metadata={"sha256": checksum},
                    IfNoneMatch="*",
                )
            except ClientError as error:
                if error.response["Error"]["Code"] not in ("PreconditionFailed", "412"):
                    raise
            body = self.client.get_object(Bucket=self.bucket, Key=key)["Body"].read()
            if hashlib.sha256(body).hexdigest() != checksum:
                raise LearningError(
                    409, "storage_checksum", "Remote artifact read-back failed; output not ready"
                )
        return {
            "bucket": self.bucket,
            "prefix": prefix,
            "endpoint": self.endpoint,
            "manifest_sha256": sha256(root / "manifest.json"),
            "storage_verified": True,
            "storage_kind": "s3-compatible-emulator" if self.endpoint else "s3",
        }

    def verify_publication(self, publication: dict[str, Any]) -> dict[str, Any]:
        if publication.get("bucket") != self.bucket or publication.get("endpoint") != self.endpoint:
            raise LearningError(
                409, "publication_target", "Publication uses a different configured object store"
            )
        raw = self.client.get_object(
            Bucket=self.bucket, Key=publication["prefix"] + "manifest.json"
        )["Body"].read()
        if hashlib.sha256(raw).hexdigest() != publication.get("manifest_sha256"):
            raise LearningError(409, "storage_checksum", "Published manifest does not match")
        manifest = json.loads(raw)
        for entry in manifest["files"]:
            body = self.client.get_object(
                Bucket=self.bucket, Key=publication["prefix"] + entry["path"]
            )["Body"].read()
            if len(body) != entry["bytes"] or hashlib.sha256(body).hexdigest() != entry["sha256"]:
                raise LearningError(409, "storage_checksum", "Published object failed read-back")
        return manifest

    def restore(self, publication: dict[str, Any], root: Path) -> Path:
        prefix = publication["prefix"]
        raw = self.client.get_object(Bucket=publication["bucket"], Key=prefix + "manifest.json")[
            "Body"
        ].read()
        if hashlib.sha256(raw).hexdigest() != publication["manifest_sha256"]:
            raise LearningError(409, "storage_checksum", "Manifest checksum mismatch")
        manifest = json.loads(raw)
        root.mkdir(parents=True, exist_ok=True)
        for entry in manifest["files"]:
            path = (root / entry["path"]).resolve()
            if not path.is_relative_to(root.resolve()):
                raise LearningError(422, "unsafe_artifact", "Invalid artifact path")
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(
                self.client.get_object(Bucket=publication["bucket"], Key=prefix + entry["path"])[
                    "Body"
                ].read()
            )
        (root / "manifest.json").write_bytes(raw)
        verify(root)
        return root
