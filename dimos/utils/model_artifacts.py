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

"""Pinned public model artifacts from their canonical Hugging Face providers."""

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
import hashlib
from pathlib import Path
import shutil

from huggingface_hub import hf_hub_download


@dataclass(frozen=True, slots=True)
class ModelArtifact:
    """An immutable model file published by its canonical provider."""

    repo_id: str
    filename: str
    revision: str
    sha256: str
    license_id: str
    local_filename: str | None = None

    @property
    def local_name(self) -> str:
        """Return the filename expected inside a local override directory."""
        return self.local_filename or Path(self.filename).name


EDGETAM = ModelArtifact(
    repo_id="facebook/EdgeTAM",
    filename="edgetam.pt",
    revision="14d7ecc48c656b94e5184519f698cd5386c5a2bf",
    sha256="ed2d4850b8792c239689b043c47046ec239b6e808a3d9b6ae676c803fd8780df",
    license_id="Apache-2.0",
)

MOBILECLIP2_S4 = ModelArtifact(
    repo_id="apple/MobileCLIP2-S4",
    filename="mobileclip2_s4.pt",
    revision="6ded45853e73bf3d0d5ccd245c3493cdd694e015",
    sha256="129cb58989867e2e648a0e3ac2642e08e7d73cccaabe55564a4b2d775215e49e",
    license_id="Apple sample code license",
    local_filename="MobileCLIP2-S4.pt",
)

OSNET_ARTIFACTS: dict[str, ModelArtifact] = {
    "osnet_x0_25.pth": ModelArtifact(
        repo_id="kaiyangzhou/osnet",
        filename="osnet_x0_25_imagenet.pth",
        revision="a5c5cc037c24235cda3b21085b93ad77c9616224",
        sha256="f54941a66bad4ddd07f2907f498c810ce639ce7a1abeaf2a151f8da118d84693",
        license_id="MIT",
        local_filename="osnet_x0_25.pth",
    ),
    "osnet_x0_5.pth": ModelArtifact(
        repo_id="kaiyangzhou/osnet",
        filename="osnet_x0_5_imagenet.pth",
        revision="a5c5cc037c24235cda3b21085b93ad77c9616224",
        sha256="f10ea9e26ee55b1a65cee3c6e88aa1ae105fe981af3e37692fa68a72fc64bf8e",
        license_id="MIT",
        local_filename="osnet_x0_5.pth",
    ),
    "osnet_x0_75.pth": ModelArtifact(
        repo_id="kaiyangzhou/osnet",
        filename="osnet_x0_75_imagenet.pth",
        revision="a5c5cc037c24235cda3b21085b93ad77c9616224",
        sha256="37add23d2ebc7a22584203d6c22976fdda0f7f29c7b7d79257ac22ae20aec809",
        license_id="MIT",
        local_filename="osnet_x0_75.pth",
    ),
    "osnet_x1_0.pth": ModelArtifact(
        repo_id="kaiyangzhou/osnet",
        filename="osnet_x1_0_imagenet.pth",
        revision="a5c5cc037c24235cda3b21085b93ad77c9616224",
        sha256="fe2d63f9157c28a4a8d8ca29bec12d5b2988ac0346d712025789ea9174968e79",
        license_id="MIT",
        local_filename="osnet_x1_0.pth",
    ),
}

YOLO11_ARTIFACTS: dict[str, ModelArtifact] = {
    "yolo11n.pt": ModelArtifact(
        repo_id="Ultralytics/YOLO11",
        filename="yolo11n.pt",
        revision="8b8ac7d1fae7468f85dbf89670dd66f41f485aab",
        sha256="0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1",
        license_id="AGPL-3.0",
    ),
    "yolo11n-pose.pt": ModelArtifact(
        repo_id="Ultralytics/YOLO11",
        filename="yolo11n-pose.pt",
        revision="8b8ac7d1fae7468f85dbf89670dd66f41f485aab",
        sha256="869e83fcdffdc7371fa4e34cd8e51c838cc729571d1635e5141e3075e9319dc0",
        license_id="AGPL-3.0",
    ),
    "yolo11s-pose.pt": ModelArtifact(
        repo_id="Ultralytics/YOLO11",
        filename="yolo11s-pose.pt",
        revision="8b8ac7d1fae7468f85dbf89670dd66f41f485aab",
        sha256="1060bda4a27012060eca246f9b2adeea22eabb045a1e58f8d229be29b7ebc2ba",
        license_id="AGPL-3.0",
    ),
    "yolo11m-pose.pt": ModelArtifact(
        repo_id="Ultralytics/YOLO11",
        filename="yolo11m-pose.pt",
        revision="8b8ac7d1fae7468f85dbf89670dd66f41f485aab",
        sha256="29b17eaf3a3117cbea906090dbedf9159f7c6a49db58ec8b99ed2dfde1cf6eb2",
        license_id="AGPL-3.0",
    ),
}

GRASPGEN_ARTIFACTS: dict[str, ModelArtifact] = {
    "checkpoints/graspgen_franka_panda.yml": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_franka_panda.yml",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="3b666d28ffb91001ddb6ba24a2e0c11458478a986b808b493cf6fa9a987c2abd",
        license_id="NVIDIA Source Code License",
    ),
    "checkpoints/graspgen_franka_panda_dis.pth": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_franka_panda_dis.pth",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="e47d703c63b54c2d11fbc1effd43898f251b4147250888541e3b16e9c0d19e1c",
        license_id="NVIDIA Source Code License",
    ),
    "checkpoints/graspgen_franka_panda_gen.pth": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_franka_panda_gen.pth",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="0597583b89b322d42ceb4e596967d6ed68d1b56cba4039895909ccd5bdc66eff",
        license_id="NVIDIA Source Code License",
    ),
    "checkpoints/graspgen_robotiq_2f_140.yml": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_robotiq_2f_140.yml",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="93804f423ec02def08add164d83379212204ddc33779cd273ffffdacf94f67f3",
        license_id="NVIDIA Source Code License",
    ),
    "checkpoints/graspgen_robotiq_2f_140_dis.pth": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_robotiq_2f_140_dis.pth",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="52f68d3cd3decb71499e084fdb88808d174419a72cd9ef38f821083a4409895d",
        license_id="NVIDIA Source Code License",
    ),
    "checkpoints/graspgen_robotiq_2f_140_gen.pth": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_robotiq_2f_140_gen.pth",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="fe8497108e39d8fc50be06cd7df22a2f680e0495e713d24c3104616f291e00dd",
        license_id="NVIDIA Source Code License",
    ),
    "checkpoints/graspgen_single_suction_cup_30mm.yml": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_single_suction_cup_30mm.yml",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="9b47604119188f6de12ba1d30bdb2c5b26af8c6a2de69356861c0b916132859f",
        license_id="NVIDIA Source Code License",
    ),
    "checkpoints/graspgen_single_suction_cup_30mm_dis.pth": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_single_suction_cup_30mm_dis.pth",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="552f270447ef50d964ae9315ab4da9f6342f0b9eef8f6c0a28e646ac5b1f0ba5",
        license_id="NVIDIA Source Code License",
    ),
    "checkpoints/graspgen_single_suction_cup_30mm_gen.pth": ModelArtifact(
        repo_id="adithyamurali/GraspGenModels",
        filename="checkpoints/graspgen_single_suction_cup_30mm_gen.pth",
        revision="ec1ccbb5eec0680db669246ac312a3636f16ee43",
        sha256="e7ce37894bd4473545088bd1522a2b6917d220ebec45f47a156ffb38e3c4cade",
        license_id="NVIDIA Source Code License",
    ),
}


def resolve_model_artifact(
    artifact: ModelArtifact,
    local_path: str | Path | None = None,
) -> Path:
    """Resolve an exact model artifact from a local override or the Hub cache.

    A local override may point directly to a file or to a directory containing
    the artifact's historical local filename.
    """
    if local_path is not None:
        candidate = Path(local_path).expanduser()
        if candidate.is_dir():
            candidate = candidate / artifact.local_name
        if not candidate.is_file():
            raise FileNotFoundError(f"Local model artifact does not exist: {candidate}")
        return candidate

    return Path(
        hf_hub_download(
            repo_id=artifact.repo_id,
            filename=artifact.filename,
            revision=artifact.revision,
            token=False,
        )
    )


def resolve_model_family_artifact(
    artifacts: Mapping[str, ModelArtifact],
    model_name: str,
    local_path: str | Path | None = None,
) -> Path:
    """Resolve a named family member, allowing custom models only via a local override."""
    if local_path is not None:
        candidate = Path(local_path).expanduser()
        if candidate.is_dir():
            candidate = candidate / model_name
        if not candidate.is_file():
            raise FileNotFoundError(f"Local model artifact does not exist: {candidate}")
        return candidate

    try:
        artifact = artifacts[model_name]
    except KeyError as exc:
        supported = ", ".join(sorted(artifacts))
        raise ValueError(
            f"Model {model_name!r} has no pinned canonical artifact; "
            f"supported models: {supported}. Pass model_path to use a local model."
        ) from exc
    return resolve_model_artifact(artifact)


def materialize_model_artifacts(
    destination: str | Path,
    artifacts: Iterable[ModelArtifact],
) -> None:
    """Download, verify, and copy pinned artifacts beneath ``destination``."""
    destination_path = Path(destination)
    for artifact in artifacts:
        source = resolve_model_artifact(artifact)
        target = destination_path / artifact.filename
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        verify_model_artifact(target, artifact)


def verify_model_artifact(path: str | Path, artifact: ModelArtifact) -> None:
    """Raise ``ValueError`` if a local artifact does not match its pinned hash."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as artifact_file:
        for chunk in iter(lambda: artifact_file.read(1024 * 1024), b""):
            digest.update(chunk)
    actual = digest.hexdigest()
    if actual != artifact.sha256:
        raise ValueError(
            f"Model artifact checksum mismatch for {artifact.repo_id}/{artifact.filename}: "
            f"expected {artifact.sha256}, got {actual}"
        )
