# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

from pathlib import Path

from dimos.benchmark.vlnce_r2r.external_runtime import VlnceExternalRuntime
from dimos.benchmark.vlnce_r2r.models import VlnceTaskManifest
from dimos.benchmark.vlnce_r2r.preparation import (
    EpisodeBinding,
    PreparationReceipt,
    PreparedAsset,
)


def test_container_exposes_only_public_uds_and_private_result_mounts(tmp_path: Path) -> None:
    case = VlnceTaskManifest.model_validate_json(
        (Path(__file__).parent / "cases/mp3d-example-episode-515/task.json").read_bytes()
    )
    source = case.source
    root = tmp_path / "asset"
    root.mkdir()
    asset = PreparedAsset("shared", root, "a" * 64, {}, True)
    receipt = PreparationReceipt(
        assets={source.episode_asset_id: asset, source.scene_asset_id: asset},
        episode=EpisodeBinding({}, source.episode_sha256),
    )
    runtime = VlnceExternalRuntime(
        case=case,
        attempt_id="attempt-1",
        attempt_path=tmp_path / "attempt",
        preparation=receipt,
        image_id="sha256:image",
        render="native",
    )

    command = runtime._container_command([], [])
    volumes = [command[index + 1] for index, value in enumerate(command) if value == "--volume"]
    assert f"{root}:/dataset:ro" in volumes
    assert f"{runtime.private_dir}:/private:ro" in volumes
    assert f"{runtime.public_dir}:/public:rw" in volumes
    assert f"{runtime.result_dir}:/result:rw" in volumes
    assert f"{runtime.render_staging_dir}:/render:rw" in volumes
    assert "--expected-identity" not in command
    assert runtime.public_dir != runtime.result_dir
