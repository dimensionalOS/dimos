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

import json
from pathlib import Path
import subprocess

from dimos.benchmark.vlnce_r2r.external_runtime import (
    VlnceExternalRuntime,
    _cdi_arguments,
    _remove_missing_cdi_devices,
)
from dimos.benchmark.vlnce_r2r.models import VlnceTaskManifest
from dimos.benchmark.vlnce_r2r.preparation import (
    EpisodeBinding,
    PreparationReceipt,
    PreparedAsset,
)


def test_cdi_options_are_split_between_global_and_run_parsers() -> None:
    global_arguments, run_arguments = _cdi_arguments(Path("/attempt/cdi"))

    assert global_arguments == ["--cdi-spec-dir", "/attempt/cdi"]
    assert run_arguments == ["--device", "nvidia.com/gpu=all"]


def test_attempt_cdi_omits_devices_missing_from_host(tmp_path: Path) -> None:
    present = tmp_path / "nvidia0"
    present.touch()
    specification = tmp_path / "nvidia.json"
    specification.write_text(
        json.dumps(
            {
                "devices": [
                    {
                        "name": "all",
                        "containerEdits": {
                            "deviceNodes": [
                                {"path": str(present)},
                                {"path": "/dev/dri/card-does-not-exist"},
                            ],
                            "hooks": [
                                {
                                    "args": [
                                        "nvidia-cdi-hook",
                                        "create-symlinks",
                                        "--link",
                                        "../card-does-not-exist::/dev/dri/by-path/card",
                                    ]
                                }
                            ],
                        },
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    _remove_missing_cdi_devices(specification)

    document = json.loads(specification.read_text(encoding="utf-8"))
    edits = document["devices"][0]["containerEdits"]
    assert edits["deviceNodes"] == [{"path": str(present)}]
    assert "--link" not in edits["hooks"][0]["args"]
    assert "../card-does-not-exist::/dev/dri/by-path/card" not in edits["hooks"][0]["args"]


def test_container_mounts_keep_inputs_read_only_and_public_result_paths_separate(
    tmp_path: Path,
) -> None:
    case_path = Path(__file__).parent / "cases/mp3d-example-episode-515/task.json"
    case = VlnceTaskManifest.model_validate_json(case_path.read_bytes())
    source = case.source
    episode_root = tmp_path / "episodes"
    scene_root = tmp_path / "scenes"
    episode_root.mkdir()
    scene_root.mkdir()
    receipt = PreparationReceipt(
        assets={
            source.episode_asset_id: PreparedAsset(
                asset_id=source.episode_asset_id,
                root=episode_root,
                archive_sha256="a" * 64,
                required_file_sha256={},
                cache_hit=True,
            ),
            source.scene_asset_id: PreparedAsset(
                asset_id=source.scene_asset_id,
                root=scene_root,
                archive_sha256="b" * 64,
                required_file_sha256={},
                cache_hit=True,
            ),
        },
        episode=EpisodeBinding(episode={}, episode_sha256=source.episode_sha256),
    )
    runtime = VlnceExternalRuntime(
        case=case,
        attempt_id="attempt-test",
        attempt_path=tmp_path / "attempt",
        preparation=receipt,
        image_id=f"sha256:{source.preparation.image.image_digest}",
    )

    command = runtime._container_command([], [])
    volumes = [command[index + 1] for index, value in enumerate(command) if value == "--volume"]

    assert f"{scene_root}:/assets/mp3d:ro" in volumes
    assert f"{episode_root}:/dataset:ro" in volumes
    assert f"{runtime.private_dir}:/private:ro" in volumes
    assert f"{runtime.public_dir}:/public:rw" in volumes
    assert f"{runtime.result_dir}:/result:rw" in volumes
    assert not any("/render:rw" in volume for volume in volumes)
    assert runtime.public_dir != runtime.result_dir
    assert len(str(runtime.socket_path).encode()) < 100
    assert not any("credential" in argument.lower() for argument in command)

    rendered = VlnceExternalRuntime(
        case=case,
        attempt_id="attempt-rendered",
        attempt_path=tmp_path / "rendered-attempt",
        preparation=receipt,
        image_id=f"sha256:{source.preparation.image.image_digest}",
        render="native",
    )
    rendered_command = rendered._container_command([], [])
    rendered_volumes = [
        rendered_command[index + 1]
        for index, value in enumerate(rendered_command)
        if value == "--volume"
    ]

    assert f"{rendered.render_staging_dir}:/render:rw" in rendered_volumes
    assert rendered_command[-4:] == [
        "--render-output",
        "/render/native-render.mp4",
        "--render-metadata",
        "/render/native-render.v1.json",
    ]


def test_image_recipe_copies_runtime_code_but_no_cases_credentials_or_assets() -> None:
    recipe = Path(__file__).with_name("Containerfile").read_text()

    assert "COPY container/runtime/ /runtime/" in recipe
    assert "COPY cases" not in recipe
    assert "COPY upstream-manifest" not in recipe
    assert "OPENAI_API_KEY" not in recipe
    assert "auth.json" not in recipe


def test_close_kills_a_container_that_ignores_bounded_termination(tmp_path: Path) -> None:
    case_path = Path(__file__).parent / "cases/mp3d-example-episode-515/task.json"
    case = VlnceTaskManifest.model_validate_json(case_path.read_bytes())
    source = case.source
    asset_root = tmp_path / "asset"
    asset_root.mkdir()
    prepared = PreparedAsset(
        asset_id="shared",
        root=asset_root,
        archive_sha256="a" * 64,
        required_file_sha256={},
        cache_hit=True,
    )
    receipt = PreparationReceipt(
        assets={source.episode_asset_id: prepared, source.scene_asset_id: prepared},
        episode=EpisodeBinding(episode={}, episode_sha256=source.episode_sha256),
    )
    runtime = VlnceExternalRuntime(
        case=case,
        attempt_id="attempt-stubborn",
        attempt_path=tmp_path / "attempt",
        preparation=receipt,
        image_id=f"sha256:{source.preparation.image.image_digest}",
    )

    class StubbornProcess:
        def __init__(self) -> None:
            self.terminated = False
            self.killed = False
            self.waits = 0

        def poll(self) -> None:
            return None

        def terminate(self) -> None:
            self.terminated = True

        def wait(self, timeout: float) -> int:
            self.waits += 1
            if self.waits == 1:
                raise subprocess.TimeoutExpired("podman", timeout)
            return 137

        def kill(self) -> None:
            self.killed = True

    process = StubbornProcess()
    runtime._process = process  # type: ignore[assignment]

    runtime.close()

    assert process.terminated is True
    assert process.killed is True
    assert process.waits == 2
