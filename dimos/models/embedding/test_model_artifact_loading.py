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

from dimos.models.embedding.mobileclip import MobileCLIPModel
from dimos.models.embedding.treid import TorchReIDModel
from dimos.utils.model_artifacts import MOBILECLIP2_S4, OSNET_ARTIFACTS


def test_mobileclip_loads_pinned_artifact(mocker, tmp_path: Path) -> None:
    cached_model = tmp_path / "mobileclip2_s4.pt"
    resolver = mocker.patch(
        "dimos.models.embedding.mobileclip.resolve_model_artifact",
        return_value=cached_model,
    )
    open_clip_loader = mocker.patch(
        "dimos.models.embedding.mobileclip.open_clip.create_model_and_transforms",
        return_value=(mocker.MagicMock(), None, mocker.sentinel.preprocess),
    )
    model = MobileCLIPModel(device="cpu")

    _, preprocess = model._model_and_preprocess

    resolver.assert_called_once_with(MOBILECLIP2_S4, None)
    open_clip_loader.assert_called_once_with(
        "MobileCLIP2-S4",
        pretrained=str(cached_model),
    )
    assert preprocess is mocker.sentinel.preprocess


def test_torchreid_loads_selected_pinned_artifact(mocker, tmp_path: Path) -> None:
    cached_model = tmp_path / "osnet_x0_5.pth"
    resolver = mocker.patch(
        "dimos.models.embedding.treid.resolve_model_family_artifact",
        return_value=cached_model,
    )
    extractor = mocker.patch("dimos.models.embedding.treid.torchreid_utils.FeatureExtractor")
    model = TorchReIDModel(model_name="osnet_x0_5", device="cpu")

    assert model._model is extractor.return_value

    resolver.assert_called_once_with(OSNET_ARTIFACTS, "osnet_x0_5.pth", None)
    extractor.assert_called_once_with(
        model_name="osnet_x0_5",
        model_path=str(cached_model),
        device="cpu",
    )
