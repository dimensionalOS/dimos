"""Native upstream split parity and opt-in real CPU package verification."""

import json
import os
from types import SimpleNamespace

from dimos_lerobot.learning_train import ACTConfig, feature_metadata, load_check, train
import pytest


@pytest.mark.parametrize(
    "split,expected,held",
    [(0, [0, 1, 2, 3, 4], []), (0.1, [0, 2, 1], [4, 3]), (0.2, [0, 2, 1], [4, 3])],
)
def test_native_per_task_split(monkeypatch, split, expected, held):
    from lerobot.datasets import factory

    class Dataset:
        def __init__(self, *args, episodes=None, **kwargs):
            self.episodes = episodes
            self.num_episodes = 5
            self.meta = SimpleNamespace(episodes={"tasks": [["a"], ["b"], ["a"], ["b"], ["a"]]})

    monkeypatch.setattr(factory, "make_dataset", lambda cfg: Dataset())
    monkeypatch.setattr(factory, "LeRobotDataset", Dataset)
    monkeypatch.setattr(factory, "resolve_delta_timestamps", lambda *args: None)
    cfg = SimpleNamespace(
        dataset=SimpleNamespace(
            eval_split=split,
            repo_id="local/test",
            root="unused",
            revision=None,
            video_backend="pyav",
            use_imagenet_stats=False,
            image_transforms=SimpleNamespace(enable=False),
        ),
        trainable_config=None,
        tolerance_s=0.001,
    )
    training, evaluation = factory.make_train_eval_datasets(cfg)
    assert (training.episodes if training.episodes is not None else list(range(5))) == expected
    assert (evaluation.episodes if evaluation is not None else []) == held


def test_native_empty_train_blocked(monkeypatch):
    from lerobot.datasets import factory

    monkeypatch.setattr(
        factory,
        "make_dataset",
        lambda cfg: SimpleNamespace(
            episodes=None, num_episodes=2, meta=SimpleNamespace(episodes={"tasks": [["a"], ["b"]]})
        ),
    )
    with pytest.raises(ValueError, match="0 training episodes"):
        factory.make_train_eval_datasets(SimpleNamespace(dataset=SimpleNamespace(eval_split=0.1)))


@pytest.mark.parametrize("split", [0, 0.1, 0.2])
def test_real_reference_training(tmp_path, split):
    path = os.environ.get("DIMOS_LEARNING_TEST_DATASET")
    if not path:
        pytest.skip("Set DIMOS_LEARNING_TEST_DATASET to converted real reference")
    output = tmp_path / "policy"
    result = train(
        {"dataset_path": path, "output": str(output), "run": {"steps": 1, "eval_split": split}}
    )
    assert result["load_ready"] and result["inference_ready"]
    assert not result["resumable_ready"] and result["checkpoint_kind"] == "inference_only"
    assert not (output / "training_state.pt").exists()
    assert result == json.loads((output / "training.json").read_text())
    assert result["metadata"]["camera_coverage_verified"]
    assert len(result["metadata"]["features"]) == 3
    assert result["metadata"]["features"][1]["component_order"][-1] == "arm/gripper"
    assert result["metadata"]["features"][1]["units"] is None
    assert result["metadata"]["fps"] == 30
    assert result["metadata"]["versions"]["lerobot"] == "0.6.0"
    assert result["eval_indices"] == ([1] if split else [])
    assert result["train_indices"] == ([0] if split else [0, 1])
    if split:
        assert result["evaluation"]["frames"] > 0
    else:
        assert result["evaluation"] is None
    assert load_check({"path": str(output)})["no_action_executed"]


def test_empty_split_protocol(monkeypatch, capsys):
    import io
    import sys

    import dimos_lerobot.learning_train as adapter

    def blocked(request):
        raise adapter.EmptySplitError("0 training episodes")

    monkeypatch.setattr(adapter, "train", blocked)
    monkeypatch.setattr(sys, "stdin", io.StringIO('{"command":"train"}'))
    adapter.main()
    assert json.loads(capsys.readouterr().out)["error"]["code"] == "empty_split"


def test_missing_camera_rejected(tmp_path):
    sidecar = {
        "observation": {"camera": {"dtype": "video", "shape": [4, 4, 3]}},
        "action": {},
        "episodes": [],
    }
    (tmp_path / "dimos_meta.json").write_text(json.dumps(sidecar))
    with pytest.raises(ValueError, match="Camera features omitted"):
        feature_metadata(tmp_path, {}, ACTConfig())
