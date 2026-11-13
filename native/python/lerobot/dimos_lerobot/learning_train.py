"""Native LeRobot 0.6 ACT adapter; CPU-only, no robot runtime or action execution."""

from __future__ import annotations

from contextlib import ExitStack
from importlib.metadata import version
import json
from pathlib import Path
import sys
from typing import Any

from dimos_lerobot.dataprep import inspect_dataset, write
from lerobot.configs.default import DatasetConfig
from lerobot.configs.train import TrainPipelineConfig
from lerobot.configs.types import FeatureType
from lerobot.datasets.factory import make_train_eval_datasets
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.utils.feature_utils import dataset_to_policy_features
import torch
from torch.utils.data import DataLoader

from dimos.imitation.dataprep.core import inspect_episode_quality, iter_episode_samples
from dimos.imitation.dataprep.formats.lerobot.features import canonical_feature_names
from dimos.imitation.dataprep.schema import Episode, OutputConfig, RecordingSchema
from dimos.memory.store.mcap import McapStore


def emit(message: str, **extra: Any) -> None:
    print(json.dumps({"message": message, **extra}), flush=True)


class EmptySplitError(ValueError):
    """A native split with no usable training or requested evaluation set."""


def convert(request: dict[str, Any]) -> dict[str, Any]:
    snapshots = request["snapshots"]
    schema = RecordingSchema.model_validate(snapshots[0]["capture"]["schema"])
    features = {**schema.observation, **schema.action}
    output = OutputConfig(
        path=Path(request["output"]),
        format="lerobot",
        metadata={
            "repo_id": "local/learning",
            "robot_type": schema.robot_type,
            "fps": schema.sync.rate_hz,
            "feature_schema": {
                **{k: v.model_dump() for k, v in features.items()},
                "complementary_info.is_filled": {
                    "dtype": "bool",
                    "shape": [1],
                    "names": ["is_filled"],
                },
            },
        },
    )
    lineage = []
    with ExitStack() as stack:
        selections = []
        for snapshot in snapshots:
            directory = Path(snapshot["directory"])
            store = McapStore(path=str(directory / "recording.mcap"), decode_metadata=False)
            stack.callback(store.stop)
            for annotation in snapshot["annotation"]["episodes"]:
                if not annotation["included"]:
                    continue
                episode = Episode(
                    id=f"{snapshot['source']['id']}:{annotation['episode_id']}",
                    start_ts=snapshot["capture"]["origin_s"] + annotation["start_s"],
                    end_ts=snapshot["capture"]["origin_s"] + annotation["end_s"],
                    task_label=annotation["instruction"],
                    success=True,
                )
                report = inspect_episode_quality(
                    store, episode, features, schema.sync, schema.quality
                )
                if not report.valid:
                    raise ValueError(
                        f"Selected episode fails strict source quality: {report.model_dump()}"
                    )
                selections.append((store, episode))
                lineage.append(
                    {
                        "source_id": snapshot["source"]["id"],
                        "revision": snapshot["annotation"]["revision"],
                        "episode": episode.model_dump(),
                        "quality": report.model_dump(),
                    }
                )
        if not selections:
            raise ValueError("No selected episodes")

        def samples():
            for index, (store, episode) in enumerate(selections):
                yield from iter_episode_samples(
                    store=store,
                    episode=episode,
                    streams=features,
                    sync=schema.sync,
                    quality=schema.quality,
                    obs_keys=set(schema.observation),
                    action_keys=set(schema.action),
                )
                emit(
                    "Converted source episode",
                    completed=index + 1,
                    total=len(selections),
                    unit="episodes",
                )

        write(samples(), output)
    (output.path / "dimos_meta.json").write_text(
        json.dumps(
            {
                "observation": {k: v.model_dump() for k, v in schema.observation.items()},
                "action": {k: v.model_dump() for k, v in schema.action.items()},
                "feature_names": canonical_feature_names(
                    {k: v.dtype for k, v in schema.observation.items()}, schema.action
                ),
                "episodes": lineage,
            },
            default=str,
            indent=2,
        )
    )
    return inspect_dataset(output.path)


def feature_metadata(root: Path, info: dict[str, Any], config: ACTConfig) -> dict[str, Any]:
    """Bind the actual policy inputs to capture schema; unknown units stay unknown."""
    sidecar = json.loads((root / "dimos_meta.json").read_text())
    names = sidecar.get("feature_names") or canonical_feature_names(
        {k: v["dtype"] for k, v in sidecar["observation"].items()}, sidecar["action"]
    )
    inputs = set(config.input_features)
    expected_cameras = {
        names[k] for k, v in sidecar["observation"].items() if v["dtype"] in ("image", "video")
    }
    actual_cameras = {k for k, v in config.input_features.items() if v.type == FeatureType.VISUAL}
    if expected_cameras != actual_cameras:
        raise ValueError(
            f"Camera features omitted or unexpected: {expected_cameras} != {actual_cameras}"
        )
    records = []
    for role in ("observation", "action"):
        for key, definition in sidecar[role].items():
            canonical = names[key]
            if canonical not in inputs | set(config.output_features):
                raise ValueError(f"Source feature silently omitted by policy: {key}")
            records.append(
                {
                    "source_key": key,
                    "policy_key": canonical,
                    "role": role,
                    "source_schema": definition,
                    "dataset_feature": info["features"][canonical],
                    "component_order": definition.get("names"),
                    "units": definition.get("units", definition.get("unit")),
                    "units_status": "declared"
                    if definition.get("units", definition.get("unit"))
                    else "unknown_not_declared_in_capture_schema",
                }
            )
    return {
        "fps": info["fps"],
        "features": records,
        "policy_input_order": list(config.input_features),
        "policy_output_order": list(config.output_features),
        "camera_order": [k for k in config.input_features if k in actual_cameras],
        "camera_coverage_verified": True,
        "versions": {
            k: version(k) for k in ("lerobot", "torch", "torchvision", "numpy", "datasets")
        },
        "episode_lineage": sidecar["episodes"],
    }


def train(request: dict[str, Any]) -> dict[str, Any]:
    if version("lerobot") != "0.6.0":
        raise ValueError("Learning training requires the pinned LeRobot 0.6.0 environment")
    torch.set_num_threads(int(request.get("cpu_threads", 2)))
    torch.manual_seed(0)
    root = Path(request["dataset_path"])
    info = json.loads((root / "meta/info.json").read_text())
    features = dataset_to_policy_features(info["features"])
    config = ACTConfig(
        device="cpu",
        pretrained_backbone_weights=None,
        chunk_size=8,
        n_action_steps=8,
        dim_model=128,
        n_heads=4,
        dim_feedforward=512,
        n_encoder_layers=2,
        n_decoder_layers=1,
        n_vae_encoder_layers=2,
        input_features={k: v for k, v in features.items() if v.type != FeatureType.ACTION},
        output_features={k: v for k, v in features.items() if v.type == FeatureType.ACTION},
    )
    run = request["run"]
    pipeline = TrainPipelineConfig(
        dataset=DatasetConfig(
            repo_id="local/learning",
            root=str(root),
            video_backend="pyav",
            eval_split=run.get("eval_split", 0),
            use_imagenet_stats=False,
        ),
        policy=config,
    )
    try:
        dataset, evaluation_dataset = make_train_eval_datasets(pipeline)
    except ValueError as error:
        if "0 training episodes" in str(error):
            raise EmptySplitError(str(error)) from error
        raise
    train_indices = dataset.episodes
    if train_indices is None:
        train_indices = list(range(dataset.num_episodes))
    eval_indices = evaluation_dataset.episodes if evaluation_dataset is not None else []
    if not train_indices or (run.get("eval_split", 0) and not eval_indices):
        raise EmptySplitError("Requested native train/evaluation split is empty")
    tasks = dataset.meta.episodes["tasks"]
    per_task = {}
    for kind, indices in (("train_indices", train_indices), ("eval_indices", eval_indices)):
        for index in indices:
            task = tasks[index][0] if tasks[index] else ""
            per_task.setdefault(task, {"train_indices": [], "eval_indices": []})[kind].append(index)
    metadata = feature_metadata(root, info, config)
    emit(
        "Native per-task split",
        train_indices=train_indices,
        eval_indices=eval_indices,
        split_per_task=per_task,
    )
    pre, post = make_pre_post_processors(config, dataset_stats=dataset.meta.stats)
    policy = ACTPolicy(config)
    policy.train()
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=1e-5)
    loader = DataLoader(dataset, batch_size=1, shuffle=True, num_workers=0)
    iterator = iter(loader)
    losses = []
    for step in range(run["steps"]):
        try:
            batch = next(iterator)
        except StopIteration:
            iterator = iter(loader)
            batch = next(iterator)
        batch = pre(batch)
        loss, _ = policy.forward(batch)
        if not torch.isfinite(loss):
            raise ValueError("Non-finite ACT loss")
        loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.0)
        optimizer.step()
        optimizer.zero_grad()
        losses.append(float(loss.detach()))
        emit(
            "ACT optimizer step",
            completed=step + 1,
            total=run["steps"],
            unit="steps",
            loss=losses[-1],
        )
    evaluation = None
    if evaluation_dataset is not None:
        policy.eval()
        values = []
        with torch.no_grad():
            for batch in DataLoader(evaluation_dataset, batch_size=1, num_workers=0):
                loss, _ = policy.forward(pre(batch))
                values.append(float(loss))
        evaluation = {"frames": len(values), "mean_loss": sum(values) / len(values)}
    output = Path(request["output"])
    output.mkdir(parents=True, exist_ok=False)
    policy.save_pretrained(output / "pretrained_model")
    pre.save_pretrained(output / "pretrained_model")
    post.save_pretrained(output / "pretrained_model")

    result = {
        "steps": run["steps"],
        "losses": losses,
        "evaluation": evaluation,
        "train_indices": train_indices,
        "eval_indices": eval_indices,
        "split_per_task": per_task,
        "split_implementation": "lerobot.datasets.factory.make_train_eval_datasets",
        "metadata": metadata,
        "device": "cpu",
        "policy_type": "act",
        "lerobot_version": version("lerobot"),
        "inference_ready": True,
        "resumable_ready": False,
        "checkpoint_kind": "inference_only",
        "resume_limitation": "No resume entrypoint or sampler/loader/RNG continuation state is saved",
        "archive_ready": True,
        "load_ready": False,
        "rollout_ready": False,
    }

    # A real load check, never an action selection or robot graph.
    loaded = ACTPolicy.from_pretrained(output / "pretrained_model")
    loaded.eval()
    result["load_ready"] = load_check({"path": str(output)})["load_ready"]
    result["parameters"] = sum(p.numel() for p in loaded.parameters())
    result["policy_config"] = json.loads((output / "pretrained_model/config.json").read_text())
    (output / "training.json").write_text(json.dumps(result, indent=2))
    return result


def load_check(request: dict[str, Any]) -> dict[str, Any]:
    torch.set_num_threads(2)
    policy = ACTPolicy.from_pretrained(Path(request["path"]) / "pretrained_model")
    policy.to("cpu").eval()
    pre, post = make_pre_post_processors(
        policy.config, pretrained_path=str(Path(request["path"]) / "pretrained_model")
    )
    return {
        "load_ready": True,
        "device": "cpu",
        "parameters": sum(p.numel() for p in policy.parameters()),
        "no_action_executed": True,
    }


def main() -> None:
    request = json.loads(sys.stdin.read())
    try:
        result = {"convert": convert, "train": train, "load-check": load_check}[request["command"]](
            request
        )
    except EmptySplitError as error:
        emit("Native split blocked", error={"code": "empty_split", "message": str(error)})
        return
    emit("result", result=result)


if __name__ == "__main__":
    main()
