"""Native LeRobot 0.6 ACT adapter; CPU-only, no robot runtime or action execution."""

from __future__ import annotations

from contextlib import ExitStack
import json
from pathlib import Path
import sys
from typing import Any

from dimos_lerobot.dataprep import inspect_dataset, write
from lerobot.configs.types import FeatureType
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.utils.feature_utils import dataset_to_policy_features
import torch
from torch.utils.data import DataLoader

from dimos.imitation.dataprep.core import inspect_episode_quality, iter_episode_samples
from dimos.imitation.dataprep.schema import Episode, OutputConfig, RecordingSchema
from dimos.memory.store.mcap import McapStore


def emit(message: str, **extra: Any) -> None:
    print(json.dumps({"message": message, **extra}), flush=True)


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
                "episodes": lineage,
            },
            default=str,
            indent=2,
        )
    )
    return inspect_dataset(output.path)


def train(request: dict[str, Any]) -> dict[str, Any]:
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
    delta = {"action": [i / info["fps"] for i in range(config.chunk_size)]}
    run = request["run"]
    train_indices = run["train_indices"]
    eval_indices = run["eval_indices"]
    dataset = LeRobotDataset(
        "local/learning",
        root=root,
        episodes=train_indices,
        delta_timestamps=delta,
        video_backend="pyav",
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
    if eval_indices:
        evaluation_dataset = LeRobotDataset(
            "local/learning",
            root=root,
            episodes=eval_indices,
            delta_timestamps=delta,
            video_backend="pyav",
        )
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
    torch.save(
        {
            "optimizer": optimizer.state_dict(),
            "step": run["steps"],
            "torch_rng_state": torch.get_rng_state(),
        },
        output / "training_state.pt",
    )
    result = {
        "steps": run["steps"],
        "losses": losses,
        "evaluation": evaluation,
        "train_indices": train_indices,
        "eval_indices": eval_indices,
        "device": "cpu",
        "policy_type": "act",
        "lerobot_version": "0.6.0",
        "inference_ready": True,
        "resumable_ready": True,
        "archive_ready": True,
        "load_ready": False,
        "rollout_ready": False,
    }
    (output / "training.json").write_text(json.dumps(result, indent=2))
    # A real load check, never an action selection or robot graph.
    loaded = ACTPolicy.from_pretrained(output / "pretrained_model")
    loaded.eval()
    result["load_ready"] = True
    result["parameters"] = sum(p.numel() for p in loaded.parameters())
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
    result = {"convert": convert, "train": train, "load-check": load_check}[request["command"]](
        request
    )
    emit("result", result=result)


if __name__ == "__main__":
    main()
