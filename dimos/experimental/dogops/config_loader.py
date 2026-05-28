from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from dimos.experimental.dogops.models import (
    DogOpsConfig,
    Manifest,
    MissionConfig,
    PolicyConfig,
    SiteConfig,
)

DEFAULT_SITE = Path("examples/dogops/site_demo.yaml")
DEFAULT_MANIFEST = Path("examples/dogops/manifest_demo.yaml")
DEFAULT_POLICY = Path("examples/dogops/policy_demo.yaml")
DEFAULT_MISSION = Path("examples/dogops/mission_demo.yaml")


def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"DogOps config not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"DogOps config must be a mapping: {path}")
    return data


def load_site_config(path: str | Path = DEFAULT_SITE) -> SiteConfig:
    return SiteConfig.model_validate(_read_yaml(Path(path)))


def load_manifest(path: str | Path = DEFAULT_MANIFEST) -> Manifest:
    return Manifest.model_validate(_read_yaml(Path(path)))


def load_policy(path: str | Path = DEFAULT_POLICY) -> PolicyConfig:
    return PolicyConfig.model_validate(_read_yaml(Path(path)))


def load_mission(path: str | Path = DEFAULT_MISSION) -> MissionConfig:
    return MissionConfig.model_validate(_read_yaml(Path(path)))


def load_dogops_config(
    site_path: str | Path = DEFAULT_SITE,
    manifest_path: str | Path = DEFAULT_MANIFEST,
    mission_path: str | Path = DEFAULT_MISSION,
    policy_path: str | Path = DEFAULT_POLICY,
) -> DogOpsConfig:
    site = load_site_config(site_path)
    manifest = load_manifest(manifest_path)
    policy = load_policy(policy_path)
    mission = load_mission(mission_path)
    return DogOpsConfig(
        site=site,
        manifest=manifest,
        policy=policy,
        mission=mission,
        paths={
            "site": Path(site_path),
            "manifest": Path(manifest_path),
            "policy": Path(policy_path),
            "mission": Path(mission_path),
        },
    )
