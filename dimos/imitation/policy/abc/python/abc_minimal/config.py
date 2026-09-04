"""Inference configuration extracted from the upstream ABC minimal release."""

import os
from dataclasses import dataclass, field
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CACHE_ROOT = REPO_ROOT / "cache"


def default_cache_root() -> Path:
    return Path(os.environ.get("ABC_CACHE", str(DEFAULT_CACHE_ROOT))).expanduser()


@dataclass
class ClipConfig:
    """CLIP ViT-B/32 text asset locations."""

    cache_dir: str = field(default_factory=lambda: str(Path.home() / ".cache" / "clip"))
    model_url: str = (
        "https://openaipublic.azureedge.net/clip/models/"
        "40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af/ViT-B-32.pt"
    )
    bpe_url: str = "https://github.com/openai/CLIP/raw/main/clip/bpe_simple_vocab_16e6.txt.gz"
    model_name: str = "ViT-B-32.pt"
    bpe_name: str = "bpe_simple_vocab_16e6.txt.gz"


@dataclass
class DiTConfig:
    """ABC-DiT architecture used by the released checkpoint."""

    hidden_size: int = 1536
    depth: int = 32
    num_heads: int = 24
    mlp_ratio: float = 4.0
    state_dim: int = 14
    action_dim: int = 14
    chunk_length: int = 30
    camera_keys: tuple[str, ...] = ("top", "left", "right")
    task_embed_dim: int = 512
    vit_embed_dim: int = 768
    vit_depth: int = 12
    vit_num_heads: int = 12
    vision_pool_num_queries: int = 12
    vision_pool_num_heads: int = 8
    vision_pool_mlp_ratio: int = 4
