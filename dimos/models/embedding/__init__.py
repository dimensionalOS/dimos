from dimos.models.embedding.base import Embedding, EmbeddingModel
from dimos.models.embedding.clip import CLIPEmbedding, CLIPModel

__all__ = [
    "Embedding",
    "EmbeddingModel",
    "CLIPEmbedding",
    "CLIPModel",
]

# Optional: MobileCLIP (requires open-clip-torch)
try:
    from dimos.models.embedding.mobileclip import MobileCLIPEmbedding, MobileCLIPModel

    __all__.extend(["MobileCLIPEmbedding", "MobileCLIPModel"])
except ImportError:
    pass


# Optional: TorchReid (requires torchreid)
try:
    from dimos.models.embedding.treid import TorchReIDEmbedding, TorchReIDModel

    __all__.extend(["TorchReIDEmbedding", "TorchReIDModel"])
except ImportError:
    pass
