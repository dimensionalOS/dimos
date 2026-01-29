from dimos.models.embedding.base import Embedding, EmbeddingModel

__all__ = [
    "Embedding",
    "EmbeddingModel",
]

# Optional: CLIP support
try:
    from dimos.models.embedding.clip import CLIPEmbedding, CLIPModel
except ImportError:
    pass
else:
    __all__ += ["CLIPEmbedding", "CLIPModel"]

# Optional: MobileCLIP support
try:
    from dimos.models.embedding.mobileclip import MobileCLIPEmbedding, MobileCLIPModel
except ImportError:
    pass
else:
    __all__ += ["MobileCLIPEmbedding", "MobileCLIPModel"]

# Optional: TorchReID support
try:
    from dimos.models.embedding.treid import TorchReIDEmbedding, TorchReIDModel
except ImportError:
    pass
else:
    __all__ += ["TorchReIDEmbedding", "TorchReIDModel"]
