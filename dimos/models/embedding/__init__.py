from dimos.models.embedding.base import Embedding, EmbeddingModel

__all__ = [
    "Embedding",
    "EmbeddingModel",
]

# Optional: CLIP support
try:
    from dimos.models.embedding.clip import CLIPEmbedding, CLIPModel

    __all__.extend(["CLIPEmbedding", "CLIPModel"])
except ImportError:
    pass

# Optional: MobileCLIP support
try:
    from dimos.models.embedding.mobileclip import MobileCLIPEmbedding, MobileCLIPModel

    __all__.extend(["MobileCLIPEmbedding", "MobileCLIPModel"])
except ImportError:
    pass

# Optional: TorchReID support
try:
    from dimos.models.embedding.treid import TorchReIDEmbedding, TorchReIDModel

    __all__.extend(["TorchReIDEmbedding", "TorchReIDModel"])
except ImportError:
    pass
