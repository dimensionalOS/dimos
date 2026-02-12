import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "base": ["Captioner", "VlModel"],
        "florence": ["Florence2Model"],
        "moondream": ["MoondreamVlModel"],
        "moondream_hosted": ["MoondreamHostedVlModel"],
        "openai": ["OpenAIVlModel"],
        "qwen": ["QwenVlModel"],
    },
)
