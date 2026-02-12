import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "agent": ["Agent", "deploy"],
        "spec": ["AgentSpec"],
        "vlm_agent": ["VLMAgent"],
        "vlm_stream_tester": ["VlmStreamTester"],
        "_skill_exports": ["skill", "Output", "Reducer", "Stream"],
    },
)
