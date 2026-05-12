"""onnxruntime is ~85 MB, so we have a stub to avoid installing in lint job."""

from typing import Any

from numpy.typing import NDArray

class NodeArg:
    name: str
    type: str
    shape: list[int | str | None]

class InferenceSession:
    def __init__(
        self,
        path_or_bytes: str | bytes,
        sess_options: Any | None = ...,
        providers: list[str] | None = ...,
        provider_options: list[dict[str, Any]] | None = ...,
        **kwargs: Any,
    ) -> None: ...
    def run(
        self,
        output_names: list[str] | None,
        input_feed: dict[str, NDArray[Any]],
        run_options: Any | None = ...,
    ) -> list[NDArray[Any]]: ...
    def get_inputs(self) -> list[NodeArg]: ...
    def get_outputs(self) -> list[NodeArg]: ...
    def get_providers(self) -> list[str]: ...

def get_available_providers() -> list[str]: ...
def get_device() -> str: ...
