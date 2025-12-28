import sys
from dimos.core.stream import Out
from typing import (
    Any,
    get_args,
    get_origin,
    get_type_hints,
)

# value, address, metadata
VizMessageType = tuple[Any, str, dict]

class Viz:
    viz_auto_log_types = set()
    
    # this can be overridden by subclasses to customize where viz messages are sent
    def viz_auto_type_sender(self, msg: Any, channel_name: str, kwargs: dict) -> None:
        address = f"{self.__module__}:{self.__class__.__name__}/{channel_name}"
        self.viz.publish((msg, address, kwargs))
    
    @classmethod
    def _viz_attach_channel(cls) -> None:
        # Auto-add viz channel for automated outputs
        ann_dict = getattr(cls, "__annotations__", None) or {}
        ann_dict.setdefault("viz", Out[VizMessageType])
        cls.__annotations__ = ann_dict
    
    def _viz_attach_publish_hooks(self) -> None:
        """Mirror viz-able outputs to the viz channel."""
        try:
            globalns = {}
            for cls in self.__class__.__mro__:
                if cls.__module__ in sys.modules:
                    globalns.update(sys.modules[cls.__module__].__dict__)
            hints = get_type_hints(self.__class__, globalns=globalns, include_extras=True)
        except Exception:
            hints = {}

        for name, ann in hints.items():
            if get_origin(ann) is not Out:
                continue
            inner_args = get_args(ann) or ()
            inner = inner_args[0] if inner_args else None
            if inner not in self.viz_auto_log_types:
                continue

            out_stream = getattr(self, name, None)
            orig_publish = out_stream.publish

            def _wrapped_publish(msg, _orig=orig_publish, viz_stream=self.viz):
                _orig(msg)
                if viz_stream:
                    try:
                        self.viz_auto_type_sender(msg, name, {})
                    except Exception as e:  # pragma: no cover - defensive
                        print(f"viz_auto_type_sender failed for {name}: {e}", file=sys.stderr)
            
            out_stream.publish = _wrapped_publish  # type: ignore[assignment]