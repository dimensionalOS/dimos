"""
Dimensional Interface package
"""

import lazy_loader as lazy

__getattr__, __dir__, __all__ = lazy.attach(
    __name__,
    submod_attrs={
        "api.server": ["FastAPIServer"],
    },
)
