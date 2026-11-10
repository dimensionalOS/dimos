# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Keep pydantic's view of generated messages on the registry-owned classes.

``rosbags`` builds generated classes under a synthetic ``rosbags.usertypes``
module.  That module can also hold a class with the same name as one the frozen
registry already owns (``builtin_interfaces/msg/Time`` ships with the empty
store), so resolving a nested dataclass annotation through the module would
build a second, unequal class.  Configuration JSON round-trips must restore the
canonical registry classes, so point the module at what the registry owns.
"""

from __future__ import annotations

import sys


def canonicalize_generated_message_types() -> None:
    """Replace ``rosbags.usertypes`` aliases with the frozen registry's classes."""
    try:
        from dimos_messages import _types as generated
    except ImportError:
        return
    module = sys.modules.get("rosbags.usertypes")
    if module is None:
        return
    for name, cls in generated.store.types.items():
        setattr(module, name.replace("/", "__"), cls)
