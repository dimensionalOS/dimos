# Backwards-compat shim — the real module moved to dimos.core.coordination.blueprints.
# Re-export everything so existing imports keep working.
from dimos.core.coordination.blueprints import *  # noqa: F403
from dimos.core.coordination.blueprints import (  # noqa: F401 — explicit re-exports for type checkers
    Blueprint,
    _BlueprintAtom,
    autoconnect,
)
