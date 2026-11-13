import importlib
import sys

importlib.import_module("dimos.imitation.learning.app")
assert "dimos.imitation.learning.app" in sys.modules

banned = [
    key
    for key in sys.modules
    if key.startswith(
        ("dimos.robot", "dimos.core.coordination", "dimos.imitation.policy", "PySide")
    )
]
assert not banned, banned
print("learning lightweight import ok:", len(sys.modules), "modules, no robot graph")
