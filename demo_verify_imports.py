import sys

banned = [
    key
    for key in sys.modules
    if key.startswith(
        ("dimos.robot", "dimos.core.coordination", "dimos.imitation.policy", "PySide")
    )
]
assert not banned, banned
print("learning lightweight import ok:", len(sys.modules), "modules, no robot graph")
