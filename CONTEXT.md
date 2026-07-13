# DimOS

DimOS composes independently developed robotics modules into local robot stacks.

## Language

**External Python Module**:
A DimOS module whose implementation runs in the managed environment of its own Runtime Project.
_Avoid_: host-environment module, fallback module

**Runtime Project**:
The sibling `python/` directory of an external Python module's declaration. It always contains `pyproject.toml`, may contain `pixi.toml`, and owns the isolated implementation environment.
_Avoid_: deployment target, package root
