# Robot Assets

`dimos.robot.assets` resolves robot description sources into local filesystem paths.
It is the home for Git-backed robot model assets, package-root resolution, and
generic URDF rendering helpers.

This directory is intentionally self-contained so it can be extracted later. Do
not add compatibility wrappers outside this module for new code. Import directly
from the source modules, for example:

```python
from dimos.robot.assets.source import RobotDescriptionSource
```

There is no `__init__.py` on purpose: DimOS disallows package `__init__.py` files
except at the root package to avoid accidental import side effects.

## Cache behavior

Assets live under:

```text
<platform user cache>/dimos/robot_assets/
├── sources/                 # Git checkouts by source identity
├── locks/                   # per-source file locks
└── derived/
    ├── rendered_urdfs/      # generic rendered URDF cache
    └── drake_urdfs/         # Drake-specific prepared URDF cache
```

`GitAssetCache` uses the “fresh-when-safe” policy:

- clone when the source is missing;
- update clean cached repos before use;
- warn and keep cached content if update fails;
- warn and skip update for dirty cached repos, preserving local edits.

## Using a robot description source

Create a source handle wherever the robot adapter or catalog is defined, then
join paths from the repository root:

```python
from dimos.robot.assets.source import RobotDescriptionSource

_MYARM_REPO = RobotDescriptionSource(
    url="https://github.com/example/myarm_description",
    ref="main",
)

model_path = _MYARM_REPO / "urdf" / "myarm.urdf.xacro"
package_paths = {"myarm_description": _MYARM_REPO / "."}
```

Package roots map ROS package names to source-relative directories. These roots
are used for `package://...` URIs and Xacro `$(find package_name)`.

## Using assets in catalogs

Catalogs should stay lazy at import time:

```python
from dimos.robot.assets.source import RobotDescriptionSource

_MYARM_REPO = RobotDescriptionSource(url="https://github.com/example/myarm_description", ref="main")

model_path = _MYARM_REPO / "urdf" / "myarm.urdf.xacro"
package_paths = {"myarm_description": _MYARM_REPO / "."}
```

`RobotDescriptionPath` defers clone/update/path validation until path operations
such as `str(path)`, `path.resolve()`, or `path.exists()`.

## Rendering URDFs

Use `processing.py` for generic robot-description rendering:

```python
from dimos.robot.assets.processing import render_urdf

rendered_path = render_urdf(
    model_path,
    package_paths,
    xacro_args={"limited": "true"},
    package_uri_mode="preserve",  # or "absolute"
)
```

Keep consumer-specific processing outside this module. For example, Drake-specific
cleanup still belongs in `dimos/manipulation/planning/utils/mesh_utils.py`.
