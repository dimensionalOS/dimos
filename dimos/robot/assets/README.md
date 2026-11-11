# Robot Assets

`dimos.robot.assets` resolves robot description sources into local filesystem paths.
It is the home for Git-backed robot model assets, package-root resolution, and
in-memory URDF/Xacro loading.

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
<DimOS cache root>/robot_assets/
├── sources/                 # Git checkouts by source identity
├── locks/                   # per-source file locks
└── derived/
    └── drake_meshes/        # content-addressed converted mesh artifacts
```

`GitAssetCache` uses the “fresh-when-safe” policy:

- clone when the source is missing;
- update clean cached repos before use;
- warn and keep cached content if update fails;
- warn and skip update for dirty cached repos or checkouts with local-only
  commits, preserving local work.

`dimos cache clean` removes robot assets along with all other DimOS caches.
Because source entries are Git checkouts, the command preserves checkouts with
uncommitted changes, untracked files, local-only commits, or Git state that
cannot be inspected. It removes the remaining cache entries and exits with
status 1 when anything is preserved. Use `dimos cache clean --force` to delete
the preserved checkouts too.

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

## Loading robot descriptions

Use `RobotModel` to keep source loading and portable model edits above backend
adapters:

```python
from dimos.robot.assets.model import RobotModel

model = (
    RobotModel.from_file(
        model_path,
        package_paths=package_paths,
        xacro_args={"limited": "true"},
    )
    .with_fixed_joints("finger_joint")
    .with_fixed_frame(
        "tool_center_point",
        "tool_flange",
        xyz=(0.1, 0.0, 0.0),
    )
)
```

`RobotModel` is lazy and pickle-safe. It expands Xacro, resolves package URIs,
and applies model edits when a backend first calls `load()`. Chained fixed
frames may use earlier additions as parents. Fixed joints retain their links
and geometry at the URDF zero pose without contributing a model degree of
freedom. Expanded XML stays in memory and is not sent across worker boundaries.

Keep consumer-specific processing outside this module. For example, Drake-specific
cleanup and optional mesh conversion still belong in
`dimos/manipulation/planning/utils/mesh_utils.py`. Only converted meshes are
cached because backends require them as filesystem artifacts.

## One layer, three readers

A USD layer here is read by three things that each take a different slice of it, and a field
one writes and another ignores diverges in silence:

| reader | what it takes | what it does not |
|---|---|---|
| **MuJoCo** (the physics world) | each prim's own `xformOp:transform`, the Mjc* schemas, `physics:axis` | the weld joints' local frames, a material's `physics:dynamicFriction` (`sim2/mujoco/world.py` applies it afterwards) |
| **`usd_to_urdf`** (the kinematics, so tf) | each joint's `physics:localPos0/localRot0` and `physics:axis`, the prim transform only for a world weld | the prim transforms of jointed bodies |
| **the Controller** (what a person sees) | `primvars:displayColor` first, then the bound material's `inputs:color` | reference arcs, so a prototype's colour has to be stated where the mesh is |

Three bugs came out of that split in one day: an added frame welded with identity local frames
(right in physics, wrong on tf), joints read as turning about z because the axis token was
ignored, and robots drawn in the converter's placeholder grey. `usd_with_frames` and
`scripts/mjcf_to_usd.py` now state each fact everywhere its readers look, and
`dimos/robot/unitree/test_vendored_usd.py::test_the_physics_and_the_tf_chain_place_a_frame_the_same`
poses MuJoCo and the tf chain at the same angles and compares them, so the next divergence of
this shape fails a test rather than a demo.
