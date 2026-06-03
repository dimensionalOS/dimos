"""mujoco is ~20 MB, so we have a stub to avoid installing in lint job."""

from collections.abc import Callable
from typing import Any

import numpy as np
from numpy.typing import NDArray

# --- model / data -----------------------------------------------------

class MjModel:
    nq: int
    nv: int
    nu: int
    njnt: int
    qpos0: NDArray[np.float64]
    keyframe: Callable[[str | int], Any]
    # Mujoco's `MjModel` exposes hundreds of arrays (cam_*, jnt_*,
    # actuator_*, body_*, site_*, tendon_*, wrap_*, sensor_*, opt, …).
    # Listing every one is impractical; type rare accesses as `Any` so
    # mypy lets dimos use them without us tracking every API addition.
    def __getattr__(self, name: str) -> Any: ...
    @classmethod
    def from_xml_path(cls, path: str) -> MjModel: ...
    @classmethod
    def from_xml_string(cls, xml: str, assets: dict[str, bytes] | None = ...) -> MjModel: ...

class MjData:
    qpos: NDArray[np.float64]
    qvel: NDArray[np.float64]
    ctrl: NDArray[np.float64]
    time: float
    # See `MjModel.__getattr__`. Same idea for the per-step state arrays
    # (cam_xmat, mocap_pos, sensor, actuator_force, …).
    def __getattr__(self, name: str) -> Any: ...
    def __init__(self, model: MjModel) -> None: ...

class MjvOption:
    def __init__(self) -> None: ...

class Renderer:
    def __init__(self, model: MjModel, height: int = ..., width: int = ...) -> None: ...
    def update_scene(
        self,
        data: MjData,
        camera: int | str | Any = ...,
        scene_option: MjvOption | None = ...,
    ) -> None: ...
    def render(self) -> NDArray[np.uint8]: ...
    def enable_depth_rendering(self) -> None: ...
    def close(self) -> None: ...

# --- top-level functions ----------------------------------------------

def mj_forward(model: MjModel, data: MjData) -> None: ...
def mj_step(model: MjModel, data: MjData, nstep: int = ...) -> None: ...
def mj_resetDataKeyframe(model: MjModel, data: MjData, key: int) -> None: ...
def mj_name2id(model: MjModel, type: int, name: str | None) -> int: ...
def mj_id2name(model: MjModel, type: int, id: int) -> str | None: ...
def set_mjcb_control(
    cb: Callable[[MjModel, MjData], None] | None,
) -> None: ...

# --- enum-like namespaces ---------------------------------------------

class mjtObj:
    mjOBJ_ACTUATOR: int
    mjOBJ_BODY: int
    mjOBJ_CAMERA: int
    mjOBJ_JOINT: int
    mjOBJ_TENDON: int

class mjtJoint:
    mjJNT_HINGE: int
    mjJNT_SLIDE: int

class mjtTrn:
    mjTRN_JOINT: int
    mjTRN_TENDON: int

class mjtWrap:
    mjWRAP_JOINT: int
