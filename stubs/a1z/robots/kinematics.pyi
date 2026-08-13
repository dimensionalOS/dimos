from typing import Any

import numpy as np
import numpy.typing as npt

class Kinematics:
    def __init__(self, urdf_path: str, end_effector_frame: str | None = ...) -> None: ...
    def fk(
        self,
        q: npt.NDArray[np.floating[Any]],
        frame_name: str | None = ...,
    ) -> npt.NDArray[np.floating[Any]]: ...
