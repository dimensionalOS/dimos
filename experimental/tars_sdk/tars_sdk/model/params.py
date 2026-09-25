"""All TARS design parameters. Edit here, then run `python -m tars_sdk.model.generate`.

Frame convention: X forward, Y left, Z up. Slabs are numbered 1..N from left (+Y) to right.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

from tars_sdk import scaling as sc


@dataclass
class Params:
    """Reference values are for the film-size (1.52 m) robot; `scale` shrinks everything.

    Use `Params().resolved()` for the actual robot: it applies `scale` with Froude scaling
    (see tars_sdk.scaling) so masses, motor limits and gains stay consistent.
    """

    scale: float = 0.5  # 0.5 = 0.76 m tall, 7.5 kg

    # --- geometry (meters) ---
    n_slabs: int = 4
    slab_width: float = 0.23  # lateral (Y)
    slab_depth: float = 0.28  # front-back (X)
    slab_gap: float = 0.01  # lateral gap between neighbouring slabs
    height: float = 1.52  # standing height with slabs retracted
    pivot_from_top: float = 0.15  # hinge axis distance below slab top
    upper_len: float = 0.95  # upper segment length; lower segment = height - upper_len
    sleeve_len: float = 0.72  # telescoping sleeve hidden inside the upper segment
    sleeve_inset: float = 0.012  # sleeve wall inset per side
    foot_pad: float = 0.012  # rubber pad thickness at the bottom of each slab
    hub_radius: float = 0.03  # central axle through all slabs
    foot_friction: float = (
        1.2  # roll hand-offs form a 90deg A-frame: needs mu >= 1 or the spokes splay
    )

    # --- styling ---
    seams_from_top: tuple[float, ...] = (0.40, 0.70)  # horizontal panel seams on upper segment
    seam_height: float = 0.006
    display_slab: int = 2  # which slab carries the display + camera
    display_w: float = 0.17
    display_h: float = 0.12
    display_from_top: float = 0.19  # display center distance below slab top
    camera_from_top: float = 0.05

    # --- mass (kg) ---
    hub_mass: float = 4.0  # axle + electronics + IMU
    upper_mass: float = 10.0  # per slab
    lower_mass: float = 4.0  # per slab (foot + sleeve)

    # --- hinge joints (slab pitch about the hub axis, Y) ---
    hinge_limit: float = math.pi  # +/- rad, full range enables wheel mode
    hinge_effort: float = 600.0  # Nm (roll hand-offs load the hinges like a wide split)
    hinge_velocity: float = 3.0  # rad/s
    hinge_damping: float = 5.0

    # --- slide joints (lower segment telescopes down) ---
    slide_travel: float = 0.70  # m (roll mode needs >0.57 to keep the hub level)
    slide_effort: float = 800.0  # N
    slide_velocity: float = 0.3  # m/s
    slide_damping: float = 50.0

    # --- simulation ---
    hinge_armature: float = 0.05  # kg m^2
    slide_armature: float = 0.1  # kg
    timestep: float = 0.002  # s
    contact_timeconst: float = 0.004  # s (solref)
    torsional_friction: float = 0.02  # m
    rolling_friction: float = 0.001  # m
    control_hz: float = 100.0

    _DIMS = {
        **dict.fromkeys(
            (
                "slab_width",
                "slab_depth",
                "slab_gap",
                "height",
                "pivot_from_top",
                "upper_len",
                "sleeve_len",
                "sleeve_inset",
                "foot_pad",
                "hub_radius",
                "seams_from_top",
                "seam_height",
                "display_w",
                "display_h",
                "display_from_top",
                "camera_from_top",
                "slide_travel",
            ),
            sc.LENGTH,
        ),
        **dict.fromkeys(("hub_mass", "upper_mass", "lower_mass", "slide_effort"), sc.MASS),
        "hinge_effort": sc.TORQUE,
        "hinge_velocity": sc.ANG_VEL,
        "hinge_damping": sc.ROT_DAMP,
        "slide_velocity": sc.SPEED,
        "slide_damping": sc.LIN_DAMP,
        "hinge_armature": sc.INERTIA,
        "slide_armature": sc.MASS,
        "timestep": sc.TIME,
        "contact_timeconst": sc.TIME,
        **dict.fromkeys(("torsional_friction", "rolling_friction"), sc.LENGTH),
        "control_hz": -sc.TIME,
    }

    def resolved(self) -> Params:
        """The robot actually built: reference values Froude-scaled by `scale`."""
        return replace(sc.froude(self, self.scale, self._DIMS), scale=1.0)

    # --- derived ---
    @property
    def lower_len(self) -> float:
        return self.height - self.upper_len

    @property
    def pivot_height(self) -> float:
        return self.height - self.pivot_from_top

    @property
    def pitch(self) -> float:
        return self.slab_width + self.slab_gap

    @property
    def span(self) -> float:
        return self.n_slabs * self.slab_width + (self.n_slabs - 1) * self.slab_gap

    def slab_y(self, i: int) -> float:
        """Lateral offset of slab i (1-indexed, left to right)."""
        return ((self.n_slabs - 1) / 2 - (i - 1)) * self.pitch

    @property
    def total_mass(self) -> float:
        return self.hub_mass + self.n_slabs * (self.upper_mass + self.lower_mass)


# Named poses: per-slab (hinge rad, slide m) plus hub height/pitch.
def poses(p: Params) -> dict[str, dict]:
    stride = 0.35
    ext = p.pivot_height * (1 / math.cos(stride) - 1)
    return {
        "stand": {
            "hub_z": p.pivot_height,
            "hub_pitch": 0.0,
            "slabs": [(0.0, 0.0)] * p.n_slabs,
        },
        "stride": {
            "hub_z": p.pivot_height + p.slab_depth / 2 * math.sin(stride),
            "hub_pitch": 0.0,
            "slabs": [(stride, ext), (-stride, ext), (-stride, ext), (stride, ext)],
        },
        "wheel": {
            "hub_z": p.pivot_height,
            "hub_pitch": 0.0,
            "slabs": [(0.0, 0.0), (math.pi, 0.0), (math.pi / 2, 0.0), (-math.pi / 2, 0.0)],
        },
    }
