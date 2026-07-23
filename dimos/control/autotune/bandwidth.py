# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Frequency-domain views of a fitted FOPDT plant: bandwidth, Bode, pole-zero.

All three are closed-form from the fitted ``(K, tau, L)`` - no chirp, no
swept-sine, and no ``python-control`` dependency (scipy/matplotlib are already
deps; control is not, and for a FOPDT it would only obscure the deadtime
approximation we deliberately label).

FOPDT transfer function:

    G(s) = K * exp(-L s) / (tau s + 1)

  * magnitude  |G(jw)| = K / sqrt(1 + (w tau)^2)
  * phase      arg G(jw) = -atan(w tau) - w L      (deadtime = linear phase ramp)

The only physical pole is at s = -1/tau; there are NO finite zeros. The
deadtime exp(-L s) is non-rational, so the pole-zero MAP uses a first-order
Pade approximation,

    exp(-L s) ~= (1 - L s / 2) / (1 + L s / 2),

which introduces ONE extra pole at s = -2/L and ONE right-half-plane zero at
s = +2/L. These are approximation artifacts, flagged as such so they are never
read as physical plant roots.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Bandwidth is only meaningful for a trustworthy fit. Gate on this R^2 unless
# the caller overrides; below it, bandwidth is withheld.
DEFAULT_FIT_QUALITY_GATE = 0.8


def fopdt_response(
    K: float, tau: float, L: float, omega: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Complex magnitude and phase (rad) of the FOPDT at angular freqs ``omega``."""
    if tau <= 0:
        raise ValueError("tau must be > 0")
    jw_tau = omega * tau
    mag = abs(K) / np.sqrt(1.0 + jw_tau**2)
    phase = -np.arctan(jw_tau) - omega * L
    return mag, phase


def bandwidth_hz(
    K: float,
    tau: float,
    L: float,
    *,
    r2: float | None = None,
    quality_gate: float = DEFAULT_FIT_QUALITY_GATE,
) -> float | None:
    """Open-loop -3 dB bandwidth (Hz) of the FOPDT, or ``None`` if the fit is
    too poor to trust.

    For a first-order lag the -3 dB magnitude point is w = 1/tau, independent of
    the (all-pass) deadtime - deadtime adds phase lag, not magnitude rolloff. So
    the bandwidth is ``1 / (2*pi*tau)``. ``L`` is accepted for signature
    symmetry and to make the deadtime's effect on achievable *closed-loop*
    bandwidth explicit to callers (it caps it well below this open-loop number).
    """
    if r2 is not None and r2 < quality_gate:
        return None
    if tau <= 0:
        return None
    return float(1.0 / (2.0 * np.pi * tau))


@dataclass(frozen=True)
class PoleZero:
    """A pole or zero on the s-plane, tagged by origin so Pade artifacts are
    never confused with the physical plant root."""

    s: complex
    kind: str  # "pole" | "zero"
    origin: str  # "physical" | "pade_pole" | "pade_zero_rhp"


def pole_zero_map(K: float, tau: float, L: float) -> list[PoleZero]:
    """Pole-zero list for the FOPDT with a 1st-order Pade deadtime.

    Physical: one pole at -1/tau, no finite zeros.
    Pade artifacts (only when ``L`` > 0): a pole at -2/L and an RHP zero at
    +2/L, both labeled as approximation artifacts.
    """
    if tau <= 0:
        raise ValueError("tau must be > 0")
    out = [PoleZero(s=complex(-1.0 / tau, 0.0), kind="pole", origin="physical")]
    if L > 0:
        out.append(PoleZero(s=complex(-2.0 / L, 0.0), kind="pole", origin="pade_pole"))
        out.append(PoleZero(s=complex(+2.0 / L, 0.0), kind="zero", origin="pade_zero_rhp"))
    return out


@dataclass
class FrequencyPlots:
    """Paths of emitted per-axis Bode and pole-zero figures."""

    bode_paths: dict[str, str] = field(default_factory=dict)
    pole_zero_paths: dict[str, str] = field(default_factory=dict)


def _decade_grid(tau: float, L: float, n: int = 400) -> np.ndarray:
    """Log-spaced angular-frequency grid spanning two decades either side of the
    corner 1/tau (and the deadtime corner if faster)."""
    corner = 1.0 / tau
    hi_feature = max(corner, (2.0 / L) if L > 0 else corner)
    return np.logspace(np.log10(corner / 100.0), np.log10(hi_feature * 100.0), n)


def plot_bode(K: float, tau: float, L: float, out_path: str, title: str = "") -> str:
    """Write a Bode magnitude/phase figure for one axis. Returns ``out_path``."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    omega = _decade_grid(tau, L)
    mag, phase = fopdt_response(K, tau, L, omega)
    f_hz = omega / (2.0 * np.pi)

    fig, (ax_mag, ax_ph) = plt.subplots(2, 1, sharex=True, figsize=(7, 6))
    ax_mag.semilogx(f_hz, 20.0 * np.log10(np.maximum(mag, 1e-12)))
    ax_mag.axhline(20.0 * np.log10(abs(K) / np.sqrt(2.0)), ls=":", color="gray", lw=1)
    ax_mag.axvline(1.0 / (2.0 * np.pi * tau), ls="--", color="C3", lw=1, label="-3 dB (1/tau)")
    ax_mag.set_ylabel("magnitude (dB)")
    ax_mag.legend(loc="best", fontsize=8)
    ax_mag.grid(True, which="both", alpha=0.3)
    ax_ph.semilogx(f_hz, np.degrees(phase))
    ax_ph.set_ylabel("phase (deg)")
    ax_ph.set_xlabel("frequency (Hz)")
    ax_ph.grid(True, which="both", alpha=0.3)
    fig.suptitle(title or f"Bode - K={K:.3g} tau={tau:.3g}s L={L:.3g}s")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path


def plot_pole_zero(K: float, tau: float, L: float, out_path: str, title: str = "") -> str:
    """Write a pole-zero figure for one axis, styling physical vs Pade-artifact
    roots distinctly. Returns ``out_path``."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pzs = pole_zero_map(K, tau, L)
    fig, ax = plt.subplots(figsize=(6, 6))
    styles = {
        "physical": dict(marker="x", color="C0", s=90, label="physical pole (-1/tau)"),
        "pade_pole": dict(marker="x", color="0.6", s=70, label="Pade pole (artifact)"),
        "pade_zero_rhp": dict(
            marker="o", color="0.6", s=70, facecolors="none", label="Pade RHP zero (artifact)"
        ),
    }
    seen: set[str] = set()
    for pz in pzs:
        st = dict(styles[pz.origin])
        label = st.pop("label")
        ax.scatter(pz.s.real, pz.s.imag, label=label if label not in seen else None, **st)
        seen.add(label)
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(0, color="k", lw=0.5)
    ax.set_xlabel("Re(s)")
    ax.set_ylabel("Im(s)")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title(title or f"Pole-Zero (1st-order Pade deadtime) - tau={tau:.3g}s L={L:.3g}s")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return out_path
