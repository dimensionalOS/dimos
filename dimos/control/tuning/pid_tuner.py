#!/usr/bin/env python3
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

"""Systematic PID tuner from FOPDT plant parameters.

Given the identified FOPDT model (K, tau, theta) for a velocity channel,
computes PID gains using the **SIMC (Skogestad IMC)** method.

The cross-track PID's effective plant is::

    G_ct(s) = G_vel(s) * 1/s  =  K * exp(-theta*s) / (tau*s + 1) * 1/s

i.e. the velocity response followed by kinematic integration to position.

Usage:
    python -m dimos.control.tuning.pid_tuner --K 0.95 --tau 0.15 --theta 0.03
    python -m dimos.control.tuning.pid_tuner --from-csv step_response_data.csv
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass
class PIDGains:
    """Parallel-form PID gains: u = Kp*e + Ki*integral(e) + Kd*de/dt."""

    Kp: float
    Ki: float
    Kd: float
    method: str = ""

    def __repr__(self) -> str:
        return f"PID({self.method}: Kp={self.Kp:.4f}, Ki={self.Ki:.4f}, Kd={self.Kd:.4f})"


def simc_velocity_pi(K: float, tau: float, theta: float, tau_c: float | None = None) -> PIDGains:
    """SIMC tuning for the velocity (inner) loop.

    Treats the plant as FOPDT: G(s) = K * exp(-theta*s) / (tau*s + 1).

    Args:
        K: Steady-state gain.
        tau: Time constant (s).
        theta: Dead time (s).
        tau_c: Desired closed-loop time constant. If None, uses max(tau, 8*theta).

    Returns:
        PI gains (Kd=0).
    """
    if tau_c is None:
        tau_c = max(tau, 8.0 * theta)

    Kp = tau / (K * (tau_c + theta))
    Ti = min(tau, 4.0 * (tau_c + theta))
    Ki = Kp / Ti if Ti > 1e-6 else 0.0

    return PIDGains(Kp=Kp, Ki=Ki, Kd=0.0, method=f"SIMC-PI(tau_c={tau_c:.3f})")


def simc_cross_track_pid(
    K: float, tau: float, theta: float, tau_c: float | None = None
) -> PIDGains:
    """SIMC tuning for the cross-track (outer) loop.

    The effective plant includes a kinematic integrator::

        G_ct(s) = K / (tau*s + 1) * exp(-theta*s) * 1/s

    This is an integrating process (Type-1), so we use SIMC rules for
    integrating plants.

    Args:
        K: Velocity gain.
        tau: Velocity time constant (s).
        theta: Total dead time (s) — includes WebRTC transport delay.
        tau_c: Desired closed-loop time constant. If None, uses 4*theta.

    Returns:
        PID gains.
    """
    if tau_c is None:
        tau_c = max(4.0 * theta, 0.5)

    # For integrating process, the effective integrating gain is K/tau
    # (FOPDT + integrator approximated as integrating FOPDT).
    # SIMC for integrating process:
    #   Kp = 1 / (K_int * (tau_c + theta))  where K_int = K
    #   Ti = 4 * (tau_c + theta)
    #   Td = tau  (adds phase lead to compensate for the lag)
    Kp = 1.0 / (K * (tau_c + theta))
    Ti = 4.0 * (tau_c + theta)
    Td = tau

    Ki = Kp / Ti if Ti > 1e-6 else 0.0
    Kd = Kp * Td

    return PIDGains(Kp=Kp, Ki=Ki, Kd=Kd, method=f"SIMC-CT(tau_c={tau_c:.3f})")


def print_config_snippet(gains: PIDGains, channel: str = "cross_track") -> None:
    """Print a PathFollowerTaskConfig snippet with the computed gains."""
    print(f"\n# --- {channel} PID ({gains.method}) ---")
    if channel == "cross_track":
        print(f"ct_kp={gains.Kp:.4f},")
        print(f"ct_ki={gains.Ki:.4f},")
        print(f"ct_kd={gains.Kd:.4f},")
    else:
        print(f"Kp={gains.Kp:.4f},")
        print(f"Ki={gains.Ki:.4f},")
        print(f"Kd={gains.Kd:.4f},")


def sweep_tau_c(
    K: float, tau: float, theta: float, n_points: int = 10
) -> list[tuple[float, PIDGains]]:
    """Sweep tau_c from aggressive to conservative and return gains.

    Useful for selecting aggressiveness based on benchmark results.
    """
    tau_c_min = max(2.0 * theta, 0.1)
    tau_c_max = max(10.0 * theta, 2.0)
    step = (tau_c_max - tau_c_min) / max(n_points - 1, 1)

    results = []
    for i in range(n_points):
        tc = tau_c_min + i * step
        gains = simc_cross_track_pid(K, tau, theta, tau_c=tc)
        results.append((tc, gains))
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description="Systematic PID tuner from FOPDT parameters")
    parser.add_argument("--K", type=float, help="Steady-state gain")
    parser.add_argument("--tau", type=float, help="Time constant (s)")
    parser.add_argument("--theta", type=float, help="Dead time (s)")
    parser.add_argument(
        "--tau-c", type=float, default=None, help="Desired closed-loop time constant"
    )
    parser.add_argument("--from-csv", type=str, help="Load FOPDT params from step_response CSV")
    parser.add_argument("--sweep", action="store_true", help="Sweep tau_c and show range of gains")
    args = parser.parse_args()

    if args.from_csv:
        from dimos.control.tuning.plant_identification import load_and_identify

        results = load_and_identify(args.from_csv)
        print("\nIdentified plant parameters:")
        for _ch, params in sorted(results.items()):
            print(f"  {params}")

        # Use vx channel for cross-track tuning (lateral correction acts through vx)
        if "vx" in results:
            p = results["vx"]
            K, tau, theta = p.K, p.tau, p.theta
        else:
            ch = next(iter(results))
            p = results[ch]
            K, tau, theta = p.K, p.tau, p.theta
            print(f"\nWarning: no 'vx' channel found, using '{ch}' instead")
    else:
        if args.K is None or args.tau is None or args.theta is None:
            parser.error("Provide --K, --tau, --theta or --from-csv")
        K, tau, theta = args.K, args.tau, args.theta

    print(f"\nPlant: K={K:.4f}, tau={tau:.4f}s, theta={theta:.4f}s")

    # Velocity loop PI
    vel_gains = simc_velocity_pi(K, tau, theta, tau_c=args.tau_c)
    print(f"\nVelocity loop: {vel_gains}")

    # Cross-track PID
    ct_gains = simc_cross_track_pid(K, tau, theta, tau_c=args.tau_c)
    print(f"Cross-track:   {ct_gains}")

    print_config_snippet(ct_gains, "cross_track")

    if args.sweep:
        print("\n" + "=" * 70)
        print(f"{'tau_c':>8}  {'Kp':>8}  {'Ki':>8}  {'Kd':>8}  {'Aggressiveness':>16}")
        print("-" * 70)
        sweep = sweep_tau_c(K, tau, theta)
        for tc, gains in sweep:
            label = (
                "aggressive"
                if tc < 4 * theta
                else ("moderate" if tc < 8 * theta else "conservative")
            )
            print(f"{tc:>8.3f}  {gains.Kp:>8.4f}  {gains.Ki:>8.4f}  {gains.Kd:>8.4f}  {label:>16}")
        print("=" * 70)


if __name__ == "__main__":
    main()
