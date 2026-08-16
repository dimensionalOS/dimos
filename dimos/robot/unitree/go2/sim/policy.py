# Copyright 2026 Dimensional Inc.
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

"""Self-contained reader + forward pass for "FREE" v1 policy blobs.

A `.bin` bundles the speed-band HIMLoco experts and their normalization, so a
run needs only the blob -- no MNN, no `~/coding/go2` checkout. Format is
defined by ``export_freewalk_bin.py``; this is an independent numpy port of the
same forward pass, cross-checked against MNN in the frozen package's
``test_policy.py``.

    encoder: obs*hist -ELU-> 128 -ELU-> 64 -linear-> [vel(3), latent(16)]
    actor:   [obs_cur, vel, latent] -ELU-> 512 -> 256 -> 128 -linear-> act(12)

The latent is L2-normalized before it reaches the actor.

Loop 2 (:mod:`~dimos.robot.unitree.go2.sim.sysid.ground`) runs this net closed
loop in the tuned plant. THE NET MUST BE THE ONE THAT PRODUCED THE RECORDING —
a grounding against the wrong net produces confident, meaningless numbers.
``sysid.verify_net`` checks that identity by teacher-forced replay against the
recorded ``policy/lowcmd`` before anything builds on it.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import struct

import numpy as np

# band_kind values written by export_freewalk_bin.py
BAND_0_1 = 0
BAND_1_5 = 1
BAND_ROTATE = 3

Layer = tuple[np.ndarray, np.ndarray]


def _elu(x: np.ndarray) -> np.ndarray:
    return np.where(x > 0, x, np.expm1(np.minimum(x, 0)))


class _Reader:
    def __init__(self, blob: bytes) -> None:
        self.blob = blob
        self.pos = 0

    def u32(self) -> int:
        v: int = struct.unpack_from("<I", self.blob, self.pos)[0]
        self.pos += 4
        return v

    def f32(self, n: int) -> np.ndarray:
        a = np.frombuffer(self.blob, "<f4", n, self.pos).astype(np.float64)
        self.pos += 4 * n
        return a

    def branch(self) -> list[Layer]:
        layers = []
        for _ in range(self.u32()):
            nin, nout = self.u32(), self.u32()
            w = self.f32(nin * nout).reshape(nin, nout)  # row-major (nin, nout)
            layers.append((w, self.f32(nout)))
        return layers


@dataclass
class Band:
    kind: int
    encoder: list[Layer]
    actor: list[Layer]


@dataclass
class FreePolicy:
    """A "FREE" v1 blob: speed-banded HIM experts plus shared normalization."""

    hist: int
    obs_per_frame: int
    act_dim: int
    enc_vel: int
    enc_lat: int
    clip_obs: float
    clip_act: float
    ob_mean: np.ndarray
    ob_scale: np.ndarray
    act_mean: np.ndarray
    act_scale: np.ndarray
    default_pose: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    bands: dict[int, Band]

    @classmethod
    def load(cls, path: str | Path) -> FreePolicy:
        return cls.loads(Path(path).read_bytes(), str(path))

    @classmethod
    def loads(cls, blob: bytes, path: str = "<bytes>") -> FreePolicy:
        """Parse a FREE v1 blob already in memory (an embedded section, say)."""
        r = _Reader(blob)
        if r.blob[:4] != b"FREE":
            raise ValueError(f"{path}: not a FREE blob")
        r.pos = 4
        if (ver := r.u32()) != 1:
            raise ValueError(f"{path}: unsupported FREE version {ver}")
        hist, obs, act, enc_vel, enc_lat = r.u32(), r.u32(), r.u32(), r.u32(), r.u32()
        clip_obs, clip_act = r.f32(2)
        ob_mean, ob_scale = r.f32(obs), r.f32(obs)
        act_mean, act_scale = r.f32(act), r.f32(act)
        default_pose, kp, kd = r.f32(act), r.f32(act), r.f32(act)
        r.f32(6)  # band velocity thresholds; selection uses the fixed rule below
        bands = {}
        for _ in range(r.u32()):
            kind = r.u32()
            bands[kind] = Band(kind=kind, encoder=r.branch(), actor=r.branch())
        return cls(
            hist=hist,
            obs_per_frame=obs,
            act_dim=act,
            enc_vel=enc_vel,
            enc_lat=enc_lat,
            clip_obs=float(clip_obs),
            clip_act=float(clip_act),
            ob_mean=ob_mean,
            ob_scale=ob_scale,
            act_mean=act_mean,
            act_scale=act_scale,
            default_pose=default_pose,
            kp=kp,
            kd=kd,
            bands=bands,
        )

    def select(self, vx: float, vy: float, vyaw: float) -> Band:
        """Speed-band rule from ``mcf_walk.select_model``.

        Turning in place gets its own expert; otherwise the band is chosen by
        translational speed. On-robot switching adds hysteresis (io_contract
        §6) that this does not model.
        """
        speed = max(abs(vx), abs(vy))
        if speed < 0.05 and abs(vyaw) > 0.05 and BAND_ROTATE in self.bands:
            return self.bands[BAND_ROTATE]
        if speed >= 1.0 and BAND_1_5 in self.bands:
            return self.bands[BAND_1_5]
        return self.bands[BAND_0_1]

    def normalize(self, raw: np.ndarray) -> np.ndarray:
        """Normalize and clip one raw observation frame."""
        clipped: np.ndarray = np.clip(
            (raw - self.ob_mean) * self.ob_scale, -self.clip_obs, self.clip_obs
        )
        return clipped

    def forward(self, band: Band, p_obs: np.ndarray) -> np.ndarray:
        """HIM forward: history-stacked obs (newest first) -> raw action."""
        h = p_obs
        for w, b in band.encoder[:-1]:
            h = _elu(h @ w + b)
        w, b = band.encoder[-1]
        e = h @ w + b
        vel, lat = e[: self.enc_vel], e[self.enc_vel :]
        lat = lat / np.sqrt((lat * lat).sum())

        a = np.concatenate([p_obs[: self.obs_per_frame], vel, lat])
        for w, b in band.actor[:-1]:
            a = _elu(a @ w + b)
        w, b = band.actor[-1]
        out: np.ndarray = a @ w + b
        return out

    def act(self, p_obs: np.ndarray, cmd: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(action, joint_target)`` for a stacked observation."""
        band = self.select(*cmd)
        action = np.clip(self.forward(band, p_obs), -self.clip_act, self.clip_act)
        return action, action * self.act_scale + self.act_mean
