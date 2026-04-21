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

import numpy as np


def integrate_hit_miss(
    log_odds: np.ndarray,
    occupied_mask: np.ndarray,
    free_mask: np.ndarray,
    hit_inc: float,
    miss_dec: float,
    max_abs_log_odds: float,
) -> np.ndarray:
    if log_odds.shape != occupied_mask.shape or log_odds.shape != free_mask.shape:
        raise ValueError("log_odds, occupied_mask and free_mask must have the same shape")

    updated = log_odds.astype(np.float32, copy=True)
    updated[occupied_mask] += hit_inc
    updated[free_mask] -= miss_dec
    np.clip(updated, -max_abs_log_odds, max_abs_log_odds, out=updated)
    return updated


def apply_time_decay(
    log_odds: np.ndarray, last_seen_ts: np.ndarray, now: float, decay_sec: float
) -> np.ndarray:
    if log_odds.shape != last_seen_ts.shape:
        raise ValueError("log_odds and last_seen_ts must have the same shape")

    if decay_sec <= 0:
        return log_odds.astype(np.float32, copy=True)

    updated = log_odds.astype(np.float32, copy=True)
    known = last_seen_ts > 0
    if not np.any(known):
        return updated

    elapsed = np.maximum(0.0, now - last_seen_ts.astype(np.float64))
    decay = np.exp(-elapsed / decay_sec)
    updated[known] *= decay[known].astype(np.float32)
    return updated


def compute_persistence(
    persistence: np.ndarray,
    occupied_mask: np.ndarray,
    free_mask: np.ndarray,
    max_persistence: int = 100,
) -> np.ndarray:
    if persistence.shape != occupied_mask.shape or persistence.shape != free_mask.shape:
        raise ValueError("persistence, occupied_mask and free_mask must have the same shape")

    updated = persistence.astype(np.int32, copy=True)
    updated[occupied_mask] += 1
    updated[free_mask] -= 1
    np.clip(updated, 0, max_persistence, out=updated)
    return updated.astype(np.int16)
