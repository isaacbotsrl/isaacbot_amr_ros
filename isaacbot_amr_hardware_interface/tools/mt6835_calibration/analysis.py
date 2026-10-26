# Copyright 2026 IsaacBot
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

"""
Speed and angle-error analysis of an MT6835 angle stream. Pure, no I/O.

The motor turns at a near-constant speed, so the true angle is close to a straight line in
time. What the sensor reports minus that line is its angle error plus any real speed change.
Expressed against the reported angle itself, an error that repeats per revolution shows up as
harmonics of the mechanical angle: the 2x term is what turns into the wheel pulsing.
"""

import math

from mt6835_protocol import COUNTS_PER_REV


def unwrap(raw: list) -> list:
    """Return cumulative counts from 21-bit raw angles (each step must be under half a turn)."""
    if not raw:
        return []
    out = [raw[0]]
    for prev, cur in zip(raw, raw[1:]):
        step = cur - prev
        if step > COUNTS_PER_REV // 2:
            step -= COUNTS_PER_REV
        elif step < -COUNTS_PER_REV // 2:
            step += COUNTS_PER_REV
        out.append(out[-1] + step)
    return out


def linear_fit(t: list, y: list) -> tuple:
    """Least-squares y = a + b*t; return (a, b)."""
    n = len(t)
    if n < 2:
        raise ValueError('need at least two samples')
    mean_t = sum(t) / n
    mean_y = sum(y) / n
    var_t = sum((ti - mean_t) ** 2 for ti in t)
    if var_t == 0:
        raise ValueError('all samples at the same time')
    b = sum((ti - mean_t) * (yi - mean_y) for ti, yi in zip(t, y)) / var_t
    return mean_y - b * mean_t, b


def rpm(t_s: list, raw: list) -> float:
    """Return the mean speed in RPM (signed) of an angle stream."""
    _, counts_per_s = linear_fit(t_s, unwrap(raw))
    return counts_per_s * 60.0 / COUNTS_PER_REV


def error_harmonics(t_s: list, raw: list, orders=(1, 2, 3, 4)) -> dict:
    """
    Return per-revolution harmonics of the angle error, in degrees.

    Result: {'rpm', 'revolutions', 'rms_deg', 'harmonics': {k: amplitude_deg}}.
    """
    counts = unwrap(raw)
    a, b = linear_fit(t_s, counts)
    residual_deg = [(c - (a + b * t)) * 360.0 / COUNTS_PER_REV for t, c in zip(t_s, counts)]
    theta = [2.0 * math.pi * (c % COUNTS_PER_REV) / COUNTS_PER_REV for c in counts]
    n = len(residual_deg)
    harmonics = {}
    for k in orders:
        re = sum(e * math.cos(k * th) for e, th in zip(residual_deg, theta)) * 2.0 / n
        im = sum(e * math.sin(k * th) for e, th in zip(residual_deg, theta)) * 2.0 / n
        harmonics[k] = math.hypot(re, im)
    rms = math.sqrt(sum(e * e for e in residual_deg) / n)
    return {
        'rpm': b * 60.0 / COUNTS_PER_REV,
        'revolutions': abs(counts[-1] - counts[0]) / COUNTS_PER_REV,
        'rms_deg': rms,
        'harmonics': harmonics,
    }
