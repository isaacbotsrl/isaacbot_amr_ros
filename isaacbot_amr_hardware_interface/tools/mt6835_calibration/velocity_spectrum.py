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
Speed-ripple spectrum of a Phidget Control Panel velocity CSV. No Arduino needed.

Record a DCC1020 channel open loop (fixed duty, wheels lifted) in the Phidget Control Panel with
a 10 ms data interval and moving average 1, then export the velocity graph as CSV. Each row is
`YYYY/MM/DD HH:MM:SS.fff, <motor RPM>`.

The script reports the speed ripple at once per wheel turn, and at 1x-4x per motor turn. For the
per-motor-turn terms it also gives the encoder angle error that would cause that ripple on its
own (velocity amplitude / (2 pi f)). Compare with the MT6835 factory INL of +-0.5 deg typical,
+-1.0 deg max. A component above the CSV's Nyquist limit is reported as not measurable.

Measured 2026-10-26 at duty ~0.2: the 2x term is +-0.5-0.9 deg on FL/FR/RR and +-2.26 deg on RL.
"""

import argparse
import cmath
import csv
import math
import statistics
from datetime import datetime

# isaacbot_amr_description/urdf/ardennes/ardennes_macro.urdf.xacro, gear_ratio.
GEAR_RATIO = 23.3


def load(path):
    """Return (seconds since the first sample, motor RPM) from a Control Panel CSV."""
    with open(path, newline='') as f:
        rows = list(csv.reader(f))[1:]
    t = [datetime.strptime(r[0], '%Y/%m/%d %H:%M:%S.%f').timestamp() for r in rows]
    return [x - t[0] for x in t], [float(r[1]) for r in rows]


def analyse(path, gear_ratio):
    t, v = load(path)
    n = len(v)
    mean = statistics.mean(v)
    x = [a - mean for a in v]
    hann = [0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1)) for i in range(n)]

    def amp(f):
        # Hann-windowed DFT at an arbitrary frequency; the samples need not be evenly spaced.
        s = sum(x[i] * hann[i] * cmath.exp(-2j * math.pi * f * t[i]) for i in range(n))
        return 2 * abs(s) / (n * 0.5)

    def near(f):
        # The speed drifts a little, so search +-3 % around the nominal frequency.
        return max((amp(f * (1 + d / 200)), f * (1 + d / 200)) for d in range(-6, 7))

    fs = (n - 1) / t[-1]
    nyquist = fs / 2
    f_motor = abs(mean) / 60
    f_wheel = f_motor / gear_ratio
    print(f'{path}: {n} samples, {t[-1]:.1f} s, mean {mean:.1f} RPM, '
          f'std {statistics.pstdev(v):.1f} RPM ({100 * statistics.pstdev(v) / abs(mean):.1f} %), '
          f'range {min(v):.0f}..{max(v):.0f}, sampled {fs:.0f} Hz')
    print(f'  motor turn {f_motor:.3f} Hz, wheel turn {f_wheel:.4f} Hz ({1 / f_wheel:.2f} s)')
    for name, f, k in (('1x wheel', f_wheel, None), ('1x motor', f_motor, 1),
                       ('2x motor', 2 * f_motor, 2), ('3x motor', 3 * f_motor, 3),
                       ('4x motor', 4 * f_motor, 4)):
        if f >= 0.95 * nyquist:
            print(f'  {name:9s} {f:6.3f} Hz  above the {nyquist:.0f} Hz Nyquist limit - '
                  'not measurable')
            continue
        a, fp = near(f)
        line = f'  {name:9s} {fp:6.3f} Hz  {a:6.2f} RPM ({100 * a / abs(mean):4.1f} %)'
        if k:
            # The same ripple as an encoder angle error: RPM -> deg/s is x6.
            line += f'   = +-{a * 6.0 / (2 * math.pi * f):.2f} deg encoder error'
        print(line)
    spec = [(amp(k / (8 * t[-1])), k / (8 * t[-1]))
            for k in range(1, int(0.95 * nyquist * t[-1] * 8))]
    peaks = sorted((spec[i] for i in range(1, len(spec) - 1)
                    if spec[i][0] > spec[i - 1][0] and spec[i][0] > spec[i + 1][0]),
                   reverse=True)
    print('  strongest peaks: ' + ', '.join(f'{a:.1f} RPM @ {f:.2f} Hz' for a, f in peaks[:6]))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('csv', nargs='+', help='Phidget Control Panel velocity CSV(s)')
    parser.add_argument('--gear-ratio', type=float, default=GEAR_RATIO,
                        help=f'motor turns per wheel turn (default {GEAR_RATIO})')
    args = parser.parse_args()
    for path in args.csv:
        analyse(path, args.gear_ratio)


if __name__ == '__main__':
    main()
