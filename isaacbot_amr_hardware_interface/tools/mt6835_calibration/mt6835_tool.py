#!/usr/bin/env python3
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
Bench tool for the Ardennes MT6835 wheel encoders: inspect, measure and auto-calibrate.

The MT6835 is read over SPI through an Arduino (firmware/mt6835_spi_bridge); the motor is
driven open loop through the DCC1020 on a VINT hub on the same computer (Phidget22). See
README.md for wiring and the safety preconditions. Datasheet: docs/MT6835_Rev.1.3.pdf.
"""

import argparse
import csv
import sys
import time

import analysis
from bridge import Bridge
import mt6835_protocol as p

# Ardennes hub ports (src/isaacbot_amr_driver/ardennes_driver.cpp).
HUB_PORTS = {'fl': 0, 'fr': 1, 'rl': 4, 'rr': 5}

# The motors reach ~2800 RPM at full duty (max_rpm_motor_speed); a lifted wheel never needs
# more than this here, and the tool stops if the measured speed goes past it.
HARD_MAX_RPM = 1500.0
NO_LOAD_RPM_PER_DUTY = 2800.0

STREAM_PERIOD_US = 2000


def hub_port(value: str) -> int:
    if value.lower() in HUB_PORTS:
        return HUB_PORTS[value.lower()]
    port = int(value)
    if not 0 <= port <= 5:
        raise argparse.ArgumentTypeError('hub port must be 0..5 or fl/fr/rl/rr')
    return port


def print_angle(sample) -> None:
    flags = ', '.join(sample.warnings) or 'no warnings'
    crc = 'CRC ok' if sample.crc_ok else 'CRC MISMATCH'
    print(f'angle {sample.degrees:10.5f} deg  raw {sample.raw:7d}  {crc}  ({flags})')


def cmd_info(bridge: Bridge, _args) -> int:
    regs = {addr: bridge.read_register(addr) for addr in (
        p.REG_USER_ID, p.REG_ABZ_RES_HIGH, p.REG_ABZ_RES_LOW, p.REG_ZERO_HIGH, p.REG_ZERO_LOW,
        p.REG_PWM_NLC, p.REG_ROT_DIR, p.REG_AUTOCAL, p.REG_BW, p.REG_CAL_STATUS)}
    abz = p.abz_resolution(regs[p.REG_ABZ_RES_HIGH], regs[p.REG_ABZ_RES_LOW])
    code = p.autocal_freq(regs[p.REG_AUTOCAL])
    low, high = p.AUTOCAL_RANGES_RPM[code]
    print(f'USER_ID            {regs[p.REG_USER_ID]:#04x}')
    print(f'ABZ resolution     {abz["lines"]} lines/rev  (ABZ off: {abz["abz_off"]}, '
          f'AB swap: {abz["ab_swap"]})')
    print(f'zero position      {p.zero_position(regs[p.REG_ZERO_HIGH], regs[p.REG_ZERO_LOW])}')
    print(f'rotation direction {p.rot_dir(regs[p.REG_ROT_DIR])}')
    print(f'NLC enabled        {bool(regs[p.REG_PWM_NLC] & 0x20)}')
    print(f'AUTOCAL_FREQ       {code} ({low}..{high} RPM)')
    status = p.cal_status(regs[p.REG_CAL_STATUS])
    print(f'calibration status {status:02b} ({p.CAL_STATUS_TEXT[status]})')
    print_angle(bridge.read_angle())
    if abz['lines'] != 1024:
        print('NOTE: the Ardennes is configured for 1024 lines (encoder_resolution). A different '
              'value means a different setting on this board, or a wiring/protocol problem.')
    return 0


def cmd_angle(bridge: Bridge, args) -> int:
    while True:
        print_angle(bridge.read_angle())
        if not args.watch:
            return 0
        time.sleep(0.2)


def measure_rpm(bridge: Bridge, seconds: float) -> tuple:
    """Return (rpm, samples) from an angle stream of `seconds`."""
    count = max(20, int(seconds * 1e6 / STREAM_PERIOD_US))
    samples = bridge.stream(count, STREAM_PERIOD_US)
    bad = sum(1 for _, s in samples if not s.crc_ok)
    if bad > count // 20:
        raise RuntimeError(f'{bad}/{count} angle samples failed CRC - check the SPI wiring')
    t = [ts for ts, _ in samples]
    raw = [s.raw for _, s in samples]
    return analysis.rpm(t, raw), samples


def spin_to(bridge: Bridge, motor, target_rpm: float, tolerance: float = 0.05) -> tuple:
    """Trim the open-loop duty until the measured speed is within tolerance; return (duty, rpm)."""
    duty = motor.set_duty(target_rpm / NO_LOAD_RPM_PER_DUTY)
    for _ in range(12):
        time.sleep(1.5)  # ramp (acceleration 2 duty/s) and settle
        speed, _ = measure_rpm(bridge, 0.4)
        speed = abs(speed)
        print(f'  duty {duty:.3f} -> {speed:7.1f} RPM')
        if speed > HARD_MAX_RPM:
            raise RuntimeError(f'{speed:.0f} RPM is over the {HARD_MAX_RPM:.0f} RPM bench limit')
        if abs(speed - target_rpm) <= tolerance * target_rpm:
            return duty, speed
        if speed < 1.0:
            if abs(duty) >= motor.max_duty:
                raise RuntimeError('motor does not turn at the maximum duty - is it powered?')
            duty = motor.set_duty(duty * 1.5)
            continue
        duty = motor.set_duty(duty * max(0.5, min(1.5, target_rpm / speed)))
    raise RuntimeError(f'could not settle at {target_rpm:.0f} RPM (last {speed:.0f} RPM)')


def check_rpm(rpm: float) -> None:
    if not 0 < rpm <= HARD_MAX_RPM:
        raise SystemExit(f'--rpm must be in (0, {HARD_MAX_RPM:.0f}]')


def open_motor(args):
    from motor import Motor
    return Motor(args.hub_port, hub_serial=args.hub_serial, max_duty=args.max_duty)


def cmd_spin(bridge: Bridge, args) -> int:
    check_rpm(args.rpm)
    with open_motor(args) as motor:
        _, speed = spin_to(bridge, motor, args.rpm)
        print(f'holding {speed:.1f} RPM for {args.seconds:.0f} s')
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            speed, _ = measure_rpm(bridge, 0.4)
            if abs(speed) > HARD_MAX_RPM:
                raise RuntimeError(f'overspeed: {speed:.0f} RPM')
    return 0


def cmd_measure(bridge: Bridge, args) -> int:
    check_rpm(args.rpm)
    with open_motor(args) as motor:
        spin_to(bridge, motor, args.rpm)
        _, samples = measure_rpm(bridge, args.seconds)
    t = [ts for ts, _ in samples]
    raw = [s.raw for _, s in samples]
    result = analysis.error_harmonics(t, raw)
    print(f'{result["rpm"]:.1f} RPM over {result["revolutions"]:.1f} revolutions, '
          f'{len(samples)} samples')
    print(f'angle error vs constant speed: {result["rms_deg"]:.3f} deg RMS')
    for k, amplitude in result['harmonics'].items():
        print(f'  {k}x per revolution: {amplitude:.3f} deg')
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(('t_s', 'raw', 'status', 'crc_ok'))
            for ts, s in samples:
                writer.writerow((f'{ts:.6f}', s.raw, s.status, int(s.crc_ok)))
        print(f'samples written to {args.csv}')
    return 0


def confirm_cal_en(args) -> None:
    if args.yes:
        return
    print(f'Calibration drives Arduino D{args.cal_en_pin} HIGH as the MT6835 CAL_EN (chip pin 4):')
    print('on the encoder board that is the HVPP net, JP2 pin 1 (10k pull-down R1).')
    print('Wheels lifted, motor power within reach.')
    if input('Type CAL_EN to continue: ').strip() != 'CAL_EN':
        raise SystemExit('aborted')


def cmd_calibrate(bridge: Bridge, args) -> int:
    check_rpm(args.rpm)
    code = p.autocal_code_for_rpm(args.rpm)
    low, high = p.AUTOCAL_RANGES_RPM[code]
    if not low * 1.1 <= args.rpm <= high * 0.9:
        raise SystemExit(f'pick --rpm well inside {low}..{high} RPM (10 % margin from the edges)')

    sample = bridge.read_angle()
    print_angle(sample)
    if not sample.crc_ok or sample.status & 0b110:
        raise SystemExit('the encoder reports a CRC error, weak field or under voltage: fix first')

    confirm_cal_en(args)
    bridge.set_pin(args.cal_en_pin, False)

    reg = bridge.read_register(p.REG_AUTOCAL)
    bridge.write_register(p.REG_AUTOCAL, p.with_autocal_freq(reg, code))
    print(f'AUTOCAL_FREQ = {code} ({low}..{high} RPM)')

    revolutions_s = p.CAL_MIN_REVOLUTIONS * 60.0 / args.rpm
    deadline_s = 3.0 * revolutions_s + 10.0
    status = None
    with open_motor(args) as motor:
        try:
            _, speed = spin_to(bridge, motor, args.rpm, tolerance=0.03)
            print(f'steady at {speed:.1f} RPM; starting calibration '
                  f'(>= {p.CAL_MIN_REVOLUTIONS} revolutions, ~{revolutions_s:.0f} s)')
            bridge.set_pin(args.cal_en_pin, True)
            start = time.monotonic()
            while time.monotonic() - start < deadline_s:
                time.sleep(0.5)
                status = p.cal_status(bridge.read_register(p.REG_CAL_STATUS))
                speed, _ = measure_rpm(bridge, 0.2)
                print(f'  {time.monotonic() - start:5.1f} s  {abs(speed):7.1f} RPM  '
                      f'{p.CAL_STATUS_TEXT[status]}')
                if abs(speed) > HARD_MAX_RPM or not low <= abs(speed) < high:
                    raise RuntimeError(f'speed left the calibration range: {speed:.0f} RPM')
                if status in (p.CAL_STATUS_SUCCESS, p.CAL_STATUS_FAILED):
                    break
        finally:
            bridge.set_pin(args.cal_en_pin, False)

    if status == p.CAL_STATUS_SUCCESS:
        print(f'Calibration successful. Do NOT touch the encoder for {p.CAL_SETTLE_S:.0f} s ...')
        time.sleep(p.CAL_SETTLE_S + 1.0)
        print('Now power-cycle the encoder (no other operation is allowed until then, datasheet '
              '9.2), keep CAL_EN low, and run `measure` again to compare.')
        return 0
    print(f'Calibration did not succeed: {p.CAL_STATUS_TEXT.get(status, "timed out")}. '
          'Check the magnet and the speed steadiness, then try again.')
    return 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--serial-port', required=True,
                        help='Arduino serial port, e.g. COM5 or /dev/ttyACM0')
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('info', help='decode the encoder registers and one angle')
    angle = sub.add_parser('angle', help='read the angle')
    angle.add_argument('--watch', action='store_true', help='keep reading every 0.2 s')

    def motor_args(cmd, default_rpm):
        cmd.add_argument('--hub-port', type=hub_port, required=True,
                         help='VINT hub port or wheel: fl=0 fr=1 rl=4 rr=5')
        cmd.add_argument('--hub-serial', type=int, default=None,
                         help='VINT hub serial number, if more than one hub is attached')
        cmd.add_argument('--rpm', type=float, default=default_rpm, help='motor shaft RPM')
        cmd.add_argument('--max-duty', type=float, default=0.4, help='duty cycle cap (<= 0.6)')

    spin = sub.add_parser('spin', help='spin the motor at a steady speed (open loop)')
    motor_args(spin, 400.0)
    spin.add_argument('--seconds', type=float, default=10.0)

    measure = sub.add_parser('measure', help='angle error harmonics at a steady speed')
    motor_args(measure, 400.0)
    measure.add_argument('--seconds', type=float, default=4.0)
    measure.add_argument('--csv', help='write the raw samples here')

    calibrate = sub.add_parser('calibrate', help='MT6835 User Auto-Calibration (datasheet 9.2)')
    motor_args(calibrate, 600.0)
    calibrate.add_argument('--cal-en-pin', type=int, required=True,
                           help='Arduino pin (2..9) wired to the MT6835 CAL_EN, chip pin 4')
    calibrate.add_argument('--yes', action='store_true', help='skip the wiring confirmation')

    args = parser.parse_args(argv)
    commands = {'info': cmd_info, 'angle': cmd_angle, 'spin': cmd_spin,
                'measure': cmd_measure, 'calibrate': cmd_calibrate}
    with Bridge(args.serial_port) as bridge:
        try:
            return commands[args.command](bridge, args)
        except KeyboardInterrupt:
            print('interrupted - motor stopped')
            return 130
        except RuntimeError as e:
            print(f'ERROR: {e} - motor stopped')
            return 1


if __name__ == '__main__':
    sys.exit(main())
