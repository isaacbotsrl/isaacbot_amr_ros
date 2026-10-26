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

"""The calibrate sequence against a fake bridge and motor: order of steps and CAL_EN safety."""

import argparse

import mt6835_protocol as p
import mt6835_tool as tool
import pytest


class FakeMotor:
    """Open-loop motor whose speed is proportional to the duty cycle."""

    def __init__(self, rpm_per_duty=2500.0, max_duty=0.4):
        self.rpm_per_duty = rpm_per_duty
        self.max_duty = max_duty
        self.duty = 0.0
        self.closed = False

    def set_duty(self, duty):
        self.duty = max(-self.max_duty, min(self.max_duty, duty))
        return self.duty

    @property
    def rpm(self):
        return self.duty * self.rpm_per_duty

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.duty = 0.0
        self.closed = True


class FakeBridge:
    """MT6835 behind the bridge: registers, an angle that follows the motor, CAL_EN, status."""

    def __init__(self, motor, cal_result=p.CAL_STATUS_SUCCESS, cal_after_polls=3):
        self.motor = motor
        self.regs = {p.REG_AUTOCAL: 0b1000_1011, p.REG_CAL_STATUS: 0x00}
        self.cal_en = False
        self.events = []
        self.cal_result = cal_result
        self.cal_after_polls = cal_after_polls
        self.polls = 0
        self.rpm_override = None

    def read_angle(self):
        data = bytes((0x12, 0x34, 0x50))
        return p.decode_angle(data + bytes((p.crc8(data),)))

    def read_register(self, addr):
        if addr == p.REG_CAL_STATUS and self.cal_en:
            self.polls += 1
            done = self.polls >= self.cal_after_polls
            return (self.cal_result if done else p.CAL_STATUS_RUNNING) << 6
        return self.regs[addr]

    def write_register(self, addr, value):
        self.events.append(('write', addr, value))
        self.regs[addr] = value

    def set_pin(self, pin, high):
        self.events.append(('pin', pin, high))
        self.cal_en = high

    def stream(self, count, period_us):
        rpm = self.rpm_override if self.rpm_override is not None else self.motor.rpm
        counts_per_s = rpm / 60.0 * p.COUNTS_PER_REV
        out = []
        for i in range(count):
            t = i * period_us * 1e-6
            raw = int(counts_per_s * t) % p.COUNTS_PER_REV
            data = bytes((raw >> 13, (raw >> 5) & 0xFF, (raw & 0x1F) << 3))
            out.append((t, p.decode_angle(data + bytes((p.crc8(data),)))))
        return out


@pytest.fixture
def fakes(monkeypatch):
    motor = FakeMotor()
    bridge = FakeBridge(motor)
    monkeypatch.setattr(tool, 'open_motor', lambda args: motor)
    monkeypatch.setattr(tool.time, 'sleep', lambda s: None)
    return motor, bridge


def args(**kw):
    defaults = {'rpm': 600.0, 'cal_en_pin': 7, 'yes': True, 'hub_port': 4, 'hub_serial': None,
                'max_duty': 0.4}
    defaults.update(kw)
    return argparse.Namespace(**defaults)


def test_calibrate_success_sequence(fakes):
    motor, bridge = fakes
    assert tool.cmd_calibrate(bridge, args()) == 0

    writes = [e for e in bridge.events if e[0] == 'write']
    assert writes == [('write', p.REG_AUTOCAL, p.with_autocal_freq(0b1000_1011, 0x3))]
    pins = [e for e in bridge.events if e[0] == 'pin']
    # Low first (known state), high to start, low again at the end.
    assert pins == [('pin', 7, False), ('pin', 7, True), ('pin', 7, False)]
    assert bridge.events.index(writes[0]) < bridge.events.index(('pin', 7, True))
    assert motor.closed and motor.duty == 0.0


def test_calibrate_reports_failure_and_releases_cal_en(fakes):
    motor, bridge = fakes
    bridge.cal_result = p.CAL_STATUS_FAILED
    assert tool.cmd_calibrate(bridge, args()) == 1
    assert bridge.events[-1] == ('pin', 7, False)
    assert motor.closed


def test_calibrate_releases_cal_en_when_the_speed_leaves_the_range(fakes):
    motor, bridge = fakes
    real_stream = bridge.stream
    calls = {'n': 0}

    def stream(count, period_us):
        calls['n'] += 1
        if bridge.cal_en:
            bridge.rpm_override = 900.0  # out of 400..800 while calibrating
        return real_stream(count, period_us)

    bridge.stream = stream
    with pytest.raises(RuntimeError, match='calibration range'):
        tool.cmd_calibrate(bridge, args())
    assert bridge.events[-1] == ('pin', 7, False)
    assert motor.closed


def test_calibrate_refuses_a_speed_near_the_range_edge(fakes):
    motor, bridge = fakes
    with pytest.raises(SystemExit):
        tool.cmd_calibrate(bridge, args(rpm=410.0))
    assert bridge.events == []
    assert not motor.closed


def test_calibrate_refuses_a_weak_field(fakes):
    motor, bridge = fakes
    data = bytes((0x12, 0x34, 0x50 | 0b010))
    bridge.read_angle = lambda: p.decode_angle(data + bytes((p.crc8(data),)))
    with pytest.raises(SystemExit):
        tool.cmd_calibrate(bridge, args())
    assert bridge.events == []


def test_spin_to_trims_the_duty_to_the_target(fakes):
    motor, bridge = fakes
    duty, rpm = tool.spin_to(bridge, motor, 600.0)
    assert rpm == pytest.approx(600.0, rel=0.05)
    assert duty == pytest.approx(600.0 / 2500.0, rel=0.06)
