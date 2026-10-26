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

"""Protocol and analysis tests against the MT6835 datasheet Rev.1.3, no hardware needed."""

import math

import analysis
import mt6835_protocol as p
import pytest


def angle_regs(raw: int, status: int = 0) -> bytes:
    """Build registers 0x003..0x006 for `raw`, with a correct CRC."""
    data = bytes((raw >> 13, (raw >> 5) & 0xFF, ((raw & 0x1F) << 3) | status))
    return data + bytes((p.crc8(data),))


def test_frames_match_the_datasheet_layout():
    # 7.6.3: C3..C0, A11..A0, D7..D0.
    assert p.frame(p.OP_READ, 0x113) == bytes((0x31, 0x13, 0x00))
    assert p.frame(p.OP_WRITE, 0x00E, 0x30) == bytes((0x60, 0x0E, 0x30))
    assert p.frame(p.OP_BURST_ANGLE, p.REG_ANGLE) == bytes((0xA0, 0x03, 0x00))
    with pytest.raises(ValueError):
        p.frame(p.OP_READ, 0x1000)


def test_no_eeprom_or_zero_opcode_is_defined():
    # Auto-calibration stores its result itself (9.2); the tool must never program EEPROM.
    assert not hasattr(p, 'OP_PROGRAM_EEPROM')
    assert not hasattr(p, 'OP_ZERO')


def test_crc8_is_poly_0x07_init_0():
    # CRC-8/SMBUS (poly 0x07, init 0, no reflection) check value for "123456789".
    assert p.crc8(b'123456789') == 0xF4
    assert p.crc8(b'\x00\x00\x00') == 0x00


def test_decode_angle_and_crc():
    sample = p.decode_angle(angle_regs(0x123456 & 0x1FFFFF, status=0b010))
    assert sample.raw == 0x123456 & 0x1FFFFF
    assert sample.status == 0b010
    assert sample.crc_ok
    assert sample.warnings == ['weak magnetic field']

    regs = bytearray(angle_regs(1000))
    regs[3] ^= 0xFF
    assert not p.decode_angle(bytes(regs)).crc_ok


def test_angle_degrees():
    assert p.decode_angle(angle_regs(1 << 20)).degrees == pytest.approx(180.0)


def test_abz_resolution_is_register_plus_one():
    # 7.3: ABZ_RES 0x0000 = 1 pulse/rev ... 0x3FFF = 16384. The Ardennes runs 1024 lines.
    value = 1024 - 1
    regs = p.abz_resolution(value >> 6, (value & 0x3F) << 2)
    assert regs == {'lines': 1024, 'abz_off': False, 'ab_swap': False}
    assert p.abz_resolution(0xFF, 0xFC | 0b11)['lines'] == 16384
    assert p.abz_resolution(0x00, 0b11) == {'lines': 1, 'abz_off': True, 'ab_swap': True}


def test_autocal_freq_keeps_the_other_bits():
    reg = 0b1000_1011  # GPIO_DS set, "MagnTek Use Only" bits 3:0 = 1011
    for code in p.AUTOCAL_RANGES_RPM:
        new = p.with_autocal_freq(reg, code)
        assert p.autocal_freq(new) == code
        assert new & 0x8F == reg & 0x8F
    with pytest.raises(ValueError):
        p.with_autocal_freq(reg, 8)


def test_autocal_code_for_rpm_follows_table_9_2():
    assert p.autocal_code_for_rpm(600) == 0x3
    assert p.autocal_code_for_rpm(400) == 0x3
    assert p.autocal_code_for_rpm(399.9) == 0x4
    assert p.autocal_code_for_rpm(3200) == 0x0
    with pytest.raises(ValueError):
        p.autocal_code_for_rpm(20)
    with pytest.raises(ValueError):
        p.autocal_code_for_rpm(6400)


def test_cal_status_bits_7_6():
    assert p.cal_status(0b0000_0000) == 0b00
    assert p.cal_status(0b0100_0000) == p.CAL_STATUS_RUNNING
    assert p.cal_status(0b1011_1111) == p.CAL_STATUS_FAILED
    assert p.cal_status(0b1100_0000) == p.CAL_STATUS_SUCCESS


def test_rot_dir_and_zero_position():
    assert p.rot_dir(0x08) == 'CW'
    assert p.rot_dir(0xF7) == 'CCW'
    assert p.zero_position(0xAB, 0xC5) == 0xABC


def synthetic_stream(rpm: float, seconds: float, error_deg: dict, rate_hz: float = 500.0):
    """Return (t, raw) for a constant speed plus per-revolution angle errors {k: deg}."""
    t, raw = [], []
    n = int(seconds * rate_hz)
    for i in range(n):
        ts = i / rate_hz
        true_rev = rpm / 60.0 * ts
        theta = 2.0 * math.pi * (true_rev % 1.0)
        err_rev = sum(a * math.sin(k * theta) for k, a in error_deg.items()) / 360.0
        t.append(ts)
        raw.append(int(round((true_rev + err_rev) * p.COUNTS_PER_REV)) % p.COUNTS_PER_REV)
    return t, raw


def test_unwrap_crosses_zero():
    raw = [p.COUNTS_PER_REV - 10, p.COUNTS_PER_REV - 5, 3, 20]
    assert analysis.unwrap(raw) == [p.COUNTS_PER_REV - 10, p.COUNTS_PER_REV - 5,
                                    p.COUNTS_PER_REV + 3, p.COUNTS_PER_REV + 20]


def test_rpm_from_stream():
    t, raw = synthetic_stream(600.0, 2.0, {})
    assert analysis.rpm(t, raw) == pytest.approx(600.0, rel=1e-3)


def test_harmonics_find_a_twice_per_revolution_error():
    # The wheel pulsing seen on 2026-10-25 locks to 2x per motor revolution.
    t, raw = synthetic_stream(600.0, 4.0, {2: 0.5})
    result = analysis.error_harmonics(t, raw)
    assert result['revolutions'] == pytest.approx(40.0, rel=0.01)
    assert result['harmonics'][2] == pytest.approx(0.5, rel=0.05)
    assert result['harmonics'][1] < 0.02
    assert result['harmonics'][3] < 0.02


def test_harmonics_of_a_clean_stream_are_tiny():
    t, raw = synthetic_stream(400.0, 4.0, {})
    result = analysis.error_harmonics(t, raw)
    assert max(result['harmonics'].values()) < 1e-3
