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
MT6835 SPI protocol: frames, CRC and register decoding. Pure, no I/O.

Every value here is from the MagnTek MT6835 datasheet Rev.1.3
(isaacbot_amr_hardware_interface/docs/MT6835_Rev.1.3.pdf); the section is named next to it.
SimpleFOC's MIT MT6835 driver agrees, with two exceptions the datasheet settles: a register
write has no acknowledge (MISO is Hi-Z, 7.6.5), and the ABZ resolution is the register value
plus one (7.3).
"""

from dataclasses import dataclass

# Operation commands C3..C0 (7.6.3). Program EEPROM (0b1100) is deliberately absent: User
# Auto-Calibration stores its result itself (9.2), so this tool never needs it.
OP_READ = 0b0011
OP_WRITE = 0b0110
OP_BURST_ANGLE = 0b1010

# Register map (10).
REG_USER_ID = 0x001
REG_ANGLE = 0x003
REG_ABZ_RES_HIGH = 0x007
REG_ABZ_RES_LOW = 0x008
REG_ZERO_HIGH = 0x009
REG_ZERO_LOW = 0x00A
REG_PWM_NLC = 0x00C
REG_ROT_DIR = 0x00D
REG_AUTOCAL = 0x00E
REG_BW = 0x011
REG_CAL_STATUS = 0x113

# 0x00E: bit 7 GPIO_DS, bits 6:4 AUTOCAL_FREQ, bits 3:0 "MagnTek Use Only" - never change.
AUTOCAL_FREQ_SHIFT = 4
AUTOCAL_FREQ_MASK = 0x70

# User Auto-Calibration rotation speed per AUTOCAL_FREQ code, RPM, [low, high) (9.2).
AUTOCAL_RANGES_RPM = {
    0x0: (3200, 6400),
    0x1: (1600, 3200),
    0x2: (800, 1600),
    0x3: (400, 800),
    0x4: (200, 400),
    0x5: (100, 200),
    0x6: (50, 100),
    0x7: (25, 50),
}

# 0x113[7:6] (9.2).
CAL_STATUS_TEXT = {
    0b00: 'no calibration',
    0b01: 'running',
    0b10: 'FAILED',
    0b11: 'successful',
}
CAL_STATUS_RUNNING = 0b01
CAL_STATUS_FAILED = 0b10
CAL_STATUS_SUCCESS = 0b11

# Keep rotating at the set speed for more than 64 rounds after CAL_EN goes high (9.2).
CAL_MIN_REVOLUTIONS = 64

# After a successful calibration wait more than 6 s before powering down, and do nothing else
# until the chip has been power-cycled (9.2).
CAL_SETTLE_S = 6.0

# STATUS[2:0] warning bits in the angle registers (7.6.8).
STATUS_BITS = {
    0: 'over speed',
    1: 'weak magnetic field',
    2: 'under voltage',
}

COUNTS_PER_REV = 1 << 21


def frame(op: int, addr: int, data: int = 0) -> bytes:
    """Return the 24-bit frame: C3..C0, A11..A0, D7..D0, MSB first (7.6.3)."""
    if not 0 <= op <= 0xF or not 0 <= addr <= 0xFFF or not 0 <= data <= 0xFF:
        raise ValueError(f'frame field out of range: op={op} addr={addr:#x} data={data}')
    return bytes(((op << 4) | (addr >> 8), addr & 0xFF, data))


def crc8(data: bytes) -> int:
    """Return CRC-8, polynomial x^8 + x^2 + x + 1 (0x07), MSB first, init 0 (7.6.8)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


@dataclass(frozen=True)
class AngleSample:
    """One decoded read of the angle registers 0x003..0x006."""

    raw: int
    status: int
    crc: int
    crc_ok: bool

    @property
    def degrees(self) -> float:
        return self.raw * 360.0 / COUNTS_PER_REV

    @property
    def warnings(self) -> list:
        return [text for bit, text in STATUS_BITS.items() if self.status & (1 << bit)]


def decode_angle(regs: bytes) -> AngleSample:
    """Decode the four angle registers 0x003, 0x004, 0x005, 0x006 (7.6.8)."""
    if len(regs) != 4:
        raise ValueError(f'expected 4 angle register bytes, got {len(regs)}')
    raw = (regs[0] << 13) | (regs[1] << 5) | (regs[2] >> 3)
    status = regs[2] & 0x07
    return AngleSample(raw=raw, status=status, crc=regs[3], crc_ok=crc8(regs[:3]) == regs[3])


def abz_resolution(high: int, low: int) -> dict:
    """Decode 0x007/0x008: lines per revolution (register value + 1), ABZ_OFF, AB_SWAP (10)."""
    value = (high << 6) | (low >> 2)
    return {'lines': value + 1, 'abz_off': bool(low & 0x02), 'ab_swap': bool(low & 0x01)}


def zero_position(high: int, low: int) -> int:
    """Return ZERO_POS[11:0] from 0x009/0x00A (10)."""
    return (high << 4) | (low >> 4)


def rot_dir(reg: int) -> str:
    """Return ROT_DIR, 0x00D bit 3: 0 counter-clockwise, 1 clockwise (7.3)."""
    return 'CW' if reg & 0x08 else 'CCW'


def autocal_freq(reg: int) -> int:
    """Return AUTOCAL_FREQ[2:0] from 0x00E bits 6:4."""
    return (reg & AUTOCAL_FREQ_MASK) >> AUTOCAL_FREQ_SHIFT


def with_autocal_freq(reg: int, code: int) -> int:
    """Return 0x00E with AUTOCAL_FREQ set to `code`, every other bit untouched."""
    if code not in AUTOCAL_RANGES_RPM:
        raise ValueError(f'invalid AUTOCAL_FREQ code {code}')
    return (reg & ~AUTOCAL_FREQ_MASK & 0xFF) | (code << AUTOCAL_FREQ_SHIFT)


def autocal_code_for_rpm(rpm: float) -> int:
    """Return the AUTOCAL_FREQ code whose speed range contains `rpm`."""
    for code, (low, high) in AUTOCAL_RANGES_RPM.items():
        if low <= rpm < high:
            return code
    raise ValueError(f'{rpm:.0f} RPM is outside every auto-calibration range (25..6400 RPM)')


def cal_status(reg: int) -> int:
    """Return the calibration status 0x113[7:6]."""
    return (reg >> 6) & 0x03
