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

"""Serial client for firmware/mt6835_spi_bridge (the Arduino SPI master)."""

import time

from mt6835_protocol import decode_angle
import serial

BAUD = 250000
FIRMWARE_VERSION = '1'


class BridgeError(RuntimeError):
    """The bridge answered ERR, answered something unexpected, or did not answer."""


class Bridge:
    """One Arduino running mt6835_spi_bridge, driving one MT6835."""

    def __init__(self, port: str, baud: int = BAUD, timeout_s: float = 2.0):
        # Opening the port resets an Uno/Nano (DTR); it prints its banner once it is up.
        self._serial = serial.Serial(port, baud, timeout=timeout_s)
        time.sleep(2.0)
        self._serial.reset_input_buffer()
        reply = self._command('P')
        if reply != f'MT6835-BRIDGE {FIRMWARE_VERSION}':
            raise BridgeError(f'unexpected bridge firmware: {reply!r}')

    def close(self):
        self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _readline(self) -> str:
        line = self._serial.readline().decode('ascii', errors='replace').strip()
        if not line:
            raise BridgeError('no answer from the bridge (timeout)')
        return line

    def _command(self, line: str) -> str:
        self._serial.write((line + '\n').encode('ascii'))
        reply = self._readline()
        if reply.startswith('ERR'):
            raise BridgeError(f'{line!r}: {reply}')
        if not reply.startswith('OK'):
            raise BridgeError(f'{line!r}: unexpected reply {reply!r}')
        return reply[2:].strip()

    def read_register(self, addr: int) -> int:
        return int(self._command(f'R {addr:03X}'), 16)

    def write_register(self, addr: int, value: int) -> None:
        """Write, then read back: the MT6835 returns no acknowledge for a write (7.6.5)."""
        self._command(f'W {addr:03X} {value:02X}')
        readback = self.read_register(addr)
        if readback != value:
            raise BridgeError(
                f'write to {addr:#05x} did not stick: wrote {value:#04x}, read {readback:#04x}')

    def read_angle(self):
        return decode_angle(bytes(int(b, 16) for b in self._command('A').split()))

    def stream(self, count: int, period_us: int) -> list:
        """Return [(t_s, AngleSample)] sampled every `period_us` by the Arduino's clock."""
        header = self._command(f'S {count} {period_us}')
        if header != f'STREAM {count}':
            raise BridgeError(f'unexpected stream header {header!r}')
        old_timeout = self._serial.timeout
        self._serial.timeout = max(old_timeout, count * period_us * 1e-6 + 2.0)
        try:
            samples = []
            for _ in range(count):
                fields = self._readline().split()
                if len(fields) != 6 or fields[0] != 'D':
                    raise BridgeError(f'bad stream line {" ".join(fields)!r}')
                regs = bytes(int(b, 16) for b in fields[2:])
                samples.append((int(fields[1]) * 1e-6, decode_angle(regs)))
            if self._readline() != 'OK END':
                raise BridgeError('stream did not end with OK END')
            return samples
        finally:
            self._serial.timeout = old_timeout

    def set_pin(self, pin: int, high: bool) -> None:
        self._command(f'G {pin} {1 if high else 0}')
