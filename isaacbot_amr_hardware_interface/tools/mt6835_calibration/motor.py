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
Open-loop drive of one DCC1020 for the bench, through Phidget22's DCMotor channel.

Open loop on purpose: the board's velocity controller runs on the very encoder being measured
or calibrated, so its speed would depend on that encoder's error. A fixed duty cycle on a lifted
wheel gives a steady speed of its own, and the tool measures it from the MT6835 over SPI.

Safety: the duty cycle is capped, the board's failsafe watchdog is armed and fed from a thread
(a hung or killed tool stops the motor within the failsafe time), and close() always zeroes and
disengages.
"""

import threading
import time

MAX_DUTY_LIMIT = 0.6


class Motor:
    """One DCC1020 on a VINT hub port, driven by duty cycle."""

    def __init__(self, hub_port: int, hub_serial: int = None, max_duty: float = 0.4,
                 current_limit_a: float = 15.0, acceleration: float = 2.0,
                 failsafe_ms: int = 500):
        if not 0.0 < max_duty <= MAX_DUTY_LIMIT:
            raise ValueError(f'max_duty must be in (0, {MAX_DUTY_LIMIT}]')
        # Imported here so the SPI-only commands work without the Phidget22 package.
        from Phidget22.Devices.DCMotor import DCMotor

        self._max_duty = max_duty
        self._ch = DCMotor()
        self._ch.setHubPort(hub_port)
        if hub_serial is not None:
            self._ch.setDeviceSerialNumber(hub_serial)
        self._ch.openWaitForAttachment(10000)

        self._ch.setCurrentLimit(current_limit_a)
        self._ch.setAcceleration(min(max(acceleration, self._ch.getMinAcceleration()),
                                     self._ch.getMaxAcceleration()))
        self._ch.setTargetVelocity(0.0)
        self._ch.enableFailsafe(failsafe_ms)

        self._feed_period_s = failsafe_ms / 4000.0
        self._stop = threading.Event()
        self._feeder = threading.Thread(target=self._feed, daemon=True)
        self._feeder.start()

    def _feed(self):
        while not self._stop.wait(self._feed_period_s):
            try:
                self._ch.resetFailsafe()
            except Exception:  # noqa: B902 - a dead channel just stops being fed
                return

    @property
    def max_duty(self) -> float:
        return self._max_duty

    def set_duty(self, duty: float) -> float:
        """Set the duty cycle, clamped to +-max_duty; return what was applied."""
        duty = max(-self._max_duty, min(self._max_duty, duty))
        self._ch.setTargetVelocity(duty)
        return duty

    def close(self):
        """Zero, wait for the ramp down, stop feeding the failsafe and close the channel."""
        try:
            self._ch.setTargetVelocity(0.0)
            time.sleep(1.0)
        finally:
            self._stop.set()
            self._feeder.join(timeout=1.0)
            self._ch.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
