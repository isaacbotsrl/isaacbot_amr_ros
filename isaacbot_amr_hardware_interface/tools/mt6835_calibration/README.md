# MT6835 encoder bench tool

Inspect, measure and auto-calibrate the MT6835 magnetic encoders on the Ardennes wheel motors.

**Why:** at a constant wheel speed, every wheel's speed ripples at **2× per motor revolution**
(4.5 Hz at 0.1 m/s, 13.5 Hz at 0.3 m/s; rear left worst at 6–8 %, measured 2026-10-25). The
DCC1020's on-board velocity loop chases that encoder error and turns it into pulsing and buzzing.
The MT6835's *User Auto-Calibration* (datasheet §9.2, typical INL < ±0.07°) addresses that error
at the source.

Datasheet: [`docs/MT6835_Rev.1.3.pdf`](../../docs/MT6835_Rev.1.3.pdf). Every protocol value in
`mt6835_protocol.py` cites its section.

## Setup

```
laptop ──USB── Arduino Uno/Nano ──SPI + CAL_EN── encoder board JP1/JP2 ── MT6835
       └─USB── VINT hub ── DCC1020 ── motor (the one carrying that encoder)
```

Use the encoder board's **JP2** (7 pins); it carries SPI and CAL_EN. From the board schematic:

| Arduino | JP2 pin | Board net | MT6835 pin |
|---|---|---|---|
| D7 (any of D2–D9, `--cal-en-pin`), through ~1 kΩ | 1 | HVPP | 4 CAL_EN |
| (leave open) | 2 | +5V | 9 VDD |
| D12 (MISO) | 3 | MISO | 5 MISO |
| D11 (MOSI) | 4 | MOSI | 6 MOSI |
| D13 (SCK) | 5 | CLK | 7 SCK |
| D10 | 6 | CS | 8 CSN |
| GND | 7 | GND | 12 VSS |

The other headers: JP1 is 1 GND, 2 CS, 3 CLK, 4 MOSI, 5 MISO, 6 +5V. JP3 is 1 +5V, 2 A, 3 B,
4 Z, 5 U, 6 V, 7 W, 8 GND (the DCC1020 side).

- A 5 V Arduino matches the encoder board's 5 V logic. A 3.3 V board (ESP32, RP2040) needs a
  level shifter.
- The encoder stays powered from its own 5 V. Don't power it from the Arduino.
- **CAL_EN is the board's `HVPP` net** (JP2 pin 1). The schematic labels U1 pin 4 `HVPP`, and
  R1 (10 kΩ) pulls it to GND. So CAL_EN is low by default, and the chip can't start calibrating
  by itself at power-on. Driving it to 5 V draws 0.5 mA through R1. The ~1 kΩ series resistor
  only protects against a wiring mistake.

### Software

```bash
pip install Phidget22 pyserial pytest
```

The Phidget22 *library* (driver) must also be installed. Windows-native Python is the simplest.
In WSL, attach both USB devices first with `usbipd attach --wsl --busid <id>` from an admin
PowerShell.

Flash `firmware/mt6835_spi_bridge/mt6835_spi_bridge.ino` with the Arduino IDE or
`arduino-cli compile --fqbn arduino:avr:uno` + `arduino-cli upload`.

## Safety preconditions

- **The rover's ROS bringup is stopped and the VINT hub is on the laptop.** Nothing feeds the
  safety PLC's watchdog, so **the motor contactor is open**. Motor power comes from the contactor
  bridge (as for the 2026-10-25 inductance measurement) or a bench supply, **so the safety PLC
  does not protect this setup.**
- **Wheels lifted.** Main power within reach.
- The tool drives the motor **open loop** (a fixed duty cycle, capped at `--max-duty`, default
  0.4). It arms the DCC1020's 500 ms failsafe and feeds it from a thread, so a killed tool stops
  the motor. It also stops above 1500 motor RPM or if the speed leaves the calibration range.
  Ctrl-C stops the motor.
- Open loop on purpose: the board's velocity loop would run on the very encoder being measured
  or calibrated.

Hub ports: `fl`=0, `fr`=1, `rl`=4, `rr`=5 (`ardennes_driver.cpp`). Each command takes a port
number or a wheel name.

## Use

```bash
cd isaacbot_amr_hardware_interface/tools/mt6835_calibration
T="python3 mt6835_tool.py --serial-port COM5"      # or /dev/ttyACM0

$T info                    # registers + one angle. Expect "ABZ resolution 1024 lines/rev"
$T angle --watch           # turn the wheel by hand: the angle must follow, CRC ok
$T measure --hub-port rl --rpm 400 --csv rl_before.csv
```

`measure` spins at a steady speed and reports the angle error against a constant speed as
harmonics per motor revolution. The **2×** term is the one that becomes the wheel pulsing.
Compare the four wheels, then:

```bash
$T calibrate --hub-port rl --rpm 600 --cal-en-pin 7
```

Calibrate steps (datasheet §9.2):
1. Sets `AUTOCAL_FREQ` for the range around `--rpm`. 600 RPM falls in 400–800; pick a speed
   well inside a range.
2. Trims the duty until the speed is steady.
3. Raises CAL_EN and keeps turning for at least 64 revolutions, polling `0x113[7:6]`.
4. Drops CAL_EN and stops the motor.

On success, wait at least 6 s, then **power-cycle the encoder** (the chip allows no other
operation until then), and run `measure` again. The chip stores the calibration itself. The tool
never sends the Program-EEPROM command (EEPROM endurance is 1000 cycles, §5).

Afterwards: move the hub back to the rover and repeat the 0.1 / 0.3 m/s cruise test. The rear-left
ripple should fall from 6–8 % toward the other wheels.

## Tests

```bash
python3 -m pytest test
```

The tests cover the protocol against the datasheet (frames, CRC-8/0x07, angle and register
decoding, auto-calibration ranges and status), the error-harmonic analysis on synthetic streams,
and the calibrate sequence against a fake bridge and motor. The fake checks the order of the
steps and that CAL_EN always goes low again.
