// Copyright 2026 IsaacBot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Serial-to-SPI bridge for one MT6835 encoder, for mt6835_tool.py. Arduino Uno/Nano (5 V logic,
// matching the encoder board's 5 V supply). Hardware SPI: D13 SCK, D12 MISO, D11 MOSI, D10 CSN.
//
// One command per line, 250000 baud (exact on a 16 MHz AVR). Replies start with OK or ERR.
//   P                    -> OK MT6835-BRIDGE <version>
//   R <addr>             -> OK <data>            register read (hex in, hex out)
//   W <addr> <data>      -> OK                   register write (the MT6835 returns no ack)
//   A                    -> OK <r3> <r4> <r5> <r6>  burst read of angle registers 0x003..0x006
//   S <count> <period_us>-> OK STREAM <count>, then <count> lines "D <t_us> <r3> <r4> <r5> <r6>",
//                           then OK END            timed angle stream
//   G <pin> <0|1>        -> OK                   drive a digital pin (D2..D9) - e.g. CAL_EN
//
// No decoding happens here (CRC, angle, fields): the host does it, where it is tested. There is
// deliberately no EEPROM-program or zero-setting command: the tool does not need them.

#include <SPI.h>

static const char kVersion[] = "1";
static const uint8_t kCsPin = 10;
static const uint32_t kBaud = 250000;

// MT6835: SPI mode 3, MSB first (datasheet 7.6.2). 1 MHz is well inside its 60 ns clock period.
static const SPISettings kSpiSettings(1000000, MSBFIRST, SPI_MODE3);

static const uint8_t kOpRead = 0x3;
static const uint8_t kOpWrite = 0x6;
static const uint8_t kOpBurstAngle = 0xA;
static const uint16_t kRegAngle = 0x003;

static char line_buf[48];
static uint8_t line_len = 0;

static void transfer(uint8_t * buf, uint8_t n)
{
    SPI.beginTransaction(kSpiSettings);
    digitalWrite(kCsPin, LOW);
    delayMicroseconds(1);  // TL >= 100 ns between CSN falling and the first SCK edge
    SPI.transfer(buf, n);
    delayMicroseconds(1);  // TH >= half a clock period before CSN rises
    digitalWrite(kCsPin, HIGH);
    SPI.endTransaction();
}

static void frame(uint8_t * buf, uint8_t op, uint16_t addr, uint8_t data)
{
    buf[0] = static_cast<uint8_t>((op << 4) | ((addr >> 8) & 0x0F));
    buf[1] = static_cast<uint8_t>(addr & 0xFF);
    buf[2] = data;
}

static void printHex(uint8_t value)
{
    if (value < 0x10) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

static void readAngle(uint8_t * regs)
{
    uint8_t buf[6] = {0};
    frame(buf, kOpBurstAngle, kRegAngle, 0);
    transfer(buf, 6);
    for (uint8_t i = 0; i < 4; ++i) {
        regs[i] = buf[2 + i];
    }
}

static void printAngle(const uint8_t * regs)
{
    for (uint8_t i = 0; i < 4; ++i) {
        Serial.print(' ');
        printHex(regs[i]);
    }
}

static bool parseHex(char * token, uint32_t max_value, uint32_t * out)
{
    if (token == nullptr) {
        return false;
    }
    char * end = nullptr;
    const uint32_t value = strtoul(token, &end, 16);
    if (end == token || *end != '\0' || value > max_value) {
        return false;
    }
    *out = value;
    return true;
}

static bool parseDec(char * token, uint32_t min_value, uint32_t max_value, uint32_t * out)
{
    if (token == nullptr) {
        return false;
    }
    char * end = nullptr;
    const uint32_t value = strtoul(token, &end, 10);
    if (end == token || *end != '\0' || value < min_value || value > max_value) {
        return false;
    }
    *out = value;
    return true;
}

static void handle(char * cmd)
{
    char * op = strtok(cmd, " ");
    if (op == nullptr) {
        return;
    }

    uint32_t a = 0;
    uint32_t b = 0;
    uint8_t buf[3] = {0};

    switch (op[0]) {
        case 'P':
            Serial.print(F("OK MT6835-BRIDGE "));
            Serial.println(kVersion);
            return;

        case 'R':
            if (!parseHex(strtok(nullptr, " "), 0xFFF, &a)) {
                break;
            }
            frame(buf, kOpRead, static_cast<uint16_t>(a), 0);
            transfer(buf, 3);
            Serial.print(F("OK "));
            printHex(buf[2]);
            Serial.println();
            return;

        case 'W':
            if (!parseHex(strtok(nullptr, " "), 0xFFF, &a) ||
                !parseHex(strtok(nullptr, " "), 0xFF, &b))
            {
                break;
            }
            frame(buf, kOpWrite, static_cast<uint16_t>(a), static_cast<uint8_t>(b));
            transfer(buf, 3);
            Serial.println(F("OK"));
            return;

        case 'A': {
            uint8_t regs[4];
            readAngle(regs);
            Serial.print(F("OK"));
            printAngle(regs);
            Serial.println();
            return;
        }

        case 'S': {
            if (!parseDec(strtok(nullptr, " "), 1, 20000, &a) ||
                !parseDec(strtok(nullptr, " "), 500, 1000000, &b))
            {
                break;
            }
            Serial.print(F("OK STREAM "));
            Serial.println(a);
            const uint32_t t0 = micros();
            for (uint32_t i = 0; i < a; ++i) {
                while (micros() - t0 < i * b) {
                }
                uint8_t regs[4];
                const uint32_t t = micros() - t0;
                readAngle(regs);
                Serial.print(F("D "));
                Serial.print(t);
                printAngle(regs);
                Serial.println();
            }
            Serial.println(F("OK END"));
            return;
        }

        case 'G':
            if (!parseDec(strtok(nullptr, " "), 2, 9, &a) ||
                !parseDec(strtok(nullptr, " "), 0, 1, &b))
            {
                break;
            }
            pinMode(static_cast<uint8_t>(a), OUTPUT);
            digitalWrite(static_cast<uint8_t>(a), b ? HIGH : LOW);
            Serial.println(F("OK"));
            return;

        default:
            Serial.println(F("ERR UNKNOWN"));
            return;
    }

    Serial.println(F("ERR ARGS"));
}

void setup()
{
    pinMode(kCsPin, OUTPUT);
    digitalWrite(kCsPin, HIGH);
    SPI.begin();
    Serial.begin(kBaud);
    Serial.print(F("OK MT6835-BRIDGE "));
    Serial.println(kVersion);
}

void loop()
{
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            line_buf[line_len] = '\0';
            handle(line_buf);
            line_len = 0;
        } else if (line_len < sizeof(line_buf) - 1) {
            line_buf[line_len++] = c;
        }
    }
}
