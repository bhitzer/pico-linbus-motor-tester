# Raspberry Pi Pico LINBus Motor Vehicle Tester

A port of the Microchip RCM3700 LINBus motor test controller to the Raspberry Pi Pico, using the Pico SDK C library.

## Original Hardware
- Microchip RCM3700 series controller
- Z-World prototyping board
- Custom LCD and keypad interface

## Pico Hardware
- Raspberry Pi Pico (RP2040)
- UART1 (GPIO 8 TX, GPIO 9 RX) for LIN communication
- MCP2004 LIN transceiver (enables/sleep pins on GPIO 10, 11)
- Optional keypad input (GPIO 12-19)
- USB serial for console output/input

## Features

### Motor Control
- Support for Z2LH, Z2RH, Z3LH, Z3RH motor configurations
- Up to 13 motors per test sequence
- Motor open/close commands with stall detection
- Position measurement and specification validation
- Safe mode configuration and verification

### LIN Bus Communication
- Hardware UART1 at 9600 baud
- LIN break generation via GPIO bit-banging
- Checksum calculation (inverted sum)
- Command frame: `0x55 + payload + checksum`
- Response timeout: 500ms per frame

### Test Sequencing
1. Get motor status (version, mode byte)
2. Initialize position (close all)
3. Open all motors, measure position
4. Close all motors, measure position
5. Validate against specification windows (±100)
6. Report pass/fail per motor

## Motor Commands (Preserved from Original)

| Function | Bytes | Notes |
|----------|-------|-------|
| Open Motor | `EC addr D0 87 FF` + `6F addr FF FF 10` | Two-frame sequence |
| Close Motor | `EC addr FF FF FF` + `6F addr D0 87 10` | Two-frame sequence |
| Init Position | `EC addr FF FF FF` | Sets to initial state |
| Get Position | `5B addr FF` then `A3` | Returns position in bytes 3-4 |
| Set Mode | `2E 20 mode FF FF` | NORMAL/SERVICE/STOP |
| Program Motor | `A8 addr addr mode version` | Service mode |
| Get Status | `D8 addr FF` then `20` | Returns version, mode byte |

## Motor Specifications

### Z2LH (2-Zone Left-Hand)
- 5 motors
- Open spec: [2289, 1895, 1879, 5489, 1527]
- Close spec: [2301, 1907, 1888, 5482, 1522]

### Z2RH (2-Zone Right-Hand)
- 5 motors
- Open spec: [1889, 1870, 5488, 1533, 2237]
- Close spec: [1904, 1879, 5479, 1529, 2247]

### Z3LH (3-Zone Left-Hand)
- 13 motors
- Motor addresses: `\x22\x33\x2B\x25\x27\x2F\x2E\x31\x26\x24\x2A\x28`
- Open/close specs per motor index

### Z3RH (3-Zone Right-Hand)
- 13 motors
- Motor addresses: `\x2B\x25\x27\x2F\x2E\x28\x2A\x24\x26\x31\x33\x22`
- Open/close specs per motor index

## Building

### Prerequisites
- Raspberry Pi Pico SDK
- ARM GCC compiler
- CMake 3.12+

### Build Steps

```bash
# Clone Pico SDK if not already done
git clone https://github.com/raspberrypi/pico-sdk.git
export PICO_SDK_PATH=../pico-sdk

# Build
mkdir build
cd build
cmake ..
make

# Flash to Pico (via UF2 or Picoprobe)
cp linbus_motor_tester.uf2 /path/to/Pico/
```

## Usage

1. Connect Pico to host computer via USB
2. Open serial terminal (115200 baud, 8N1)
3. Power up Pico and LIN transceiver
4. Menu prompt appears:
   ```
   === Raspberry Pi Pico LINBus Motor Tester ===
   Available tests:
     1: Z2LH (2-zone left-hand)
     2: Z2RH (2-zone right-hand)
     3: Z3LH (3-zone left-hand)
     4: Z3RH (3-zone right-hand)
   
   Enter test selection (1-4):
   ```
5. Enter `1`, `2`, `3`, or `4` to run test
6. Monitor test progress in serial console
7. Results printed showing PASS/FAIL per motor

## Console Output Example

```
=== Starting Motor Test ===
Motor Count: 5
Model: 20

Getting initial status...
Motor 22: A5 55 21 34 68 07
Opening motors...
Motor 22 stalled: pos=2275
Motor 2B stalled: pos=1893
Open results: 2275 1893 1879 5489 1527
Closing motors...
Close results: 2301 1907 1888 5482 1522

=== Test Complete ===
Passed: 5
Failed: 0

Motor 22: Open=2275 Close=2301 Status=PASS
Motor 2B: Open=1893 Close=1907 Status=PASS
```

## Wiring

### Pico to MCP2004 Transceiver
- GPIO 8 (UART TX) → MCP2004 TXD
- GPIO 9 (UART RX) → MCP2004 RXD
- GPIO 10 → MCP2004 EN (enable)
- GPIO 11 → MCP2004 NSLP (sleep, active low)
- GND → MCP2004 GND
- 3.3V → MCP2004 VCC

### MCP2004 to LIN Bus
- MCP2004 LIN_TX → LIN Bus (twisted pair)
- MCP2004 LIN_RX → LIN Bus (twisted pair)

## Differences from Original

- **No LCD/Keypad UI**: Console-based menu over USB serial
- **No costate coroutines**: Simplified sequential logic with timeouts
- **Single test execution**: Run one test at a time, not parallel states
- **Timeout-based stall detection**: 500ms per frame, 20s total sequence
- **USB serial for debug**: All printf() output to terminal

## Customization

### Change UART Pins
Modify at top of `linbus_motor_tester.c`:
```c
#define UART_TX_PIN 8
#define UART_RX_PIN 9
```

### Change Baud Rate
```c
#define BAUD_RATE 9600
```

### Add New Motor Configuration
Add arrays in the motor configuration section and extend `motor_mode_byte()` switch statement.

### Adjust Specification Windows
```c
const int WindowSpec = 100; /* Change to your tolerance */
```

## Troubleshooting

### No Serial Output
- Check USB cable connection
- Verify terminal is open at 115200 baud
- Try `pico_enable_stdio_usb` if not showing

### Motors Not Responding
- Verify LIN transceiver power and connections
- Check MCP2004 EN/NSLP pins active
- Confirm motor addresses in configuration
- Use oscilloscope to verify LIN bus activity

### Checksum Errors
- Verify `lin_checksum()` matches motor documentation
- Current: inverted sum (CRC = (sum % 256) ^ 0xFF)

### Position Measurements Out of Range
- Check `WindowSpec` tolerance
- Verify motor specs loaded for correct model
- Confirm stall detection timeout (20 seconds)

## References

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [MCP2004 LIN Transceiver Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/20005330A.pdf)
- [LIN Bus Specification](https://www.lin-cia.org/)
- Original: Z-World RCM3700 Motor Tester (2003)
