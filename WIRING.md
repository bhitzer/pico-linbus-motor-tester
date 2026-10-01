# Raspberry Pi Pico + MCP2004 LIN Transceiver Wiring Guide

## Overview
This document provides the exact wiring connections between the Raspberry Pi Pico and the MCP2004 LIN transceiver for LINBus motor control communication.

## MCP2004 LIN Transceiver Pinout

```
         ___________
        |  MCP2004  |
    NC  | 1       8 | VCC (3.3V)
   TXD  | 2       7 | GND
   RXD  | 3       6 | NSLP (Sleep Enable)
   LIN+ | 4       5 | EN (Enable)
        |___________|

Pin 1  - NC (No Connect)
Pin 2  - TXD (UART TX input from Pico)
Pin 3  - RXD (UART RX output to Pico)
Pin 4  - LIN+ (to LIN bus twisted pair)
Pin 5  - EN (Enable control from Pico GPIO2)
Pin 6  - NSLP (Sleep/Normal control from Pico GPIO3)
Pin 7  - GND (Ground)
Pin 8  - VCC (3.3V power)
```

## Complete Wiring Diagram

```
Raspberry Pi Pico                         MCP2004 Transceiver
================                          ===================

GPIO 0 (UART0 TX) ----[R1: 1kΩ]-------- Pin 2 (TXD)
GPIO 1 (UART0 RX) ----[R2: 1kΩ]-------- Pin 3 (RXD)
GPIO 2 (EN Control) -----direct-------- Pin 5 (EN)
GPIO 3 (NSLP Control) ---direct-------- Pin 6 (NSLP)

3V3 (Pico pin 36) ----[C1: 100nF]--+--- Pin 8 (VCC)
                                    |
                                   GND

GND (Pico pin 38) ----------------+--- Pin 7 (GND)
                                   |
                                  GND

Pin 4 (LIN+) --[twisted pair]-- LIN Bus H (High)
Pin 4 (LIN+) --[twisted pair]-- LIN Bus L (Low)
             --[termination]-- (if end node: 560Ω between H and L)
```

## Pin Mapping Summary

| Function          | Pico GPIO | MCP2004 Pin | Notes |
|-------------------|-----------|-------------|-------|
| UART0 TX          | GPIO 0    | Pin 2 (TXD) | 1kΩ series resistor |
| UART0 RX          | GPIO 1    | Pin 3 (RXD) | 1kΩ series resistor |
| Enable (EN)       | GPIO 2    | Pin 5 (EN)  | Active HIGH |
| Sleep (NSLP)      | GPIO 3    | Pin 6 (NSLP)| Active HIGH = Normal, LOW = Sleep |
| Power Supply      | 3V3       | Pin 8 (VCC) | 100nF decoupling cap |
| Ground            | GND       | Pin 7 (GND) | Common ground |
| LIN Bus           | —         | Pin 4 (LIN+)| To motor nodes (twisted pair) |

## Detailed Connection Instructions

### 1. Pico Power & GND
```
Pico Pin 36 (3V3) ───┬─── 100nF Capacitor ───┬─── MCP2004 Pin 8 (VCC)
                     │                        │
                     └────────────────────────┘

Pico Pin 38 (GND) ───────────────────────────── MCP2004 Pin 7 (GND)
                 ───────────────────────────── LIN Bus Ground (if applicable)
```

### 2. UART Communications (with protective resistors)
```
Pico GPIO 0 (TX) ───[1kΩ resistor]─── MCP2004 Pin 2 (TXD)
Pico GPIO 1 (RX) ───[1kΩ resistor]─── MCP2004 Pin 3 (RXD)
```

**Why 1kΩ resistors?**
- Protect Pico GPIO from transient spikes
- Limit fault current in case of short
- MCP2004 input impedance is high (CMOS compatible)
- 1kΩ × ~10pF (pin capacitance) = negligible time constant at 9600 baud

### 3. Control Lines (direct GPIO)
```
Pico GPIO 2 (EN output) ───────────── MCP2004 Pin 5 (EN)
Pico GPIO 3 (NSLP output) ─────────── MCP2004 Pin 6 (NSLP)
```

**EN (Enable) Pin:**
- HIGH = Transceiver ACTIVE (normal operation)
- LOW = Transceiver STANDBY
- Software initializes to HIGH at startup

**NSLP (Sleep) Pin:**
- HIGH = Normal mode (transceiver active)
- LOW = Sleep/reduced power mode
- Software initializes to HIGH at startup

### 4. LIN Bus Connection

The MCP2004 Pin 4 (LIN+) output goes to a twisted-pair cable that connects all motor nodes:

```
MCP2004 Pin 4 (LIN+) ──┬─── Motor Node 1
                       ├─── Motor Node 2
                       ├─── Motor Node 3
                       └─── [Optional: Termination Resistor]
                            (560Ω between differential pair if this is end node)
```

**LIN Bus Characteristics:**
- Single-wire, recessive-biased bus
- Twisted pair recommended for noise immunity
- Typical termination: 560Ω at ends (or specified by vehicle spec)
- Voltage levels: 0V (dominant) to ~12V (recessive, typical automotive)

## Breadboard Layout Example

```
Breadboard Row Assignments (if using 3.3V powered MCP2004):

VCC Rail (+3.3V):
├─ Pico 3V3 pin
├─ MCP2004 Pin 8
└─ 100nF cap (other end to GND)

GND Rail:
├─ Pico GND
├─ MCP2004 Pin 7
└─ 100nF cap (other end to VCC)

GPIO 0 → [1kΩ] → MCP2004 Pin 2
GPIO 1 → [1kΩ] → MCP2004 Pin 3
GPIO 2 → MCP2004 Pin 5
GPIO 3 → MCP2004 Pin 6

MCP2004 Pin 4 → LIN Bus (twisted pair to motors)
```

## Software Configuration

The firmware configures the Pico as follows:

```c
#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define MCP2004_EN_PIN 2
#define MCP2004_NSLP_PIN 3

// At startup:
gpio_put(MCP2004_EN_PIN, 1);      // Enable transceiver
gpio_put(MCP2004_NSLP_PIN, 1);    // Normal mode (not sleeping)
```

## LIN Frame Format (as sent by Pico)

```
[BREAK (13 bit times)] ─ [SYNC: 0x55] ─ [Payload bytes] ─ [Checksum] ─ [Inter-frame delay]
     ~1.35 ms                1 byte          N bytes         1 byte        ~1.5 ms
```

**Motor Command Example (Open Motor):**
```
Payload:  0xEC [addr] 0xD0 0x87 0xFF
Checksum: 0xFF - (0xEC + [addr] + 0xD0 + 0x87 + 0xFF) % 256
```

## Testing the Connection

### 1. Verify Power
- Measure 3.3V between MCP2004 Pin 8 (VCC) and Pin 7 (GND)
- Should be stable 3.2V – 3.4V

### 2. Verify UART
- Use serial monitor on Pico USB
- Should see boot message: `=== Pico LIN Motor Tester ===`

### 3. Verify LIN Bus
- Connect oscilloscope to MCP2004 Pin 4 (LIN+)
- Select motor test from menu
- Should see LIN frames (break pulse + data bytes)

### 4. Verify Motor Response
- Connect motor nodes to LIN bus
- After "Open all" command, should see motor position changes
- Serial monitor shows: `Motor [addr] stalled: pos=XXXX`

## Troubleshooting

| Symptom | Likely Cause | Check |
|---------|--------------|-------|
| No serial output | Pico not programmed or USB not recognized | Re-flash firmware; check USB cable |
| UART data garbled | Baud rate mismatch or GPIO pullup conflict | Verify uart_init() at 9600 baud |
| LIN bus no activity | EN or NSLP pin not asserted | Verify GPIO 2 & 3 are HIGH at startup |
| Motors not responding | LIN bus not connected or reversed | Check twisted pair continuity; verify polarity |
| Checksum errors | Wrong polynomial or byte order | Verify lin_checksum_payload() function |
| Timeout on responses | Motor nodes not powered or addressed | Verify motor power supply; check motor addresses |

## Component Bill of Materials (BOM)

| Component | Value | Qty | Notes |
|-----------|-------|-----|-------|
| MCP2004 DIP-8 | LIN Transceiver | 1 | Microchip part |
| Resistor | 1 kΩ 1/4W | 2 | For TX/RX lines |
| Capacitor | 100 nF (0.1 µF) | 1 | Decoupling (VCC bypass) |
| Twisted Pair Cable | — | as needed | For LIN bus to motors |
| Termination Resistor | 560 Ω (if needed) | 1 | For end-of-bus termination |

## References

- [MCP2004 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/20005330A.pdf)
- [LIN Specification](https://www.lin-cia.org/)
- [Raspberry Pi Pico Datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- Original firmware: Z-World RCM3700 LINBus Motor Tester (2003)

## Notes

- **Voltage compatibility**: The MCP2004 operates at 3.3V (Pico native voltage). No level shifting required.
- **LIN bus voltage**: MCP2004 output may reach 12V on the LIN bus (typical automotive), but input threshold is CMOS-compatible for 3.3V signals.
- **Termination**: Check your motor node documentation for whether termination is required. Some systems use 560Ω at both ends; others only at the farthest node.
- **Cable length**: Keep twisted pair as short as practical (< 10 meters typical for 9600 baud LIN).
