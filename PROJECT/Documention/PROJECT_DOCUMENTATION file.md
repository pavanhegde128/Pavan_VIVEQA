# Industrial Machine Runtime Monitor

### Team: K Nithin Shettigar \& Pavan Hegde

### Platform: Anmaya AT-STLN-ARTIX 7-001 (Xilinx XC7A35T) | Clock: 24 MHz

\---

## 1\. Project Overview

The Industrial Machine Runtime Monitor is an FPGA-based supervisory system that tracks the operational state of a simulated industrial machine — start/stop cycles, fault occurrences, total runtime, and maximum runtime — and exposes full remote monitoring and control through a Blynk IoT dashboard via an ESP32-C3 Wi-Fi bridge.

The system models a real machine control panel: physical slide switches and a push button drive the machine locally, while every operation can also be triggered remotely from a phone. A single relay output represents the machine's actual power contactor, and a 4-digit 7-segment display shows whichever metric (runtime, start count, stop count, fault count, current state, or max runtime) the operator has selected via mode switches.

The system supports:

* Local control via 6 physical slide switches (start, stop, fault, and 3 mode-select switches) and 1 push button (reset)
* Full remote control of all 6 switches and reset from a Blynk mobile dashboard, merged with the physical panel via OR logic
* A live serial status feed from the FPGA to the ESP32 describing state, active display mode, all four counters, the live display value, and the real relay output
* A single combined dashboard display that mirrors exactly what the physical 7-segment is showing, labelled by mode
* Relay output driving an external load (e.g. a lamp) through a Form-C relay module

\---

## 2\. Design and Architecture

### 2.1 System Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                     FPGA (AT-STLN-ARTIX 7-001)                     │
│                                                                      │
│  Physical switches ──┐                                              │
│  (sw0..sw5, rst)     │                                              │
│                      ▼                                              │
│              ┌───────────────┐        ┌─────────────────┐           │
│   UART RX ──▶│ remote\_cmd\_   │──OR──▶ │ machine\_runtime\_ │           │
│  (from ESP32)│   decoder     │        │   monitor (core) │           │
│              └───────────────┘        └─────────┬─────────┘          │
│                                                  │                   │
│                          ┌───────────────────────┼─────────┐         │
│                          ▼                       ▼         ▼         │
│                   relay / buzzer        7-seg (MAX7219)  state/     │
│                                                            counters  │
│                                                  │                   │
│                                                  ▼                   │
│                                        ┌───────────────────┐         │
│                                        │ status\_packet\_tx   │        │
│                                        │ (UART TX, 9 fields) │       │
│                                        └─────────┬───────────┘       │
└──────────────────────────────────────────────────┼───────────────────┘
                                                    │ PMOD (T2/R3)
                                                    ▼
                                          ┌───────────────────┐
                                          │   ESP32-C3-MINI-1   │
                                          │  (Blynk bridge)     │
                                          └─────────┬───────────┘
                                                    │ Wi-Fi
                                                    ▼
                                          ┌───────────────────┐
                                          │   Blynk Dashboard   │
                                          │ (mobile app)         │
                                          └───────────────────┘
```

### 2.2 PMOD / UART Link

|Signal|FPGA Net|FPGA Pin|Direction|Baud|
|-|-|-|-|-|
|esp\_uart\_tx|7SEG status packet out|T2 (PMOD IO\_0)|FPGA → ESP32|9600|
|esp\_uart\_rx|Remote command byte in|R3 (PMOD IO\_1)|ESP32 → FPGA|9600|

### 2.3 Operational Roles

**FPGA (core controller):**

* Runs the machine state machine: IDLE → RUNNING → FAULT/STOPPED
* Counts total runtime and tracks max runtime, in whole seconds
* Counts start, stop, and fault events
* Multiplexes one of six values onto the 7-segment display based on mode switches
* Merges physical switch state with remote commands via OR logic — either source can drive any input
* Streams a full status packet over UART roughly every 500ms

**ESP32-C3 (Blynk bridge):**

* Forwards single-byte commands from Blynk button/switch widgets to the FPGA
* Parses the FPGA's status packet and republishes a single combined display string to the dashboard
* Re-syncs momentary control widgets on reconnect

\---

## 3\. Implementation Approach

### 3.1 Switch Merging (Local + Remote)

Each of the six switches and the reset button has two possible sources: the physical panel switch, and a remote command byte decoded from UART. These are combined with a simple OR at the top-level module:

```verilog
wire sw0\_start\_eff = sw0\_start | remote\_start;
wire sw1\_stop\_eff  = sw1\_stop  | remote\_stop;
wire sw2\_fault\_eff = sw2\_fault | remote\_fault;
wire sw3\_mode0\_eff = sw3\_mode0 | remote\_mode0;
wire sw4\_mode1\_eff = sw4\_mode1 | remote\_mode1;
wire sw5\_mode2\_eff = sw5\_mode2 | remote\_mode2;
wire rst\_eff        = rst       | remote\_reset;
```

Start, stop, and reset are momentary (1-cycle pulses) on both the physical and remote side. Fault and the three mode switches are level signals on the panel, so their remote equivalents are **toggle latches** — one command byte flips the latch, mimicking a slide switch from the app.

### 3.2 Remote Command Protocol

A single ASCII byte per command, decoded by `remote\_cmd\_decoder.v`:

|Byte|Action|Type|
|-|-|-|
|`'1'`|Start|1-cycle pulse|
|`'0'`|Stop|1-cycle pulse|
|`'F'`|Fault toggle|Level latch|
|`'A'`|Mode bit 0 toggle|Level latch|
|`'B'`|Mode bit 1 toggle|Level latch|
|`'C'`|Mode bit 2 toggle|Level latch|
|`'R'`|Reset|1-cycle pulse|

### 3.3 Status Packet Format

`status\_packet\_tx.v` builds and transmits an ASCII line once per `send\_tick` (≈500ms):

```
S,<state>,<mode>,<runtime>,<start>,<stop>,<fault>,<maxrt>,<disp>,<relay>\\n
```

|Field|Meaning|
|-|-|
|state|0=IDLE, 1=RUNNING, 2=FAULT, 3=STOPPED|
|mode|0=runtime, 1=start count, 2=stop count, 3=fault count, 4=state, 5=max runtime — selected by sw3/4/5|
|runtime|Total runtime, whole seconds|
|start|Start event count|
|stop|Stop event count|
|fault|Fault event count|
|maxrt|Max runtime recorded, whole seconds|
|disp|Exact 16-bit value currently shown on the 7-segment display|
|relay|Actual relay output pin state (1 = energized)|

The `mode` field lets the ESP32 bridge know whether `disp` is a time value (modes 0 and 5, convert seconds → ms) or a plain count/state (modes 1–4, no conversion).

### 3.4 Display Mode Multiplexing

```verilog
wire \[2:0] mode = {sw5\_mode2, sw4\_mode1, sw3\_mode0};
case (mode)
    3'b000:  display\_data = runtime\_count\[15:0];
    3'b001:  display\_data = start\_count\[15:0];
    3'b010:  display\_data = stop\_count\[15:0];
    3'b011:  display\_data = fault\_count\[15:0];
    3'b100:  display\_data = {14'd0, state};
    3'b101:  display\_data = max\_runtime\[15:0];
endcase
```

### 3.5 Dashboard Display Consolidation

Rather than one Blynk label per counter, the ESP32 bridge publishes a single combined string to one virtual pin, mirroring exactly what's on the physical 7-segment, with a label identifying which metric is active:

```cpp
const char\* modeLabel;
switch (mode) {
  case 0: modeLabel = "RUNTIME (ms)";     break;
  case 1: modeLabel = "START COUNT";      break;
  case 2: modeLabel = "STOP COUNT";       break;
  case 3: modeLabel = "FAULT COUNT";      break;
  case 4: modeLabel = "STATE";            break;
  case 5: modeLabel = "MAX RUNTIME (ms)"; break;
}
snprintf(segBuf, sizeof(segBuf), "%s: %ld", modeLabel, segMs);
Blynk.virtualWrite(VPIN\_SEVEN\_SEG, segBuf);
```

\---

## 4\. Module Descriptions

### 4.1 `machine\_runtime\_monitor\_top.v` — Top-Level Module

Integrates the remote command decoder, the core monitor, and the status packet transmitter. Merges physical and remote switch sources via OR logic and drives a single effective reset (`rst\_eff`) into every sub-block so a remote reset behaves identically to the physical push button.

|Port|Direction|Description|
|-|-|-|
|clk\_24mhz|Input|24 MHz system clock|
|rst|Input|Physical push button 0, active HIGH|
|sw0\_start – sw5\_mode2|Input|6 physical slide switches|
|relay|Output|Drives relay coil transistor|
|buzzer|Output|Drives piezo buzzer transistor|
|seg\_din/cs/clk|Output|MAX7219 SPI interface|
|esp\_uart\_tx|Output|Status packet to ESP32|
|esp\_uart\_rx|Input|Remote command byte from ESP32|

### 4.2 `machine\_runtime\_monitor-1.v` — Core State Machine

Implements the IDLE/RUNNING/FAULT/STOPPED state machine, all four event counters, and the display mode multiplexer. Outputs observation signals (`state\_out`, `mode\_out`, `runtime\_out`, etc.) for the status packet transmitter to read.

|Parameter|Value|
|-|-|
|Counter width|32 bits|
|Display value width|16 bits|
|States|IDLE(0), RUNNING(1), FAULT(2), STOPPED(3)|
|Display modes|0–5 (runtime, start, stop, fault, state, max runtime)|

### 4.3 `remote\_cmd\_decoder.v` — Remote Command Decoder

Decodes single ASCII command bytes received over UART into pulse or toggle-latch outputs for every switch and reset. See protocol table in section 3.2.

### 4.4 `status\_packet\_tx.v` — Status Packet Transmitter

A flat FSM that serializes 9 comma-separated ASCII fields plus a newline once per `send\_tick` pulse, transmitted byte-by-byte through `uart\_tx`. See packet format in section 3.3.

### 4.5 `uart\_rx.v` / `uart\_tx.v` — UART Primitives (Shared)

Standard 8N1 byte-level UART receiver and transmitter, parameterized by `CLK\_FREQ` and `BAUD\_RATE` (24 MHz / 9600 baud in this design).

### 4.6 `bin\_to\_bcd.v` — Binary to BCD Converter

Converts a 16-bit binary display value into 4 BCD digits for the MAX7219 driver, supporting values up to 9999 before truncation.

### 4.7 `max7219\_driver.v` — 7-Segment Display Driver

Drives the onboard 4-digit 7-segment display via SPI to a MAX7219 controller chip.

### 4.8 `esp32\_blynk\_bridge.ino` — ESP32 Blynk Bridge (Arduino/ESP32)

Bridges the FPGA's UART status feed to Blynk virtual pins, and forwards Blynk widget events back to the FPGA as single-byte commands. Parses the 9-field status packet, builds the combined display string (section 3.5), and prints a full debug breakdown of every parsed field over USB serial for verification.

|Virtual Pin|Direction|Widget|Purpose|
|-|-|-|-|
|V6|App → FPGA|Button|Start (`'1'`)|
|V7|App → FPGA|Button|Stop (`'0'`)|
|V9|FPGA → App|Labeled String Display|Combined 7-segment mirror|
|V10|App → FPGA|Switch|Fault toggle (`'F'`)|
|V11|App → FPGA|Switch|Mode bit 0 toggle (`'A'`)|
|V12|App → FPGA|Switch|Mode bit 1 toggle (`'B'`)|
|V13|App → FPGA|Switch|Mode bit 2 toggle (`'C'`)|
|V14|App → FPGA|Button|Reset (`'R'`)|

### 4.9 `tb\_uart\_link.v` — Testbench

Exercises `status\_packet\_tx` with known field values and `remote\_cmd\_decoder` against all 7 command bytes, verifying pulses fire and toggle latches flip correctly in simulation before hardware deployment.

\---

## 5\. Build and Run Instructions

### 5.1 Requirements

* Xilinx Vivado 2020.x or later
* Anmaya AT-STLN-ARTIX 7-001 board
* ESP32-C3-MINI-1 (onboard) with Arduino IDE configured for ESP32-C3
* Blynk account with a configured template/device matching the virtual pin table above
* Jumper wires for PMOD ↔ ESP32 UART link
* 15V DC power supply
* USB-C cable for FPGA programming, USB-C for ESP32 programming

### 5.2 Project Setup in Vivado

1. Create new Vivado project → RTL Project
2. Add sources: `machine\_runtime\_monitor\_top.v`, `machine\_runtime\_monitor-1.v`, `remote\_cmd\_decoder.v`, `status\_packet\_tx.v`, `uart\_rx.v`, `uart\_tx.v`, `bin\_to\_bcd.v`, `max7219\_driver.v`
3. Set top module: `machine\_runtime\_monitor\_top`
4. Add constraints: `machine\_runtime\_monitor\_top.xdc`
5. Run Synthesis → Implementation → Generate Bitstream
6. Program via Hardware Manager → USB-C on J1

### 5.3 ESP32 Setup in Arduino IDE

1. Install ESP32 board support and the Blynk library
2. Open `esp32\_blynk\_bridge.ino`
3. Set Wi-Fi SSID/password and Blynk auth token
4. Create matching datastreams V6, V7, V9 (String type), V10–V14 in the Blynk console
5. Build the dashboard layout: Start/Stop buttons, Fault/Mode0/Mode1/Mode2 switches, Reset button, one labeled string display for V9
6. Upload to the ESP32-C3 module

### 5.4 Hardware Setup

1. Connect 15V DC power to J2; verify all power rail LEDs illuminate
2. Set configuration switches: SW17=OFF, SW36=OFF, SW37=ON (Master SPI mode)
3. Jumper PMOD IO\_0 (T2) → ESP32 GPIO RX pin, PMOD IO\_1 (R3) → ESP32 GPIO TX pin
4. Connect relay output (J17 screw terminal: NO/COM/NC) to the desired load, e.g. a lamp, via COM + NO for fail-safe-off behavior
5. Power on the board and confirm the 7-segment display is active

### 5.5 Switch Configuration

|Switch|Pin|Function|
|-|-|-|
|sw0\_start|C9|Start machine (momentary)|
|sw1\_stop|B9|Stop machine (momentary)|
|sw2\_fault|G5|Fault toggle|
|sw3\_mode0|A7|Display mode bit 0|
|sw4\_mode1|C7|Display mode bit 1|
|sw5\_mode2|A10|Display mode bit 2|
|rst|A13|Reset (push button)|

\---

## 6\. Testing Instructions

### 6.1 Local Control Test

1. Power on the board — display should show 0 in runtime mode (default)
2. Flip sw0\_start — state changes to RUNNING, relay energizes
3. Flip sw1\_stop — state changes to STOPPED, relay de-energizes
4. Cycle sw3/sw4/sw5 mode switches and confirm the display shows runtime, start count, stop count, fault count, state, and max runtime in turn

### 6.2 Remote Control Test

1. Open the Blynk dashboard, confirm connection indicator is active
2. Tap Start Button — machine starts remotely, relay energizes, physical board confirms RUNNING
3. Tap Stop Button — machine stops remotely
4. Flip the Fault switch widget — fault state toggles on the FPGA
5. Flip Mode0/Mode1/Mode2 widgets — dashboard display label and value change to match whichever metric is now selected
6. Tap Reset — FPGA resets exactly as if the physical push button were pressed

### 6.3 Status Feed Verification

1. Open the Arduino serial monitor at 115200 baud
2. Confirm `\[RX]` lines appear roughly every 500ms with a full field breakdown (`state=`, `mode=`, `seg\_ms=`, `relay=`, etc.)
3. Compare `disp\_raw` against the physical 7-segment digits — they should match exactly
4. Confirm `relay=` flips in lockstep with the physical relay click, not lagging behind state

### 6.4 Relay Load Test

1. Wire a lamp to J17 via COM + NO
2. Start the machine (locally or remotely) — lamp should turn on as the relay energizes
3. Stop the machine — lamp should turn off

\---

## 7\. Known Constraints

* Display values are limited to 4 BCD digits (0–9999); larger values truncate on the physical 7-segment and in the dashboard mirror
* Only modes 0 (runtime) and 5 (max runtime) are time-based and converted to milliseconds on the dashboard; all other modes show the raw count/state value unconverted
* Fault and mode switches are toggle latches when triggered remotely — Blynk switch widgets are not re-synced on reconnect, since re-sending the last known position would incorrectly re-toggle the latch
* The relay is rated 10A @ 250VAC / 10A @ 30VDC — any externally connected load must stay within this limit
* All remote and physical switch sources are OR'd together; there is currently no way to disable physical panel input while under remote control

