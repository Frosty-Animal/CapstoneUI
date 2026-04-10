# Scan-to-Mill — ClearCore ↔ Raspberry Pi Comms Test

A standalone test rig that validates the Modbus TCP communication path between
the Raspberry Pi operator interface and the Teknic ClearCore motion controller
in the OSU Scan-to-Mill capstone project. When this works end-to-end, you have
proof that your network, framing, register layout, command dispatch, and
status readback are all healthy — which is the foundation everything else in
the system depends on.

This README is split into two paths:

- **Quick Start** — for someone who has the hardware in front of them and
  just wants to bring up the test in the next 30 minutes.
- **Deep Dive** — architecture, register map, debugging procedures, and
  rationale for the design choices.

---

## What This Project Is

The Scan-to-Mill capstone integrates a depth camera, a motion-controlled scan
carriage, and a CNC router into one workflow: scan a part, generate a
toolpath, mill a copy. There are roughly four subsystems:

1. **Capture** — Intel RealSense D405 depth camera mounted on a motorized arc
   carriage, scanning the part from multiple angles.
2. **Motion** — Teknic ClearCore PLC driving a NEMA17 stepper through a TB6600
   driver, moving the camera carriage along the arc track.
3. **Operator interface** — PyQt6/PyVista UI on a Raspberry Pi 5 that shows
   the live point cloud, controls the scan, and exports G-code.
4. **Machining** — Genmitsu 3020 Pro Max V2 CNC router that consumes the
   exported G-code.

The motion subsystem (the ClearCore) and the operator interface (the Pi) are
separate computers connected by an Ethernet cable. **They speak Modbus TCP to
each other.** This repository contains the test harness that proves that link
works before you try to run any of the actual scan logic on top of it.

If the comms test passes, you can be confident that:

- The Pi can reach the ClearCore over the wire
- The ClearCore is running its sketch and listening on port 502
- Commands sent from the Pi reach the ClearCore and trigger the right actions
- Status read by the Pi accurately reflects ClearCore state
- The 32-bit numeric encoding agrees on both sides
- The MBAP framing and Modbus function codes are correctly implemented

If it fails, you have a layered diagnostic tool that tells you exactly which
of those things is broken.

---

## Hardware Bill of Materials

| Component | Notes |
|---|---|
| Raspberry Pi 5 (4 GB or 8 GB) | Running Raspberry Pi OS Bookworm or Trixie |
| Official 7" Raspberry Pi Touchscreen | Or any HDMI display ≥ 1280×800 |
| Teknic ClearCore (CLCR-4-13) | The motion controller |
| ClearPath-SD or generic stepper | M-0 connector, step-and-direction mode |
| TB6600 stepper driver | If using a generic stepper instead of ClearPath |
| Ethernet cable, Cat5e or better | Direct Pi → ClearCore, no switch required |
| ClearCore 24V power supply | Required for motion |
| USB-C power for Pi 5 | 5.1V 5A official supply recommended |
| USB-C cable, Pi to ClearCore | For flashing the ClearCore and reading its serial debug log |

You can run the comms test without the motor connected — the ClearCore will
boot, log "M-0 enabled", and report "no motion" status, but the Modbus link
itself will still come up. This is useful for software-only debugging.

---

## Quick Start

These steps assume you have the hardware listed above, fresh OS images on
both devices, and roughly 30 minutes. For the why and how behind each step,
see the Deep Dive sections below.

### 1. Flash the Raspberry Pi (if not already done)

Download Raspberry Pi Imager, flash Raspberry Pi OS (64-bit, Desktop) onto
a microSD card. Boot the Pi, complete first-run setup, connect to WiFi for
package downloads.

### 2. Install dependencies on the Pi

```bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-pip libgl1 libglib2.0-0 screen
pip install pyvista pyvistaqt pymodbus --break-system-packages
```

The `--break-system-packages` flag is required because Raspberry Pi OS marks
the system Python as externally-managed (PEP 668). It does NOT actually
break anything — it just installs to your user site-packages.

### 3. Configure the Pi's Ethernet for the comms link

The Pi's `eth0` interface has no IP address by default. Give it a static one:

```bash
sudo nmcli connection modify netplan-eth0 ipv4.method manual ipv4.addresses 192.168.1.10/24 ipv4.gateway "" ipv6.method ignore
sudo nmcli connection down netplan-eth0
sudo nmcli connection up netplan-eth0
```

Verify with `ip addr show eth0` — you should see `inet 192.168.1.10/24`.

If your eth0 connection has a different name, run `nmcli connection show`
first and substitute the actual name in the commands above.

### 4. Flash the ClearCore sketch

Open `ClearCoreModbusTest.ino` in the Arduino IDE with the Teknic ClearCore
board package installed (Tools → Board → Boards Manager → search "ClearCore").
Make sure the static IP at the top of the file matches what the Pi expects:

```cpp
IPAddress STATIC_IP (192, 168, 1, 20);
```

Compile and upload. Open the Arduino Serial Monitor at 9600 baud and verify
you see the boot sequence:

```
[BOOT] Scan-to-Mill ClearCore Modbus TCP test
[BOOT] CLIENT_INFC = 88 bytes = 44 regs, cmd@w6
[MOTOR] M-0 enabled, step-and-dir
[NET] PHY link up
[NET] IP = 192.168.1.20
[NET] Modbus TCP listening on :502 unit 1
[SEQ1] started
```

If the motor is connected, it should immediately start moving back and forth
between +5000 and −5000 steps. This auto-start is intentional — if you see
motion before the Pi is even connected, you know the motor and motion logic
are healthy independent of the comms layer.

### 5. Connect the cable and run the test

Plug an Ethernet cable directly from the Pi's RJ-45 jack to the ClearCore's
ETHERNET port. No router or switch needed. Then on the Pi:

```bash
cd ~/CapstoneUI
python3 cc_modbus_test.py
```

Expected output:

```
ClearCore Modbus TCP comms test → 192.168.1.20:502 unit 1

[1/3] TCP layer: can we open a socket to 192.168.1.20:502?
  ✓ TCP connect to 192.168.1.20:502 succeeded

[2/3] Modbus layer: pymodbus client open
  ✓ pymodbus connected

[3/3] Protocol layer: read holding register 0 (unit 1)
  ✓ Read OK: HR0 = 100000

All three layers OK. Comms link is healthy.

Current ClearCore status:
  cur_posn  :    +5000 steps
  state     : RUNNING
  status    : 0x0023  READY MOVING SCANNING
  msg_cnt   : 3
  msg       : SEQ1 RUNNING

cc>
```

You're now in the interactive shell. Type `watch` to see the motor position
flip back and forth in real time, then Ctrl+C and type `stop` to halt the
motor. Type `q` to exit.

If you got here, **everything works**. The rest of this README is for when
something doesn't, or when you want to understand what you just did.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Raspberry Pi 5                                │
│                                                                      │
│   ┌──────────────┐    ┌──────────────────┐    ┌─────────────────┐  │
│   │   UI.py      │    │ cc_modbus_test.py│    │  (future)       │  │
│   │  (PyQt6 +    │    │  (CLI tester)    │    │  Camera capture │  │
│   │   PyVista)   │    │                  │    │  pipeline       │  │
│   └──────┬───────┘    └────────┬─────────┘    └─────────────────┘  │
│          │                     │                                     │
│          └─────────┬───────────┘                                     │
│                    │                                                 │
│           ┌────────▼─────────┐                                       │
│           │   pymodbus       │                                       │
│           │   ModbusTcpClient│                                       │
│           └────────┬─────────┘                                       │
│                    │ TCP/IP                                          │
│                    │                                                 │
│              ┌─────▼──────┐                                          │
│              │   eth0     │ 192.168.1.10                             │
│              │  (RJ-45)   │                                          │
│              └─────┬──────┘                                          │
└────────────────────┼─────────────────────────────────────────────────┘
                     │
               Ethernet cable
                     │
┌────────────────────┼─────────────────────────────────────────────────┐
│              ┌─────▼──────┐                                          │
│              │  ETHERNET  │ 192.168.1.20                             │
│              │   port     │                                          │
│              └─────┬──────┘                                          │
│                    │                                                 │
│           ┌────────▼─────────┐                                       │
│           │  EthernetServer  │ port 502                              │
│           │  (Arduino API)   │                                       │
│           └────────┬─────────┘                                       │
│                    │                                                 │
│           ┌────────▼─────────┐                                       │
│           │  MBAP parser     │                                       │
│           │  (hand-rolled)   │                                       │
│           └────────┬─────────┘                                       │
│                    │                                                 │
│           ┌────────▼─────────┐                                       │
│           │  CLIENT_INFC     │  ← single source of truth for         │
│           │  shared struct   │     register layout                   │
│           └────────┬─────────┘                                       │
│                    │                                                 │
│           ┌────────▼─────────┐    ┌────────────────┐                 │
│           │  Motion state    │───▶│  ConnectorM0   │                 │
│           │  machine         │    │  (StepGen)     │                 │
│           └──────────────────┘    └────────────────┘                 │
│                                                                      │
│                       Teknic ClearCore                               │
└──────────────────────────────────────────────────────────────────────┘
```

The Pi acts as the Modbus **master** (client). The ClearCore acts as the
Modbus **slave** (server). The Pi periodically polls the ClearCore for
status, and writes commands when the user clicks a button. There is no
unsolicited data flow from the ClearCore — every byte the Pi reads was
requested by the Pi.

### Why Modbus TCP and not something else

Modbus TCP is well-supported on both sides (pymodbus on Python, multiple
libraries on ClearCore), it's a well-established industrial protocol, and
the hand-rolled server on the ClearCore side is small enough (~150 lines)
to audit completely. Alternatives considered and rejected:

- **Raw TCP with custom protocol** — would require designing and debugging
  a framing layer from scratch, no library support
- **Modbus RTU over RS-485** — would work but requires RS-485 transceivers
  on both ends, an extra USB-RS485 adapter on the Pi side, and adds wires
- **OPC UA** — heavyweight, ClearCore library support is immature
- **MQTT** — fundamentally pub/sub, not request/response, awkward fit for
  command/status semantics

Modbus TCP gives request/response semantics natively, sits directly on
TCP/IP with no extra hardware, and the protocol is dead simple to debug
with Wireshark or even `nc` if you really need to.

### Why TCP and not the original Serial RTU plan

An earlier iteration of this project used Modbus RTU over the ClearCore's
COM-0 serial port via a USB-RS485 adapter on the Pi. That works fine, but
the test rig already had Ethernet ports on both devices and direct Ethernet
is cleaner: no adapter, no driver dependencies on the Pi for the serial
device, lower latency, and no risk of corrupted UART frames at high baud
rates. The protocol-level register map and command dispatch is identical
between RTU and TCP — only the transport layer differs.

---

## Detailed Setup — Raspberry Pi

This section assumes Raspberry Pi OS Bookworm or Trixie on a Pi 5. Other
Pi models and other Linux distros will work, but commands and package names
may differ.

### Installing Python dependencies

The project needs:

- **PyQt5 or PyQt6** — UI framework. The code prefers PyQt6 and falls back
  to PyQt5 automatically. On the Pi, install PyQt5 via apt because PyQt6
  doesn't have a prebuilt wheel for ARM and pip would try to compile it
  from source (slow and frequently fails).
- **PyVista + pyvistaqt** — 3D visualization for the point cloud viewport
- **pymodbus** — Modbus client library
- **numpy** — implied by PyVista
- **screen** — for reading the ClearCore's USB serial debug log

Install order (this is the right sequence for a fresh Pi):

```bash
sudo apt update
sudo apt install -y python3-pyqt5 python3-pip libgl1 libglib2.0-0 screen
pip install pyvista pyvistaqt pymodbus --break-system-packages
```

Verify everything imports cleanly before going further:

```bash
python3 -c "import pyvista, pyvistaqt, PyQt5, numpy, pymodbus; print('all good')"
```

If that prints `all good`, your Python environment is ready.

### Network configuration

This is the part that bites everyone. **Raspberry Pi OS does not assign
an IP address to `eth0` by default.** It assumes you'll either plug into
a router (which would hand out DHCP) or configure it manually. For a
direct Pi-to-ClearCore cable, neither happens automatically.

You can confirm this is the issue by running `ip addr show eth0` and
looking for an `inet` line. If you only see `inet6 fe80::...` (the IPv6
link-local address that always exists on a live link), the interface has
no IPv4 address and can't reach the ClearCore.

The fix is to configure a static IP on `eth0` using NetworkManager:

```bash
nmcli connection show
```

Find the connection name for `eth0`. On Bookworm/Trixie this is typically
`netplan-eth0` (because netplan generates it during install). Then:

```bash
sudo nmcli connection modify netplan-eth0 ipv4.method manual ipv4.addresses 192.168.1.10/24 ipv4.gateway "" ipv6.method ignore
sudo nmcli connection down netplan-eth0
sudo nmcli connection up netplan-eth0
```

The empty `ipv4.gateway ""` is intentional and important. Setting it would
tell the Pi to route default traffic through the ClearCore, which has no
internet connection. With no gateway on this interface, the Pi only uses
`eth0` for traffic destined for `192.168.1.0/24` and continues using
`wlan0` for everything else, including internet access.

Verify:

```bash
ip addr show eth0       # should show inet 192.168.1.10/24
ping -c 4 192.168.1.20  # only works if the ClearCore is also up
```

This config persists across reboots.

### Display setup (Pi 5 + Wayland)

The Pi 5 on Bookworm/Trixie defaults to Wayland (Wayfire compositor), which
PyVista's VTK rendering doesn't always handle gracefully. If you launch
`UI.py` and get an error like `BadWindow (invalid Window parameter)` from
the X server, force Qt to use the X11 platform plugin via XWayland:

```bash
QT_QPA_PLATFORM=xcb python3 UI.py
```

To make this permanent:

```bash
echo 'export QT_QPA_PLATFORM=xcb' >> ~/.bashrc
source ~/.bashrc
```

Note: the current UI layout was designed for a 1280×800 monitor minimum
and is unusably squished on the official 7" Pi touchscreen (800×480).
This is a known issue documented in "Future Work" below.

---

## Detailed Setup — ClearCore

### Installing the ClearCore Arduino board package

1. Open the Arduino IDE (1.8.x or 2.x)
2. File → Preferences → Additional Boards Manager URLs → add:
   `https://raw.githubusercontent.com/Teknic-Inc/ClearCore-Arduino-wrapper/master/package_clearcore_index.json`
3. Tools → Board → Boards Manager → search "ClearCore" → Install
4. Tools → Board → Teknic → ClearCore

### Important: which ClearCore API are you using?

There are two ClearCore programming environments and they use different
APIs even though they target the same hardware:

- **Native C++ library** (Microchip Studio / Atmel Studio) — uses
  `EthernetTcpServer`, `EthernetTcpClient`, `IpAddress`, `EthernetMgr`,
  `ConnectorUsb.SendLine()`, etc. All under the `ClearCore::` namespace.
- **Arduino wrapper** (Arduino IDE) — uses standard Arduino names:
  `EthernetServer`, `EthernetClient`, `IPAddress`, `Ethernet.begin()`,
  `Serial.println()`, etc.

`ClearCoreModbusTest.ino` uses the **Arduino wrapper API**. If you see
compile errors like "EthernetTcpServer does not name a type", you're
either trying to compile this sketch in the wrong environment, or you've
mixed up the two APIs in your edits. Stick to the Arduino names.

### Configuring the static IP

Open `ClearCoreModbusTest.ino` and find the network config section near
the top:

```cpp
IPAddress STATIC_IP (192, 168, 1, 20);
IPAddress NETMASK   (255, 255, 255, 0);
IPAddress GATEWAY   (192, 168, 1, 1);
IPAddress DNS_IP    (192, 168, 1, 1);
```

These must match what the Pi side is configured to expect. If you change
the IP here, also change `MODBUS_DEFAULT_HOST` in `UI.py` and `DEFAULT_HOST`
in `cc_modbus_test.py`.

### Wiring

Direct Ethernet cable from the ClearCore's RJ-45 ETHERNET jack to the Pi's
Ethernet jack. No switch, no router. Modern RJ-45 hardware on both ends has
auto-MDIX so a regular cable works fine; you do not need a crossover cable.

For motion (optional for the comms test):

- Stepper or ClearPath-SD on **M-0** connector
- ClearCore powered with 24V on the COM/PWR terminals
- For TB6600 + generic stepper: tie M-0 STEP and DIR outputs to TB6600 PUL
  and DIR inputs respectively, with a common ground

### Verifying the ClearCore is alive

After flashing, plug the ClearCore's USB into any computer (Pi or laptop),
then open the Arduino Serial Monitor at 9600 baud. You should see the boot
sequence shown earlier in the Quick Start.

On the Pi, you can use `screen` instead:

```bash
ls /dev/ttyACM*               # find which port the ClearCore is on
sudo screen /dev/ttyACM0 9600 # connect at 9600 baud
```

Press the ClearCore's reset button to see a fresh boot. Exit screen with
`Ctrl+A` then `\` then `y`.

The critical line in the boot log is `[NET] IP = 192.168.1.20`. If that
shows a different address, the static IP didn't take effect — usually
because you uploaded an older sketch by mistake. Re-flash and re-check.

---

## Verifying the Comms Link

`cc_modbus_test.py` is the standalone test tool. It walks through three
layers in order and tells you exactly which one fails:

1. **TCP layer** — raw `socket()` connect to 192.168.1.20:502, no
   pymodbus involved. Failure here means a network problem.
2. **Modbus client open** — pymodbus wrapping the socket. Failure here
   is unusual and usually indicates a pymodbus version issue.
3. **Modbus protocol** — one round-trip read of HR0. Failure here means
   the ClearCore is reachable but the protocol-level conversation isn't
   working — wrong unit ID, malformed framing, etc.

After all three layers pass, the script drops into an interactive shell:

```
cc> status        # one-shot read of cur_posn, status flags, state, msg
cc> watch         # 4 Hz live updates until Ctrl+C
cc> stop          # send CCMD_STOP — verify motor halts
cc> run           # send CCMD_RUN_1 — restart sequence 1
cc> enab          # CCMD_ENAB_MTRS — enable motor drive
cc> disab         # CCMD_DISAB_MTRS — disable drive (software estop)
cc> move 5000     # CCMD_MOVE to absolute position 5000 steps
cc> raw 6         # read any single register by word address
cc> q             # quit
```

The `watch` command is the most useful for verifying everything works.
You should see `cur_posn` flipping between +5000 and −5000 every couple
of seconds, the `SCANNING` status flag set, and the `msg` field showing
ClearCore-side events as they happen.

For automation or scripts, run with `--once` to read status once and
exit without entering the shell:

```bash
python3 cc_modbus_test.py --once
```

---

## Running the Full UI

Once the comms test confirms the link works, you can launch the operator
interface:

```bash
QT_QPA_PLATFORM=xcb python3 UI.py
```

The UI brings up a PyQt6 window with three panels: scan controls on the
left, the 3D point cloud viewport in the middle, and status/log on the
right. The Modbus link to the ClearCore is established automatically at
startup — watch the log panel for the `[MODBUS] ClearCore link
ESTABLISHED` line.

**Known issue:** the current UI layout assumes a desktop-class display
(≥1280×800). On the official 7" Pi touchscreen (800×480) it's unusably
small. This is documented in Future Work below.

---

## Register Map Reference

The contract between the Pi and the ClearCore is the `CLIENT_INFC` struct
defined in `ClearCoreModbusTest.ino`. The Modbus register address of any
field is `offsetof(field) / 2` (i.e. byte offset in the packed struct,
divided by two because Modbus addresses are word-based).

| Word | Field | Type | Direction | Description |
|---:|---|---|---|---|
| 0–1 | `acc` | uint32 | Pi → CC | Acceleration, steps/sec² |
| 2–3 | `vel` | int32 | Pi → CC | Max velocity, steps/sec |
| 4–5 | `target_posn` | int32 | Pi → CC | Move target, absolute steps |
| 6 | `cmd` | uint16 | Pi → CC | Command code (see below) |
| 7–8 | `cur_posn` | int32 | CC → Pi | Current position, steps |
| 9 | `status` | uint16 | CC → Pi | Status bitfield (see below) |
| 10 | `msg_cnt` | uint16 | CC → Pi | Increments on each new debug msg |
| 11–42 | `msg` | char[64] | CC → Pi | Debug message text |
| 43 | `state` | uint16 | CC → Pi | State machine state (see below) |

Total: 44 holding registers (88 bytes).

### Command codes (`CCMTR_CMD`, written to word 6)

| Value | Name | Effect |
|---:|---|---|
| 0 | `CCMD_NONE` | No-op |
| 1 | `CCMD_ENAB_MTRS` | Enable motor drive |
| 2 | `CCMD_DISAB_MTRS` | Disable motor drive (software estop) |
| 3 | `CCMD_SET_ZERO` | Zero current position (not implemented in test) |
| 4 | `CCMD_MOVE` | Absolute move using acc/vel/target_posn |
| 5 | `CCMD_STOP` | Abrupt stop, drive stays enabled |
| 6 | `CCMD_RUN_1` | Start/restart sequence 1 (back-and-forth test) |
| 7 | `CCMD_ACK` | Reserved for future use |
| 8 | `CCMD_NACK` | Returned by ClearCore on unknown command |

### Status bits (`CCMTR_STATUS`, word 9)

| Bit | Mask | Name | Meaning |
|---:|---:|---|---|
| 0 | 0x01 | `READY` | Drive enabled |
| 1 | 0x02 | `MOVING` | Motion in progress |
| 2 | 0x04 | `HOMED` | Homing complete (unused in test) |
| 3 | 0x08 | `FAULT` | Fault latched |
| 4 | 0x10 | `ESTOP` | E-stop latched |
| 5 | 0x20 | `SCANNING` | Sequence 1 running |
| 6 | 0x40 | `HLFB` | ClearPath HLFB OK (unused in test) |

### State codes (`CCMTR_STATE`, word 43)

| Value | Name |
|---:|---|
| 0 | `INIT` |
| 1 | `IDLE` |
| 2 | `ENABLED` |
| 3 | `RUNNING` |
| 4 | `STOPPED` |
| 5 | `FAULT` |

### Endianness — important

ClearCore is little-endian ARM Cortex-M4. The `CLIENT_INFC` struct is
accessed via `reinterpret_cast<uint16_t*>` on the ClearCore side, so
32-bit fields appear on the wire **low-word-first**. For example, the
value `0x12345678` in `acc` is transmitted as register pair
`[0x5678, 0x1234]` (low word at the lower address).

The Pi side handles this correctly via `pack_u32()` / `pack_i32()` /
`unpack_i32()` helpers in both `UI.py` and `cc_modbus_test.py`.
**Do not bypass these helpers** for new 32-bit fields, or you'll get
mysterious value corruption that's painful to debug.

---

## Common Errors and Fixes

These are the things that actually went wrong during initial bring-up.
If you hit any of them, the fix is here. They're roughly ordered by
how early in the workflow you'll hit them.

### `error: externally-managed-environment` when running pip

Add `--break-system-packages` to your pip command:

```bash
pip install pymodbus --break-system-packages
```

Raspberry Pi OS marks the system Python as externally-managed under PEP
668. The flag installs to your user site-packages instead of the system
site-packages, which is safe.

### `'EthernetTcpServer' does not name a type` when compiling the ClearCore sketch

You're trying to compile the Arduino-API sketch using native ClearCore
library APIs. Make sure:

- You're using the Arduino IDE, not Microchip Studio or Atmel Studio
- The Tools → Board menu has "Teknic ClearCore" selected
- You haven't pasted in any code from the native library examples

The Arduino API uses `EthernetServer`, `EthernetClient`, `Ethernet.begin()`,
not `EthernetTcpServer`/`EthernetTcpClient`/`EthernetMgr.Setup()`.

### `BadWindow (invalid Window parameter)` when launching UI.py on Pi 5

Force Qt to use the X11 backend via XWayland:

```bash
QT_QPA_PLATFORM=xcb python3 UI.py
```

Pi 5 on Bookworm/Trixie defaults to Wayland, which PyVista's VTK
rendering doesn't always handle. XWayland is the workaround until VTK
gets better Wayland support.

### `[MODBUS] Failed to reach ClearCore at 192.168.1.20:502` even though everything is plugged in

Almost certainly the Pi's `eth0` doesn't have an IPv4 address. Run:

```bash
ip addr show eth0
```

If you only see an `inet6` line and no `inet` line, configure a static IP:

```bash
sudo nmcli connection modify netplan-eth0 ipv4.method manual ipv4.addresses 192.168.1.10/24 ipv4.gateway "" ipv6.method ignore
sudo nmcli connection down netplan-eth0
sudo nmcli connection up netplan-eth0
```

If your eth0 connection has a different name, find it with `nmcli
connection show` and substitute.

### `From <campus-ip> Destination Net Unreachable` when pinging the ClearCore

Same root cause as above. The Pi has no route to 192.168.1.0/24, so it
sent your ping out over WiFi (`wlan0`), the campus router got it, looked
in its routing table, didn't find anything, and bounced an unreachable
back. The fix is the same nmcli command — give `eth0` an IP on the
192.168.1.0/24 subnet.

### Sketch compiles and uploads but the ClearCore boot log shows the wrong IP

The `STATIC_IP` constant near the top of the .ino file didn't get changed,
or you uploaded an older version of the sketch. Verify the constant matches
what UI.py expects, then re-flash.

### Sketch boots fine but the motor doesn't move

This is a motion problem, not a comms problem. Check in this order:

1. ClearCore 24V power LED is on
2. Stepper driver power LED is on
3. M-0 connector wiring is correct (STEP, DIR, GND)
4. Motor isn't physically jammed
5. The serial log shows `[MOTOR] M-0 enabled, step-and-dir`
6. Sequence 1 actually started — look for `[SEQ1] started` in the log

If all of the above are healthy and the motor still doesn't move, check
the TB6600 (or equivalent) DIP switches for current and microstep settings,
and verify the motor is wired correctly (coil A and coil B, not A and B
of the same coil — that's the most common wiring mistake with TB6600s).

### `from pymodbus.client import ModbusTcpClient` raises `SyntaxError`

You have a typo in `UI.py`, almost certainly a stray character on the
`try:` line above the import. The Python parser sees `try:` followed by
something invalid, decides the try block is empty, and complains that
there's no matching `except`. Look at lines 14–16 and check the `try:`
line is exactly `try:` with nothing after the colon.

### `pyvista` install hangs or fails on the Pi

PyVista pulls in VTK as a dependency, and VTK is a chunky download even
when piwheels has a prebuilt wheel. If pip appears to hang, give it 5–10
minutes before assuming it's broken. If it actually fails, check disk
space with `df -h /` — VTK plus dependencies want about 500 MB free.

### `python3 UI.py` works but `python UI.py` says `ModuleNotFoundError`

`python` and `python3` are different interpreters on Raspberry Pi OS,
and pymodbus was installed for `python3`. Always use `python3` explicitly.

---

## Project Structure

```
CapstoneUI/
├── UI.py                      # Main PyQt6 operator interface
├── cc_modbus_test.py          # Standalone CLI Modbus tester
├── ClearCoreModbusTest.ino    # ClearCore Arduino sketch (the slave)
└── README.md                  # This file
```

That's it. Three source files plus this README. The simplicity is
intentional — the comms test is meant to be auditable end-to-end by one
person in one sitting.

### What's in each file

**`UI.py`** (~1130 lines) — The PyQt6 operator interface. Contains the
main window, the 3D viewport (PyVista/VTK), the scan worker thread, and
the `ClearCoreModbus` QThread that owns the Modbus client. Most of the
file is UI layout code; the comms layer is roughly 250 lines starting
near line 245.

**`cc_modbus_test.py`** (~410 lines) — Standalone CLI tester for the
Modbus link. Has zero dependencies on PyQt or PyVista, so it works even
if the UI is broken. Drops into an interactive shell after verifying
the link.

**`ClearCoreModbusTest.ino`** (~575 lines) — The ClearCore-side sketch.
Implements a Modbus TCP server with a hand-rolled MBAP parser, a
back-and-forth motion test sequence, the `CLIENT_INFC` shared register
struct, and command dispatch. Uses the ClearCore Arduino wrapper API.

---

## Future Work / Known Limitations

These are the things this comms test does NOT do, in rough priority order
of "what to tackle next."

### UI is unusable on the 7" Pi touchscreen

The current `UI.py` layout was designed for a desktop monitor (≥1280×800)
and is squished into illegibility on the 800×480 official Pi touchscreen.
A proper touchscreen UI needs:

- Single-panel-at-a-time layout with tab navigation, not the current
  3-column layout
- Buttons sized for finger touch (minimum 44×44 px, ideally 60×60)
- Larger fonts throughout
- The 3D viewport probably as its own dedicated tab rather than a sidebar
- Larger touch targets for E-stop and other safety-critical buttons

For demo purposes, an external HDMI monitor at 1920×1080 works fine and
buys time to do this redesign properly later.

### No real point cloud capture yet

`UI.py`'s viewport currently shows an empty point cloud placeholder. The
Intel RealSense D405 camera capture pipeline is not yet wired in.
`_update_pointcloud_live()` is the hook point — it accepts a `pv.PolyData`
and renders it. Plug your camera capture thread in there.

### No real scan motion profile

The ClearCore sketch's `CCMD_RUN_1` runs a placeholder back-and-forth
test, not the actual scan motion profile. Real scanning needs the camera
to capture frames at known carriage positions, which means coordinated
motion+capture instead of free-running motion. The `CCMD_MOVE` path
already exists in the sketch and accepts acc/vel/target — you can drive
discrete moves from the Pi side and capture between them.

### No homing

The ClearCore sketch doesn't implement homing. `CCMD_SET_ZERO` is a
no-op stub. For a real scan you'll want either:

- A homing routine that drives toward a hard stop or limit switch on
  startup, then sets position to a known reference
- Or absolute encoder feedback (ClearPath-SDSK supports this)

### G-code export not yet implemented

The Scan-to-Mill workflow ends with exporting a toolpath to the
Genmitsu CNC. None of that exists in this codebase yet.

### Hardware E-stop is software-only

`CCMD_DISAB_MTRS` halts the motor and drops the enable line, but it
runs in the main loop alongside everything else. A genuine hardware
E-stop should use the panic-button hardware interrupt path on DI-6
that's currently commented out in `setup()`. Wire a normally-open
button between DI-6 and GND, uncomment the `attachInterrupt` line,
and the ISR will fire regardless of what the main loop is doing.

For full safety compliance you also want a hardware contactor cutting
the 24V drive power on the same button — never trust software alone
for E-stop on a real machine.

### No authentication or encryption

Modbus TCP has no built-in authentication or encryption. Anyone on the
same network segment can read or write the registers. Fine for a direct
cable between two devices on a benchtop, NOT fine for anything connected
to a shared network. If you ever put this on a real network, put it
behind a firewall that only allows the Pi's IP to reach port 502 on the
ClearCore.

### The MBAP parser is single-client

`serviceModbus()` only handles one TCP client at a time. If a second
client tries to connect, it'll be ignored until the first one disconnects.
For this test rig that's fine — there's only ever one Pi talking to one
ClearCore — but it would need to be reworked to use the Arduino
`server.accept()` API and a client array if you ever want multiple
masters. See the Arduino `EthernetServerAccept` reference for how.

---

## Credits and Contact

**Project:** Scan-to-Mill capstone, Oklahoma State University
School of Electrical and Computer Engineering, 2025–2026

**Faculty advisor:** Dr. Piao, ENDEAVOR Room 220

**Hardware:**
- Teknic Inc. — ClearCore PLC and ClearPath-SD servos
- Intel — RealSense D405 depth camera
- Genmitsu — 3020 Pro Max V2 CNC router

**References:**
- Teknic ClearCore Arduino wrapper:
  https://github.com/Teknic-Inc/ClearCore-Arduino-wrapper
- Teknic Modbus reference (RTU example):
  https://teknic-inc.github.io/ClearCore-library/
- pymodbus documentation:
  https://pymodbus.readthedocs.io/
- Modbus TCP specification:
  https://modbus.org/docs/Modbus_Messaging_Implementation_Guide_V1_0b.pdf

---

## Quick Reference Card

For when you've forgotten everything and just need to get back to a
working state. Tape this to the wall.

```
# Pi network setup (one-time)
sudo nmcli connection modify netplan-eth0 ipv4.method manual ipv4.addresses 192.168.1.10/24 ipv4.gateway "" ipv6.method ignore
sudo nmcli connection down netplan-eth0
sudo nmcli connection up netplan-eth0

# ClearCore IP (in ClearCoreModbusTest.ino)
IPAddress STATIC_IP (192, 168, 1, 20);

# Ping test
ping -c 4 192.168.1.20

# Read ClearCore boot log
sudo screen /dev/ttyACM0 9600
# (exit with Ctrl+A then \ then y)

# Run the comms test
python3 cc_modbus_test.py

# Run the full UI
QT_QPA_PLATFORM=xcb python3 UI.py
```
