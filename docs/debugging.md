# Bus-level debugging

## Tools

```bash
candump -tz can0                     # timestamps, everything
candump -tz can0,710:7FF             # just node 0x10's heartbeat
cansniffer can0                      # per-ID view that highlights changes
cangen can0 -g 5 -I 123 -L 8         # load generator
canbusload can0@500000 -t -b -c      # bus utilisation
ip -details -statistics link show can0   # error counters, restarts, bus-off
```

`candump` is in `can-utils`: `sudo apt install can-utils`.

## Reading the traffic

With node-id 0x10 and the default configuration:

| COB-ID | What it is |
|---|---|
| `0x000` | NMT command (from the master) |
| `0x080` | SYNC |
| `0x090` | EMCY from this node |
| `0x190` | TPDO1: statusword + velocity, axis 1 |
| `0x210` | RPDO1: controlword + target, axis 1 |
| `0x290` | TPDO2: axis 2 |
| `0x310` | RPDO2: axis 2 |
| `0x390` | TPDO3: error register, currents, supply |
| `0x590` | SDO response (node → master) |
| `0x610` | SDO request (master → node) |
| `0x710` | heartbeat |

A healthy idle node in pre-operational looks like exactly one frame every
500 ms:

```
(1695...) can0  710   [1]  7F
```

`7F` = 127 = PRE-OPERATIONAL. After `NMT start` it becomes `05`.

## Symptom → cause

**Nothing on the bus at all, and the TX error counter is pinned at 128.**
The node is transmitting into a bus with no acknowledger. Either the other
node is not running, the transceiver is not powered, CANH/CANL are swapped, or
the bit rates differ. Check `ip -details -statistics link show can0` on the
Pi: if it is also error-passive, both ends are talking past each other.

**`bus-off` in the Pi statistics, or `2003h:01` reads 2 or 3.**
128 transmit errors in a row. Almost always a bit-rate mismatch or missing
termination. The firmware initiates recovery automatically and counts the
events in `2003h:04`; if that number climbs, the physical layer is wrong, not
the software.

**Frames appear but the payload is nonsense.**
Check the mapping actually programmed into the node rather than the EDS:
`node.tpdo.read()` in python-canopen reads 1A00h from the device. The EDS is a
description, the node is the authority. `master/tools/check_eds.py` exists to
keep the two honest.

**SDO aborts with `0x05040000` (timeout).**
The node did not answer within the client's window. Common during a firmware
update because the flash erase in BEGIN takes a few hundred milliseconds —
`fota_client.py --sdo-timeout 5` raises it.

**SDO aborts with `0x06040043` when writing a PDO COB-ID.**
Set bit 31 (invalid) first, change the COB-ID, then clear it. See
[object_dictionary.md](object_dictionary.md#remapping-at-runtime).

**SDO aborts with `0x08000022` when writing a mapping entry.**
Write 0 to sub-index 0 first.

**The drive will not leave SWITCH ON DISABLED.**
Read 6041h. If bit 4 (voltage enabled) is clear, the node thinks the supply is
missing — check the divider scaling in `board.h` and `2004h:03`. If the state
is FAULT, read 1003h to find out why, then send a *rising edge* on
controlword bit 7 (clear it first; `robocar.py reset-fault` does this).

**The wheels stop on their own after ~200 ms.**
That is the RPDO command watchdog (`2002h:06`) applying 6007h. Either send
RPDOs more often, or raise the timeout, or set 6007h to 0 if you really want
the drive to keep running unattended.

**EMCY `0x8210` (PDO length error).**
An RPDO arrived shorter than the mapping expects. The master is transmitting
a DLC that does not match what it programmed.

## Emergency codes this node produces

| Code | Meaning | Usual cause |
|---|---|---|
| `0x2310` | continuous over-current | 2002h:01 exceeded |
| `0x3210` / `0x3220` | over / under voltage | 2002h:05 / 2002h:04 |
| `0x7305` | incremental encoder fault | commanded and drawing current, but the counter is not moving |
| `0x8110` | CAN overrun | the RX queue filled; the node was too busy |
| `0x8120` | CAN error passive / bus-off | physical layer |
| `0x8130` | heartbeat lost | the producer in 1016h went quiet |
| `0x8140` | recovered from bus-off | informational |
| `0x8210` | PDO not processed, length error | short RPDO |
| `0xFF01` | motor stalled | over `2002h:02` for `2002h:03` with no motion |
| `0xFF02` | firmware update failed | |
| `0x0000` | error reset | sent when the last active condition clears |

Byte 0 of the manufacturer-specific field carries the axis number (1 or 2);
bytes 1–2 carry the measured current in mA for the motor faults.

## Debugging without hardware

The whole protocol stack compiles on the host:

```bash
make -C host_tests run           # 215 assertions, under ASan and UBSan
make -C host_tests sim           # build the node as a shared library
master/tools/sim_node.py         # real python-canopen against the real stack
```

The simulator runs the production object dictionary, SDO server, PDO engine
and CiA 402 state machine; only the CAN driver, the motors and the flash are
stand-ins. It is the fastest way to check a protocol change before touching
the robot.
