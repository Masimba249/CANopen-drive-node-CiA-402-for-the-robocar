# Object dictionary

The full, authoritative list lives in
[`firmware/components/robocar_od/od_data.c`](../firmware/components/robocar_od/od_data.c)
and is mirrored in [`master/eds/robocar_drive.eds`](../master/eds/robocar_drive.eds).
`master/tools/check_eds.py` compares the two and fails if they drift.

```
make -C host_tests od_dump && ./host_tests/build/od_dump   # the real table, as CSV
master/tools/check_eds.py                                  # EDS vs firmware
```

## Layout

| Range | Contents |
|---|---|
| 1000h–1FFFh | communication profile (CiA 301) |
| 2000h–2FFFh | manufacturer specific |
| 6000h–67FFh | CiA 402 drive profile, **axis 1** (left wheel) |
| 6800h–6FFFh | CiA 402 drive profile, **axis 2** (right wheel) |

The 800h gap between axes is the CiA 402 multi-axis offset. So 6040h and
6840h are the two controlwords, 60FFh and 68FFh the two velocity targets, and
6502h / 6D02h the two "supported drive modes".

## Units

Getting these wrong is the most common integration bug, so they are stated
once and used everywhere:

| Quantity | Unit | Objects |
|---|---|---|
| velocity | **0.1 rpm at the wheel** (output shaft), + = forward | 606Ch, 606Bh, 60FFh, 607Fh |
| acceleration | velocity units per second | 6083h, 6084h, 6085h |
| position (internal) | raw quadrature counts | 6063h |
| position (user) | milli-revolutions of the wheel | 6064h |
| current | mA | 6078h, 2004h:01/02 |
| torque | per mille of rated (2002h:02 is "rated") | 6077h |
| voltage | mV | 2004h:03, 2002h:04/05 |

Converting to metres per second for a wheel of radius *r*:

```
v [m/s]  = (units / 10) * 2*pi*r / 60
units    = v * 60 / (2*pi*r) * 10
```

`master/robocar.py` does this in `Geometry`; the defaults are r = 32.5 mm and
a 150 mm wheel base.

## Communication profile

| Index | Name | Type | Access | Default |
|---|---|---|---|---|
| 1000h | Device type | U32 | const | `0x00020192` (profile 402, servo drive) |
| 1001h | Error register | U8 | ro, TPDO | 0 |
| 1003h | Pre-defined error field | array | sub0 rw (write 0 to clear) | — |
| 1005h | COB-ID SYNC | U32 | rw | 0x80 |
| 1006h | Communication cycle period | U32 | rw | 0 |
| 1008h/1009h/100Ah | device / hw / sw version strings | string | ro | — |
| 1010h:01 | Store parameters | U32 | rw | signature `"save"` |
| 1011h:01 | Restore defaults | U32 | rw | signature `"load"` |
| 1014h | COB-ID EMCY | U32 | rw | `$NODEID+0x80` |
| 1015h | Inhibit time EMCY | U16 | rw | 1000 (= 100 ms) |
| 1016h | Consumer heartbeat time | array(2) | rw | 0 (disabled) |
| 1017h | Producer heartbeat time | U16 | rw | 500 ms |
| 1018h | Identity | record | const/ro | vendor `0x0000FEED` |
| 1029h:01 | Error behaviour | U8 | rw | 0 |
| 1200h | SDO server parameter | record | const | `$NODEID+0x600/0x580` |
| 1400h/1401h | RPDO communication | record | rw | — |
| 1600h/1601h | RPDO mapping | record | rw | — |
| 1800h–1802h | TPDO communication | record | rw | — |
| 1A00h–1A02h | TPDO mapping | record | rw | — |

**Vendor ID.** `0x0000FEED` is a placeholder. Real vendor IDs are assigned by
CAN in Automation; use a registered one before putting a node on a shared
industrial bus.

## Manufacturer objects

| Index | Name | Notes |
|---|---|---|
| 2000h | Motor configuration axis 1 | PWM frequency, encoder counts, gear ratio, invert, use-encoder, Kp, Ki, no-load speed |
| 2001h | Motor configuration axis 2 | same layout |
| 2002h | Protection limits | over-current, stall current/time, under/over-voltage, command watchdog |
| 2003h | Bus and system diagnostics | CAN state, TX/RX error counters, bus-off count, RX overruns, uptime, free heap |
| 2004h | Analogue measurements | both motor currents and the supply voltage, all TPDO mappable |
| 2005h | Network configuration | pending node-id and bit rate; applied after a reset once stored with 1010h |
| 2100h | Firmware update control | size, CRC-32, command, status, bytes received, running slot/version, confirm timeout |
| 2101h | Firmware image | DOMAIN, write only, SDO block download target |

`2000h:05 use_encoder = 0` switches an axis to an open-loop estimate of
606Ch derived from the applied duty cycle and `2000h:08 no_load_speed`. It is
honest about being an estimate, and it lets the node run with motors that have
no encoder fitted.

## CiA 402 objects (per axis)

Add 800h for axis 2.

| Index | Name | Type | Access |
|---|---|---|---|
| 6007h | Abort connection option code | I16 | rw |
| 6040h | Controlword | U16 | rw, **RPDO** |
| 6041h | Statusword | U16 | ro, **TPDO** |
| 605Ah | Quick stop option code | I16 | rw |
| 605Bh | Shutdown option code | I16 | rw |
| 605Ch | Disable operation option code | I16 | rw |
| 605Eh | Fault reaction option code | I16 | rw |
| 6060h | Modes of operation | I8 | rw, RPDO |
| 6061h | Modes of operation display | I8 | ro, TPDO |
| 6063h | Position actual internal value | I32 | ro, TPDO |
| 6064h | Position actual value | I32 | ro, TPDO |
| 606Bh | Velocity demand value | I32 | ro, TPDO |
| 606Ch | Velocity actual value | I32 | ro, TPDO |
| 606Dh/606Eh | Velocity window / window time | U16 | rw |
| 606Fh/6070h | Velocity threshold / threshold time | U16 | rw |
| 6077h | Torque actual value | I16 | ro, TPDO |
| 6078h | Current actual value | I16 | ro, TPDO |
| 607Fh | Max profile velocity | U32 | rw |
| 6083h/6084h | Profile acceleration / deceleration | U32 | rw |
| 6085h | Quick stop deceleration | U32 | rw |
| 60FFh | Target velocity | I32 | rw, **RPDO** |
| 6502h | Supported drive modes | U32 | const, `0x04` = profile velocity |

Only **Profile Velocity** (6060h = 3) is implemented. Writing any other mode
is accepted by the SDO server but 6061h then reports 0 ("no mode") and the
ramp output is held at zero, which is what CiA 402 requires of an unsupported
mode.

## Default PDO configuration

| PDO | COB-ID | Type | Timing | Contents |
|---|---|---|---|---|
| TPDO1 | `$NODEID+0x180` | 255 | 10 ms event timer | 6041h (16) + 606Ch (32) |
| TPDO2 | `$NODEID+0x280` | 255 | 10 ms event timer | 6841h (16) + 686Ch (32) |
| TPDO3 | `$NODEID+0x380` | 255 | 100 ms event timer | 1001h (8) + 2004h:01/02/03 (16 each) |
| RPDO1 | `$NODEID+0x200` | 255 | 100 ms watchdog | 6040h (16) + 60FFh (32) |
| RPDO2 | `$NODEID+0x300` | 255 | 100 ms watchdog | 6840h (16) + 68FFh (32) |

Event-driven TPDOs also transmit on change of state, subject to the inhibit
time in sub-index 3. Transmission types 0 and 1–240 (synchronous) work too;
set 1800h:02 and send SYNC frames on 1005h.

For an RPDO, sub-index 5 is read as a **command watchdog**: if no RPDO arrives
within that many milliseconds while the node is OPERATIONAL, the node applies
6007h. Default behaviour is a quick stop.

## Remapping at runtime

The standard three-step dance, enforced by the node:

1. write 0 to `1A00h:00` — this invalidates the mapping,
2. write the new entries to `1A00h:01..08`,
3. write the entry count back to `1A00h:00` — this validates them.

Step 2 is rejected with `0x08000022` if the count is not zero. Step 3 is
rejected with `0x06040041` if an object is missing, is not mappable in that
direction, or has a bit length that does not match its size, and with
`0x06040042` if the total exceeds 64 bits. Changing a COB-ID while the PDO is
valid is rejected with `0x06040043`; set bit 31 first.

Mapped objects must be a whole number of bytes. Bit-granular packing is legal
in CiA 301 but is not implemented here.
