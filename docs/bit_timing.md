# Bit timing and the physical layer

## How a CAN bit is built

A bit time is divided into *time quanta* (tq):

```
t_q    = BRP / f_clk
t_bit  = t_q * (1 + TSEG1 + TSEG2)          the leading 1 is SYNC_SEG
sample point = (1 + TSEG1) / (1 + TSEG1 + TSEG2)
```

Every node on the bus must agree on `t_bit`. They do **not** have to agree on
how it is divided, but the sample points should be close, and CiA 301
recommends 87.5 % (or 75 % at 1 Mbit/s on long cable).

## What the ESP32 uses

The TWAI peripheral is clocked from APB at 80 MHz. The `TWAI_TIMING_CONFIG_*`
macros in ESP-IDF give:

| Bit rate | BRP | TSEG1 | TSEG2 | SJW | tq | tq/bit | Sample point |
|---|---|---|---|---|---|---|---|
| 125 kbit/s | 32 | 15 | 4 | 3 | 400 ns | 20 | 80 % |
| 250 kbit/s | 16 | 15 | 4 | 3 | 200 ns | 20 | 80 % |
| 500 kbit/s | 8 | 15 | 4 | 3 | 100 ns | 20 | 80 % |
| 1 Mbit/s | 4 | 15 | 4 | 3 | 50 ns | 20 | 80 % |

Check for 500 kbit/s: `80 MHz / 8 = 10 MHz`, so `t_q = 100 ns`;
`20 tq = 2 µs = 500 kbit/s`; sample point `(1+15)/20 = 80 %`. 

SJW = 3 tq gives generous resynchronisation headroom, which matters because
the ESP32 runs from a crystal that is typically ±10 ppm but can be much worse
on cheap modules.

`can_port_init()` in
[`firmware/components/drive_hw/can_port.c`](../firmware/components/drive_hw/can_port.c)
accepts 125, 250, 500 and 1000 kbit/s and rejects anything else rather than
silently running at the wrong speed.

## Matching the Raspberry Pi side

SocketCAN computes its own segments from the requested bit rate and the
controller's clock:

```bash
ip -details link show can0
# ... bitrate 500000 sample-point 0.875
#     tq 125 prop-seg 6 phase-seg1 7 phase-seg2 2 sjw 1
```

87.5 % on the Pi against 80 % on the ESP32 is fine — the difference is well
inside what resynchronisation absorbs on a short bus. If you ever need them
identical, SocketCAN takes explicit segments:

```bash
sudo ip link set can0 up type can tq 100 prop-seg 7 phase-seg1 8 phase-seg2 4 sjw 3
```

## The MCP2515 crystal trap

An MCP2515 HAT derives its bit rate from its own crystal. The device-tree
overlay has to state the crystal that is actually fitted:

```
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
```

Boards with an 8 MHz crystal and an overlay that claims 16 MHz produce a bus
that looks alive and transmits nothing but error frames, because every bit is
twice as long as the other nodes expect. If `ip -details -statistics link show
can0` shows the error counters climbing and `bus-off` events with no traffic
getting through, check this first.

## Bus length

Propagation delay has to fit inside the segment before the sample point. The
budget is roughly 5 ns per metre of twisted pair each way, plus the
transceiver loop delay (around 70 ns each way for an SN65HVD230 in high-speed
mode).

| Bit rate | Practical maximum |
|---|---|
| 1 Mbit/s | ~25 m |
| 500 kbit/s | ~100 m |
| 250 kbit/s | ~250 m |
| 125 kbit/s | ~500 m |

A robocar needs a metre or two, so 500 kbit/s is comfortable and leaves
bandwidth for the firmware update path.

## Bus load

At 500 kbit/s a standard-ID data frame with 8 bytes costs about 111 bits
before stuffing, so roughly 240 µs. The default configuration sends:

- TPDO1 + TPDO2, 6 bytes each, every 10 ms → ~2 × 190 µs per 10 ms
- TPDO3, 7 bytes, every 100 ms
- heartbeat, 1 byte, every 500 ms
- two RPDOs from the master at whatever rate it chooses (20 ms is typical)

That is under 10 % bus load for the steady state, which leaves plenty of room
for the SDO block download during a firmware update.

## Wiring the SN65HVD230

| SN65HVD230 | Connect to |
|---|---|
| D (TXD) | ESP32 GPIO21 |
| R (RXD) | ESP32 GPIO22 |
| Vcc | 3.3 V (this is a 3.3 V transceiver — do not feed it 5 V) |
| GND | ground, shared with the Pi side |
| CANH / CANL | twisted pair to the bus |
| Rs | GND for high-speed mode. A resistor to GND sets slope control; high puts the device in standby. |

Terminate with **120 Ω across CANH/CANL at each physical end of the bus, and
only at the ends**. Many breakout boards ship with a 120 Ω resistor already
fitted; with two boards that is correct, with three it is not, and you will
see it as a bus that works at 125 kbit/s and falls apart at 500.
