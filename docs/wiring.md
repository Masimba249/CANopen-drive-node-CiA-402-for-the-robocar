# Wiring

## Bill of materials

| Part | Notes |
|---|---|
| ESP32-WROOM-32 dev board | any; the pin map avoids strapping and flash pins |
| SN65HVD230 (or VP230) breakout | 3.3 V CAN transceiver |
| TB6612FNG dual H-bridge | or any driver with IN1/IN2 per channel plus a standby pin |
| 2 × gearmotor with quadrature encoder | e.g. 30:1 with an 11 PPR magnetic encoder |
| 2 × current sense (INA181, or the driver's own sense pin) | optional but needed for stall and over-current detection |
| Battery divider | 100 k / 22 k into GPIO32 |
| 2 × 120 Ω resistor | bus termination, one at each end |
| Raspberry Pi + MCP2515 HAT or USB-CAN dongle | the master |

## ESP32 pin map

Defined in
[`firmware/components/drive_hw/include/board.h`](../firmware/components/drive_hw/include/board.h).

| Function | GPIO | Notes |
|---|---|---|
| TWAI TX → transceiver D | 21 | |
| TWAI RX ← transceiver R | 22 | |
| Motor L IN1 / IN2 | 16 / 17 | LEDC channels 0 and 1 |
| Motor R IN1 / IN2 | 18 / 19 | LEDC channels 2 and 3 |
| Motor driver STBY | 23 | low = both bridges off |
| Encoder L A / B | 34 / 35 | input-only pins, ideal for encoders |
| Encoder R A / B | 26 / 27 | |
| Current sense L | 36 | ADC1_CH0 |
| Current sense R | 39 | ADC1_CH3 |
| Battery divider | 32 | ADC1_CH4 |
| Status LED | 2 | on most dev boards already |

Why these pins:

- GPIO34/35/36/39 are **input only**. That makes them useless for motor
  outputs and perfect for encoder inputs and analogue measurements.
- Everything analogue is on **ADC1**. ADC2 is unusable while Wi-Fi is
  active; this firmware does not use Wi-Fi, but keeping to ADC1 means adding
  it later does not break the current sensing.
- Strapping pins (0, 2, 12, 15) and the SPI flash pins (6–11) are avoided,
  except GPIO2, which only drives the LED.

## Power

```
   battery 11.1 V ──┬── motor driver VM
                    │
                    ├── 100k ──┬── GPIO32   (divider, 0.1803 ratio)
                    │          22k
                    │          │
                    └──────────┴── GND
                    
   5 V regulator ── ESP32 VIN
   3.3 V (from the ESP32 board) ── SN65HVD230 Vcc, TB6612 VCC (logic)
```

Grounds must be common: battery, motor driver, ESP32, transceiver, and the
CAN cable's ground/shield if you run one.

`BOARD_VBAT_DIV_X1000` in `board.h` is `1000 * R2/(R1+R2)`. Change it if you
use different resistors, or the under-voltage trip will fire at the wrong
point. `BOARD_ISENSE_MV_PER_A` likewise describes the shunt and amplifier:
0.05 Ω into a ×50 amplifier gives 2500 mV per amp.

## CAN bus

```
  ESP32 + SN65HVD230                         Raspberry Pi + MCP2515
  ┌──────────────┐                            ┌──────────────┐
  │        CANH ─┼──── twisted pair ──────────┼─ CANH        │
  │        CANL ─┼────                    ────┼─ CANL        │
  │         GND ─┼────────────────────────────┼─ GND         │
  └──────┬───────┘                            └──────┬───────┘
       120 Ω                                       120 Ω
```

Two nodes, two terminators, one at each end. Check with a multimeter across
CANH/CANL with everything powered down: you should read about **60 Ω**. 120 Ω
means one terminator is missing; 40 Ω means there are three.

## Raspberry Pi setup

MCP2515 HAT — add to `/boot/firmware/config.txt` and reboot:

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
```

Set `oscillator` to the crystal actually fitted to your HAT. Then:

```bash
master/setup_can.sh can0 500000
```

USB dongle presenting as slcan:

```bash
sudo slcand -o -c -s6 /dev/ttyUSB0 can0    # -s6 = 500 kbit/s
sudo ip link set can0 up
```

## First power-up checklist

1. Motors **disconnected** from the wheels, or the robot on blocks.
2. `master/robocar.py scan` — the node should appear.
3. `master/robocar.py info` — check the supply voltage reading is right
   before trusting the under-voltage trip.
4. `candump -tz can0` — a heartbeat every 500 ms, nothing else, in
   pre-operational.
5. `master/robocar.py enable` then `master/robocar.py info` — both axes should
   report OPERATION ENABLED.
6. `master/robocar.py drive --linear 0.05 --seconds 2` — a slow crawl.
   If a wheel spins the wrong way, set `2000h:04` (or `2001h:04`) to 1 and
   store with `--save`.
