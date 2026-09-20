# Firmware

ESP-IDF project. Layout:

| Path | Contents | Portable? |
|---|---|---|
| `components/canopen` | CiA 301 stack: OD, SDO, PDO, NMT, heartbeat, SYNC, EMCY | yes, plain C99 |
| `components/cia402`  | CiA 402 state machine + velocity profile | yes, plain C99 |
| `components/robocar_od` | this device's object dictionary and defaults | yes, plain C99 |
| `components/drive_hw` | TWAI port, H-bridge PWM, encoders, ADC | ESP32 only |
| `components/fota` | firmware update over SDO, CRC-32, rollback | ESP32 only |
| `main` | wiring, persistence, fault mapping, the 1 ms task | ESP32 only |

The first three compile on the host; `host_tests/` builds and exercises them
with no hardware attached.

```
idf.py set-target esp32
idf.py build flash monitor
```
