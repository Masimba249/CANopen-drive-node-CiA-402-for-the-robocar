# CANopen drive node (CiA 402) for the robocar

An ESP32 running as a **CANopen slave** that drives the robocar's two motors,
with a Raspberry Pi as the CANopen master. Real CAN on the wire: the ESP32's
TWAI peripheral through an SN65HVD230 transceiver, an MCP2515 HAT or USB
dongle on the Pi, and `python-canopen` as the master stack.

What is here:

- a **CANopen (CiA 301) stack** written from scratch — object dictionary, SDO
  server with expedited, segmented *and block* transfer, PDO engine with
  mapping and event timers, NMT slave, heartbeat producer and consumer, SYNC
  consumer, EMCY producer;
- the **CiA 402 drive profile**: controlword/statusword, the full state
  machine, Profile Velocity mode with a ramp generator, option codes;
- two axes, at 6000h and 6800h, driven by PWM with quadrature feedback, a PI
  velocity loop, and stall / over-current / under-voltage protection;
- a **hand-written EDS** that standard CANopen tools can browse, with a
  checker that proves it still matches the firmware;
- **firmware update over CAN**: SDO block download into the spare OTA slot,
  CRC-32 verification, and automatic rollback if the new image cannot talk to
  the bus;
- a **host test suite** and an **in-process simulator**, so all of the above
  can be run and modified without any hardware on the desk.

## Layout

```
firmware/            ESP-IDF project
  components/
    canopen/         CiA 301 stack        (portable C99)
    cia402/          drive profile        (portable C99)
    robocar_od/      this device's OD     (portable C99)
    drive_hw/        TWAI, PWM, encoders, ADC   (ESP32)
    fota/            firmware update over SDO   (ESP32)
  main/              wiring, persistence, the 1 ms task
host_tests/          host build of the portable half + tests + simulator
master/              Raspberry Pi side
  robocar.py         master library and CLI
  fota_client.py     firmware update client
  eds/               the EDS
  tools/             EDS checker, simulator harness
docs/                object dictionary, state machine, bit timing, FOTA,
                     wiring, bus-level debugging
```

## Try it with no hardware

```bash
make -C host_tests run            # 215 assertions, ASan + UBSan
make -C host_tests sim            # build the stack as a shared library

python3 -m venv .venv && .venv/bin/pip install -r master/requirements.txt
.venv/bin/python master/tools/sim_node.py
```

The last command runs the real `python-canopen` master against the real
firmware stack over a virtual bus: identity reads, the 402 state machine,
PDO traffic, EMCY, a 64 KiB SDO block download and a deliberately corrupted
one. Only the CAN driver, the motors and the flash are simulated.

```bash
master/tools/check_eds.py         # EDS vs firmware object dictionary
```

## Build and flash

```bash
cd firmware
idf.py set-target esp32
idf.py build flash monitor
```

Tested against ESP-IDF v5.1–v5.3. The default node-id is `0x10` and the
default bit rate 500 kbit/s; both are in `main/app_config.h` and can be
changed over the bus through object 2005h plus a 1010h store.

## Drive it

On the Pi:

```bash
master/setup_can.sh can0 500000
pip install -r master/requirements.txt

master/robocar.py scan
master/robocar.py info
master/robocar.py enable
master/robocar.py drive --linear 0.25 --angular 0.5 --seconds 3
master/robocar.py monitor
```

Update the firmware over the bus:

```bash
master/fota_client.py firmware/build/robocar_canopen_drive.bin
master/fota_client.py firmware/build/robocar_canopen_drive.bin --no-confirm  # prove the rollback
```

## Default bus configuration

Node-id `0x10`, 500 kbit/s.

| COB-ID | Contents | Rate |
|---|---|---|
| `0x190` TPDO1 | statusword + velocity actual, axis 1 | 10 ms event timer |
| `0x290` TPDO2 | statusword + velocity actual, axis 2 | 10 ms event timer |
| `0x390` TPDO3 | error register, both currents, supply voltage | 100 ms |
| `0x210` RPDO1 | controlword + target velocity, axis 1 | master's choice |
| `0x310` RPDO2 | controlword + target velocity, axis 2 | master's choice |
| `0x090` EMCY | stall, over-current, under-voltage, bus errors | on event |
| `0x710` heartbeat | NMT state | 500 ms |
| `0x610`/`0x590` SDO | configuration and firmware download | on demand |

Velocities are in **0.1 rpm at the wheel**; see
[docs/object_dictionary.md](docs/object_dictionary.md#units).

## Documentation

| Document | Contents |
|---|---|
| [docs/object_dictionary.md](docs/object_dictionary.md) | every object, the units, the PDO defaults, the remapping rules |
| [docs/state_machine.md](docs/state_machine.md) | the CiA 402 state diagram, controlword masks, statusword decoding, option codes |
| [docs/bit_timing.md](docs/bit_timing.md) | how the ESP32's 80 MHz APB becomes 500 kbit/s, sample points, bus length, termination |
| [docs/fota.md](docs/fota.md) | the update protocol, why block download, how rollback is made to depend on the bus working |
| [docs/wiring.md](docs/wiring.md) | pin map and why each pin, power, termination, first power-up checklist |
| [docs/debugging.md](docs/debugging.md) | `candump` recipes, symptom → cause, every EMCY code this node emits |

## Design notes

**One task, no locks.** The CANopen stack and the control loop both run in a
single 1 ms FreeRTOS task, so the object dictionary is only ever touched from
one context. The cost is that a slow operation delays PDOs, which is why the
flash erase during a firmware update is one explicit step and why updates are
refused while a power stage is live.

**The portable half is genuinely portable.** `canopen/`, `cia402/` and
`robocar_od/` are plain C99 with no ESP-IDF headers, which is what makes the
host tests and the simulator possible — and what would make a port to an
STM32 a driver rewrite rather than a rewrite.

**The EDS cannot silently rot.** `check_eds.py` builds the firmware's real
object dictionary, asks it for its contents, and diffs against the EDS:
indices, sub-indices, data types, access, and PDO mappability, plus the EDS's
own internal consistency. An EDS that has drifted is worse than none.

**Block download is implemented properly.** Only in-sequence bytes are
committed, so a lost frame costs one sub-block rather than the transfer, and
the final segment is held back until the end-block frame says how much of it
is padding.

## Status and limitations

- Profile Velocity mode only. Position and torque modes are rejected as the
  standard requires (6061h reports 0), not silently ignored.
- SDO block *upload* is not implemented; the server answers `0x05040001` and
  clients fall back to segmented upload.
- PDO mapping is byte-granular. Bit-granular packing is legal in CiA 301 and
  is not supported.
- No LSS. The node-id and bit rate are set through object 2005h and a 1010h
  store instead.
- The vendor ID (`0x0000FEED`) is a placeholder; a node on a shared
  industrial bus needs one registered with CAN in Automation.
