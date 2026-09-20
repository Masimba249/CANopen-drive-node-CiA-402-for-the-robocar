# Master side (Raspberry Pi)

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
./setup_can.sh can0 500000
```

| File | What it is |
|---|---|
| `robocar.py` | master library (`RobocarDrive`) and CLI |
| `fota_client.py` | firmware update over CAN |
| `eds/robocar_drive.eds` | the node's EDS |
| `tools/check_eds.py` | proves the EDS still matches the firmware |
| `tools/sim_node.py` | runs the real firmware stack in-process over a virtual bus |
| `setup_can.sh` | brings up `can0` (or a `vcan0` for testing) |

## CLI

```bash
./robocar.py scan                                   # who is on the bus
./robocar.py info                                   # identity, state, diagnostics, error history
./robocar.py enable                                 # walk both axes to OPERATION ENABLED
./robocar.py drive --linear 0.25 --angular 0.5 --seconds 3
./robocar.py monitor                                # live TPDO telemetry
./robocar.py stop                                   # quick stop, then disable
./robocar.py reset-fault
./robocar.py config --show
./robocar.py config --set 0x6083=8000 --set 0x2002:1=3000 --save
```

Every command takes `--channel`, `--interface`, `--bitrate`, `--node` and
`--eds`. Wheel geometry for the m/s conversion comes from `--wheel-radius`
and `--wheel-base`.

## As a library

```python
from robocar import RobocarDrive, Geometry, LEFT, RIGHT

with RobocarDrive(channel="can0", node_id=0x10,
                  geometry=Geometry(wheel_radius_m=0.0325, wheel_base_m=0.15)) as d:
    d.on_emcy(lambda emcy, line: print("!", line))
    d.start()                       # NMT operational
    d.enable_all()                  # CiA 402 -> OPERATION ENABLED
    d.setup_pdos(command_period=0.02)

    d.drive(linear_mps=0.3, angular_rps=0.0)
    print(d.tpdo_snapshot())

    d.stop()
    d.disable_all()
```

`RobocarDrive.enable()` reads the statusword back after every command rather
than assuming the transition happened, because a drive that silently refuses
a transition is the single most common integration bug.

## Firmware update

```bash
./fota_client.py ../firmware/build/robocar_canopen_drive.bin
```

Add `--no-confirm` to skip the final confirmation and watch the node roll
itself back, or `--no-block` to use segmented SDO instead of block transfer
(slower, useful when isolating a transport problem). See
[../docs/fota.md](../docs/fota.md).
