#!/usr/bin/env python3
"""
Robocar CANopen master.

A thin, readable layer over python-canopen that speaks the CiA 402 profile
this node implements, plus a command line for driving and inspecting it.

    ./robocar.py info
    ./robocar.py enable
    ./robocar.py drive --linear 0.25 --angular 0.5 --seconds 3
    ./robocar.py monitor
    ./robocar.py config --show
    ./robocar.py reset-fault

Every command takes --channel/--node/--bitrate; see --help.
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

try:
    import canopen
except ImportError:  # pragma: no cover
    sys.exit("python-canopen is not installed:  pip install -r requirements.txt")

EDS_PATH = Path(__file__).resolve().parent / "eds" / "robocar_drive.eds"

AXIS_OFFSET = 0x800          # CiA 402 multi-axis index offset
LEFT, RIGHT = 0, 1

# --- CiA 402 controlword commands -----------------------------------------
CW_SHUTDOWN         = 0x0006
CW_SWITCH_ON        = 0x0007
CW_ENABLE_OPERATION = 0x000F
CW_DISABLE_VOLTAGE  = 0x0000
CW_QUICK_STOP       = 0x0002
CW_FAULT_RESET      = 0x0080
CW_HALT             = 0x0100

STATE_MASKS = [
    (0x004F, 0x0000, "NOT READY TO SWITCH ON"),
    (0x004F, 0x0040, "SWITCH ON DISABLED"),
    (0x006F, 0x0021, "READY TO SWITCH ON"),
    (0x006F, 0x0023, "SWITCHED ON"),
    (0x006F, 0x0027, "OPERATION ENABLED"),
    (0x006F, 0x0007, "QUICK STOP ACTIVE"),
    (0x004F, 0x000F, "FAULT REACTION ACTIVE"),
    (0x004F, 0x0008, "FAULT"),
]


def decode_state(statusword: int) -> str:
    """Statusword -> CiA 402 state name. Order matters: the masks overlap."""
    for mask, value, name in STATE_MASKS:
        if statusword & mask == value:
            return name
    return f"UNKNOWN(0x{statusword:04X})"


EMCY_NAMES = {
    0x0000: "error reset / no error",
    0x2310: "continuous over-current",
    0x3210: "DC link over-voltage",
    0x3220: "DC link under-voltage",
    0x7305: "incremental encoder fault",
    0x8110: "CAN overrun",
    0x8120: "CAN error passive",
    0x8130: "heartbeat lost",
    0x8140: "recovered from bus-off",
    0x8210: "PDO not processed (length error)",
    0xFF01: "motor stalled",
    0xFF02: "firmware update failed",
}


@dataclass
class Geometry:
    """Robocar wheel geometry, used to convert m/s to profile units."""
    wheel_radius_m: float = 0.0325
    wheel_base_m: float = 0.150

    def mps_to_units(self, v_mps: float) -> int:
        """m/s at the wheel rim -> 0.1 rpm at the wheel (the profile unit)."""
        rpm = v_mps * 60.0 / (2.0 * math.pi * self.wheel_radius_m)
        return int(round(rpm * 10.0))

    def units_to_mps(self, units: int) -> float:
        rpm = units / 10.0
        return rpm * 2.0 * math.pi * self.wheel_radius_m / 60.0

    def twist_to_wheels(self, linear: float, angular: float) -> tuple[float, float]:
        """(v, omega) -> (left, right) wheel speeds in m/s."""
        half = self.wheel_base_m / 2.0
        return linear - angular * half, linear + angular * half


class RobocarDrive:
    """One drive node, two CiA 402 axes."""

    def __init__(self, channel="can0", interface="socketcan", node_id=0x10,
                 bitrate=500000, eds=EDS_PATH, geometry: Geometry | None = None):
        self.network = canopen.Network()
        self.node_id = node_id
        self.geometry = geometry or Geometry()
        self._emcy_cb: Optional[Callable] = None

        kwargs = {"channel": channel, "bitrate": bitrate}
        try:
            self.network.connect(interface=interface, **kwargs)
        except TypeError:
            # python-can < 4.0 / older canopen spell it "bustype".
            self.network.connect(bustype=interface, **kwargs)

        self.node = self.network.add_node(node_id, str(eds))
        self.node.emcy.add_callback(self._on_emcy)

    # --- lifecycle --------------------------------------------------------

    def close(self):
        try:
            self.network.disconnect()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --- object dictionary helpers ---------------------------------------

    def axis_index(self, base: int, axis: int) -> int:
        return base + AXIS_OFFSET * axis

    def sdo_read(self, index: int, sub: int = 0):
        return self.node.sdo[index][sub].raw if sub else self.node.sdo[index].raw

    def sdo_write(self, index: int, value, sub: int = 0):
        if sub:
            self.node.sdo[index][sub].raw = value
        else:
            self.node.sdo[index].raw = value

    # --- NMT --------------------------------------------------------------

    def wait_for_bootup(self, timeout=5.0) -> bool:
        try:
            self.node.nmt.wait_for_bootup(timeout)
            return True
        except Exception:
            return False

    def start(self):
        """NMT start: PDOs begin flowing."""
        self.node.nmt.state = "OPERATIONAL"

    def pre_operational(self):
        self.node.nmt.state = "PRE-OPERATIONAL"

    def reset(self):
        self.node.nmt.state = "RESET"

    # --- PDO --------------------------------------------------------------

    def setup_pdos(self, command_period=0.02):
        """
        Read the node's own mapping (it is the authority, not the EDS) and
        start transmitting both command RPDOs periodically.
        """
        self.node.tpdo.read()
        self.node.rpdo.read()
        for i in (1, 2):
            self.node.rpdo[i].start(command_period)

    def stop_pdos(self):
        for i in (1, 2):
            try:
                self.node.rpdo[i].stop()
            except Exception:
                pass

    def _rpdo_set(self, axis: int, controlword: int, target_units: int):
        pdo = self.node.rpdo[axis + 1]
        # Address the entries by index so this works whether or not the EDS
        # parameter names line up with what the node reports.
        pdo[self.axis_index(0x6040, axis)].raw = controlword
        pdo[self.axis_index(0x60FF, axis)].raw = target_units
        pdo.transmit()

    # --- CiA 402 state machine -------------------------------------------

    def statusword(self, axis: int) -> int:
        return self.sdo_read(self.axis_index(0x6041, axis))

    def state(self, axis: int) -> str:
        return decode_state(self.statusword(axis))

    def _command(self, axis: int, cw: int):
        self.sdo_write(self.axis_index(0x6040, axis), cw)

    def enable(self, axis: int, timeout=2.0) -> bool:
        """
        Walk SWITCH ON DISABLED -> READY -> SWITCHED ON -> OPERATION ENABLED,
        clearing a fault first if there is one. Each step is confirmed by
        reading the statusword back, because a drive that silently refuses a
        transition is the single most common integration bug.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            state = self.state(axis)
            if state == "OPERATION ENABLED":
                return True
            if state in ("FAULT", "FAULT REACTION ACTIVE"):
                self._command(axis, CW_DISABLE_VOLTAGE)
                time.sleep(0.02)
                self._command(axis, CW_FAULT_RESET)   # rising edge of bit 7
                time.sleep(0.05)
                self._command(axis, CW_DISABLE_VOLTAGE)
            elif state == "SWITCH ON DISABLED":
                self._command(axis, CW_SHUTDOWN)
            elif state == "READY TO SWITCH ON":
                self._command(axis, CW_SWITCH_ON)
            elif state in ("SWITCHED ON", "QUICK STOP ACTIVE"):
                self._command(axis, CW_ENABLE_OPERATION)
            time.sleep(0.02)
        return False

    def disable(self, axis: int):
        self._command(axis, CW_DISABLE_VOLTAGE)

    def quick_stop(self, axis: int):
        self._command(axis, CW_QUICK_STOP)

    def reset_fault(self, axis: int):
        self._command(axis, CW_DISABLE_VOLTAGE)
        time.sleep(0.02)
        self._command(axis, CW_FAULT_RESET)
        time.sleep(0.05)
        self._command(axis, CW_DISABLE_VOLTAGE)

    def enable_all(self, timeout=2.0) -> bool:
        return all(self.enable(a, timeout) for a in (LEFT, RIGHT))

    def disable_all(self):
        for a in (LEFT, RIGHT):
            self.disable(a)

    # --- motion -----------------------------------------------------------

    def set_wheel_speeds(self, left_mps: float, right_mps: float, use_pdo=True):
        left = self.geometry.mps_to_units(left_mps)
        right = self.geometry.mps_to_units(right_mps)
        if use_pdo:
            self._rpdo_set(LEFT, CW_ENABLE_OPERATION, left)
            self._rpdo_set(RIGHT, CW_ENABLE_OPERATION, right)
        else:
            self.sdo_write(self.axis_index(0x60FF, LEFT), left)
            self.sdo_write(self.axis_index(0x60FF, RIGHT), right)

    def drive(self, linear_mps: float, angular_rps: float, use_pdo=True):
        left, right = self.geometry.twist_to_wheels(linear_mps, angular_rps)
        self.set_wheel_speeds(left, right, use_pdo=use_pdo)

    def stop(self, use_pdo=True):
        self.set_wheel_speeds(0.0, 0.0, use_pdo=use_pdo)

    # --- telemetry --------------------------------------------------------

    def velocity(self, axis: int) -> int:
        return self.sdo_read(self.axis_index(0x606C, axis))

    def tpdo_snapshot(self, timeout=1.0) -> dict:
        """Wait for one fresh TPDO from each axis and return what it carried."""
        out = {}
        for i, axis in ((1, LEFT), (2, RIGHT)):
            try:
                self.node.tpdo[i].wait_for_reception(timeout)
                sw = self.node.tpdo[i][self.axis_index(0x6041, axis)].raw
                vel = self.node.tpdo[i][self.axis_index(0x606C, axis)].raw
                out[axis] = {"statusword": sw, "state": decode_state(sw),
                             "velocity": vel,
                             "mps": self.geometry.units_to_mps(vel)}
            except Exception as exc:
                out[axis] = {"error": str(exc)}
        return out

    # --- emergencies ------------------------------------------------------

    def on_emcy(self, cb: Callable):
        self._emcy_cb = cb

    def _on_emcy(self, emcy):
        text = EMCY_NAMES.get(emcy.code, "unknown")
        line = (f"EMCY 0x{emcy.code:04X} ({text})  "
                f"register 0x{emcy.register:02X}  "
                f"data {emcy.data.hex(' ')}")
        if self._emcy_cb:
            self._emcy_cb(emcy, line)
        else:
            print(line, file=sys.stderr)

    def error_history(self) -> list[tuple[int, int]]:
        """Object 1003h: (error code, manufacturer info), newest first."""
        count = self.node.sdo[0x1003][0].raw
        out = []
        for i in range(1, count + 1):
            v = self.node.sdo[0x1003][i].raw
            out.append((v & 0xFFFF, (v >> 16) & 0xFFFF))
        return out

    def clear_error_history(self):
        self.node.sdo[0x1003][0].raw = 0

    # --- identity and diagnostics ----------------------------------------

    def info(self) -> dict:
        sdo = self.node.sdo
        return {
            "device_type": sdo[0x1000].raw,
            "device_name": sdo[0x1008].raw,
            "hw_version": sdo[0x1009].raw,
            "sw_version": sdo[0x100A].raw,
            "vendor_id": sdo[0x1018][1].raw,
            "product_code": sdo[0x1018][2].raw,
            "revision": sdo[0x1018][3].raw,
            "serial": sdo[0x1018][4].raw,
            "error_register": sdo[0x1001].raw,
            "heartbeat_ms": sdo[0x1017].raw,
            "supported_modes": sdo[0x6502].raw,
            "fw_slot": sdo[0x2100][6].raw,
            "fw_version": sdo[0x2100][7].raw,
        }

    def diagnostics(self) -> dict:
        d = self.node.sdo[0x2003]
        return {
            "can_state": d[1].raw,
            "tx_errors": d[2].raw,
            "rx_errors": d[3].raw,
            "bus_off_count": d[4].raw,
            "rx_overruns": d[5].raw,
            "uptime_s": d[6].raw,
            "free_heap": d[7].raw,
            "supply_mv": self.node.sdo[0x2004][3].raw,
        }

    def save_parameters(self):
        """1010h:01 - the signature is the ASCII string 'save'."""
        self.node.sdo[0x1010][1].raw = struct.unpack("<I", b"save")[0]

    def restore_defaults(self):
        """1011h:01 - 'load'."""
        self.node.sdo[0x1011][1].raw = struct.unpack("<I", b"load")[0]


# ---------------------------------------------------------------------------
# command line
# ---------------------------------------------------------------------------

def add_common(p):
    p.add_argument("--channel", default="can0")
    p.add_argument("--interface", default="socketcan",
                   help="python-can interface: socketcan, slcan, pcan, ...")
    p.add_argument("--bitrate", type=int, default=500000)
    p.add_argument("--node", type=lambda s: int(s, 0), default=0x10)
    p.add_argument("--eds", default=str(EDS_PATH))
    p.add_argument("--wheel-radius", type=float, default=0.0325)
    p.add_argument("--wheel-base", type=float, default=0.150)


def connect(args) -> RobocarDrive:
    return RobocarDrive(channel=args.channel, interface=args.interface,
                        node_id=args.node, bitrate=args.bitrate, eds=args.eds,
                        geometry=Geometry(args.wheel_radius, args.wheel_base))


def cmd_info(args):
    with connect(args) as d:
        info = d.info()
        print(f"node 0x{args.node:02X} on {args.channel}")
        print(f"  {info['device_name']}  hw {info['hw_version']}  sw {info['sw_version']}")
        print(f"  device type   0x{info['device_type']:08X} "
              f"(profile {info['device_type'] & 0xFFFF}, type {info['device_type'] >> 16})")
        print(f"  identity      vendor 0x{info['vendor_id']:08X} "
              f"product 0x{info['product_code']:08X} "
              f"rev 0x{info['revision']:08X} serial 0x{info['serial']:08X}")
        print(f"  firmware slot {info['fw_slot']} version {info['fw_version']}")
        print(f"  heartbeat     {info['heartbeat_ms']} ms")
        print(f"  error reg     0x{info['error_register']:02X}")
        print(f"  drive modes   0x{info['supported_modes']:08X} "
              f"({'pv ' if info['supported_modes'] & 0x04 else ''})")
        for axis, name in ((LEFT, "axis 1 (left) "), (RIGHT, "axis 2 (right)")):
            sw = d.statusword(axis)
            print(f"  {name}  statusword 0x{sw:04X}  {decode_state(sw)}")
        diag = d.diagnostics()
        print(f"  bus           state {diag['can_state']} "
              f"tx_err {diag['tx_errors']} rx_err {diag['rx_errors']} "
              f"bus-off {diag['bus_off_count']} overruns {diag['rx_overruns']}")
        print(f"  system        uptime {diag['uptime_s']} s  "
              f"heap {diag['free_heap']} B  supply {diag['supply_mv']} mV")
        hist = d.error_history()
        if hist:
            print("  error history (newest first):")
            for code, extra in hist:
                print(f"    0x{code:04X}  {EMCY_NAMES.get(code, 'unknown')}  info 0x{extra:04X}")


def cmd_enable(args):
    with connect(args) as d:
        d.start()
        ok = d.enable_all()
        for axis, name in ((LEFT, "left"), (RIGHT, "right")):
            print(f"{name}: {d.state(axis)}")
        sys.exit(0 if ok else 1)


def cmd_disable(args):
    with connect(args) as d:
        d.disable_all()
        for axis, name in ((LEFT, "left"), (RIGHT, "right")):
            print(f"{name}: {d.state(axis)}")


def cmd_reset_fault(args):
    with connect(args) as d:
        for axis in (LEFT, RIGHT):
            d.reset_fault(axis)
            print(f"axis {axis + 1}: {d.state(axis)}")


def cmd_drive(args):
    with connect(args) as d:
        d.start()
        if not d.enable_all():
            sys.exit("could not reach OPERATION ENABLED; try 'reset-fault' or check the supply")
        d.setup_pdos()
        print(f"driving linear={args.linear} m/s angular={args.angular} rad/s "
              f"for {args.seconds} s  (ctrl-c to stop)")
        t_end = time.time() + args.seconds
        try:
            while time.time() < t_end:
                d.drive(args.linear, args.angular)
                snap = d.tpdo_snapshot(timeout=0.2)
                left = snap.get(LEFT, {})
                right = snap.get(RIGHT, {})
                print(f"\r  L {left.get('mps', 0):+.3f} m/s  "
                      f"R {right.get('mps', 0):+.3f} m/s  "
                      f"[{left.get('state', '?')}]      ", end="", flush=True)
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            print()
            d.stop()
            time.sleep(0.2)
            d.stop_pdos()
            d.disable_all()
            print("stopped and disabled")


def cmd_stop(args):
    with connect(args) as d:
        for axis in (LEFT, RIGHT):
            d.quick_stop(axis)
        time.sleep(0.3)
        d.disable_all()
        print("quick stop issued, drive disabled")


def cmd_monitor(args):
    with connect(args) as d:
        d.start()
        d.setup_pdos(command_period=0)     # listen only; do not command
        d.stop_pdos()
        print("listening (ctrl-c to quit)")
        try:
            while True:
                snap = d.tpdo_snapshot(timeout=1.0)
                parts = []
                for axis, name in ((LEFT, "L"), (RIGHT, "R")):
                    s = snap.get(axis, {})
                    if "error" in s:
                        parts.append(f"{name} --")
                    else:
                        parts.append(f"{name} {s['mps']:+.3f} m/s {s['state']}")
                print("  ".join(parts))
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nbye")


def cmd_config(args):
    with connect(args) as d:
        if args.show:
            for axis in (LEFT, RIGHT):
                base = 0x2000 + axis
                rec = d.node.sdo[base]
                print(f"motor {axis + 1} (0x{base:04X}):")
                print(f"  pwm_freq         {rec[1].raw} Hz")
                print(f"  encoder_cpr      {rec[2].raw}")
                print(f"  gear_ratio       {rec[3].raw / 1000:.3f}")
                print(f"  invert           {rec[4].raw}")
                print(f"  use_encoder      {rec[5].raw}")
                print(f"  kp / ki          {rec[6].raw / 1000:.3f} / {rec[7].raw / 1000:.3f}")
                print(f"  no_load_speed    {rec[8].raw} (0.1 rpm)")
                a = d.axis_index(0x6083, axis)
                print(f"  accel / decel    {d.sdo_read(a)} / {d.sdo_read(a + 1)} units/s")
                print(f"  quick stop decel {d.sdo_read(d.axis_index(0x6085, axis))} units/s")
                print(f"  max velocity     {d.sdo_read(d.axis_index(0x607F, axis))} (0.1 rpm)")
            p = d.node.sdo[0x2002]
            print("protection (0x2002):")
            print(f"  overcurrent      {p[1].raw} mA")
            print(f"  stall            {p[2].raw} mA for {p[3].raw} ms")
            print(f"  voltage window   {p[4].raw}..{p[5].raw} mV")
            print(f"  command timeout  {p[6].raw} ms")
            return

        if args.set:
            for item in args.set:
                path, _, value = item.partition("=")
                idx, _, sub = path.partition(":")
                index = int(idx, 0)
                subidx = int(sub, 0) if sub else 0
                d.sdo_write(index, int(value, 0), subidx)
                print(f"0x{index:04X}:{subidx:02X} = {int(value, 0)}")

        if args.save:
            d.save_parameters()
            print("parameters stored to NVS")
        if args.restore:
            d.restore_defaults()
            print("defaults restored (effective after reset)")


def cmd_scan(args):
    """Find every node that answers, by reading 1000h from each id."""
    net = canopen.Network()
    try:
        net.connect(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    except TypeError:
        net.connect(bustype=args.interface, channel=args.channel, bitrate=args.bitrate)
    try:
        net.scanner.search()
        time.sleep(1.0)
        if not net.scanner.nodes:
            print("no nodes answered")
        for nid in sorted(net.scanner.nodes):
            node = net.add_node(nid, str(EDS_PATH))
            try:
                dt = node.sdo[0x1000].raw
                name = node.sdo[0x1008].raw
                print(f"  node 0x{nid:02X} ({nid:3d})  device type 0x{dt:08X}  {name}")
            except Exception as exc:
                print(f"  node 0x{nid:02X} ({nid:3d})  (no SDO response: {exc})")
    finally:
        net.disconnect()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    for name, fn, help_text in [
        ("info", cmd_info, "identity, state, diagnostics and error history"),
        ("enable", cmd_enable, "walk both axes to OPERATION ENABLED"),
        ("disable", cmd_disable, "disable voltage on both axes"),
        ("reset-fault", cmd_reset_fault, "clear a latched fault"),
        ("stop", cmd_stop, "quick stop, then disable"),
        ("monitor", cmd_monitor, "print TPDO telemetry continuously"),
        ("scan", cmd_scan, "list nodes on the bus"),
    ]:
        p = sub.add_parser(name, help=help_text)
        add_common(p)
        p.set_defaults(func=fn)

    p = sub.add_parser("drive", help="drive for a fixed time")
    add_common(p)
    p.add_argument("--linear", type=float, default=0.2, help="m/s")
    p.add_argument("--angular", type=float, default=0.0, help="rad/s")
    p.add_argument("--seconds", type=float, default=2.0)
    p.set_defaults(func=cmd_drive)

    p = sub.add_parser("config", help="read or write configuration objects")
    add_common(p)
    p.add_argument("--show", action="store_true")
    p.add_argument("--set", action="append", metavar="IDX[:SUB]=VALUE",
                   help="e.g. --set 0x6083=8000 --set 0x2002:1=3000")
    p.add_argument("--save", action="store_true", help="persist via 1010h")
    p.add_argument("--restore", action="store_true", help="defaults via 1011h")
    p.set_defaults(func=cmd_config)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
