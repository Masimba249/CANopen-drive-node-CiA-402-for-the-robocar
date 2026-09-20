#!/usr/bin/env python3
"""
Run the real firmware stack in this process and let a real CANopen master
talk to it over a python-can virtual bus. No ESP32, no CAN hardware.

    ./sim_node.py                 # scripted end-to-end session
    ./sim_node.py --hold          # keep the node alive, then use robocar.py
                                  # from the SAME process (see --hold notes)

What is real here: the object dictionary, the SDO server (including block
download), the PDO engine, NMT, heartbeat, EMCY, and the CiA 402 state
machine - all compiled from firmware/components/. What is simulated: the CAN
driver, a first-order motor model, and a RAM-backed firmware image.
"""
from __future__ import annotations

import argparse
import ctypes
import struct
import sys
import threading
import time
import zlib
from pathlib import Path

import can

ROOT = Path(__file__).resolve().parents[2]
LIB = ROOT / "host_tests" / "build" / "libcanopen_sim.so"
EDS = ROOT / "master" / "eds" / "robocar_drive.eds"

sys.path.insert(0, str(ROOT / "master"))


class SimulatedNode:
    """Pumps frames between the C stack and a python-can virtual bus."""

    def __init__(self, node_id=0x10, channel="robocar-sim"):
        if not LIB.exists():
            raise SystemExit(f"{LIB} not built. Run: make -C host_tests sim")

        self.lib = ctypes.CDLL(str(LIB))
        self.lib.sim_init.argtypes = [ctypes.c_uint8]
        self.lib.sim_rx.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8),
                                    ctypes.c_uint8]
        self.lib.sim_tick.argtypes = [ctypes.c_uint32]
        self.lib.sim_pop_tx.argtypes = [ctypes.POINTER(ctypes.c_uint32),
                                        ctypes.POINTER(ctypes.c_uint8),
                                        ctypes.POINTER(ctypes.c_uint8)]
        self.lib.sim_pop_tx.restype = ctypes.c_int
        self.lib.sim_statusword.restype = ctypes.c_uint16
        self.lib.sim_demand.restype = ctypes.c_int32
        self.lib.sim_image_len.restype = ctypes.c_uint32
        self.lib.sim_image_crc.restype = ctypes.c_uint32
        self.lib.sim_fota_status.restype = ctypes.c_uint8

        self.node_id = node_id
        self.channel = channel
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self.tx_count = 0
        self.rx_count = 0

    # --- lifecycle --------------------------------------------------------

    def start(self):
        with self._lock:
            self.lib.sim_init(self.node_id)
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        time.sleep(0.05)
        return self

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self.bus.shutdown()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # --- the pump ---------------------------------------------------------

    def _pump(self):
        buf = (ctypes.c_uint8 * 8)()
        cid = ctypes.c_uint32()
        dlc = ctypes.c_uint8()
        last = time.monotonic()

        while not self._stop.is_set():
            # bus -> node
            msg = self.bus.recv(timeout=0.001)
            while msg is not None:
                if not msg.is_extended_id:
                    data = (ctypes.c_uint8 * 8)(*(list(msg.data) + [0] * (8 - len(msg.data))))
                    with self._lock:
                        self.lib.sim_rx(msg.arbitration_id, data, len(msg.data))
                    self.rx_count += 1
                msg = self.bus.recv(timeout=0)

            # advance the node's clock in real time
            now = time.monotonic()
            dt_ms = int((now - last) * 1000)
            if dt_ms > 0:
                last = now
                with self._lock:
                    self.lib.sim_tick(min(dt_ms, 50))

            # node -> bus
            while True:
                with self._lock:
                    got = self.lib.sim_pop_tx(ctypes.byref(cid), buf, ctypes.byref(dlc))
                if not got:
                    break
                self.bus.send(can.Message(arbitration_id=cid.value,
                                          data=bytes(buf[:dlc.value]),
                                          is_extended_id=False))
                self.tx_count += 1

    # --- introspection ----------------------------------------------------

    def statusword(self, axis):  return self.lib.sim_statusword(axis)
    def demand(self, axis):      return self.lib.sim_demand(axis)
    def powered(self, axis):     return bool(self.lib.sim_powered(axis))
    def image_len(self):         return self.lib.sim_image_len()
    def image_crc(self):         return self.lib.sim_image_crc()
    def fota_status(self):       return self.lib.sim_fota_status()
    def inject_fault(self, axis): self.lib.sim_set_fault(axis)
    def inject_emcy(self, code, reg=0x80): self.lib.sim_raise_emcy(code, reg)


# ---------------------------------------------------------------------------
# scripted demo
# ---------------------------------------------------------------------------

def demo(node_id=0x10, verbose=True):
    from robocar import RobocarDrive, decode_state, LEFT, RIGHT  # noqa: E402

    failures = []

    def check(label, ok, detail=""):
        status = "ok  " if ok else "FAIL"
        print(f"  [{status}] {label}{('  ' + detail) if detail else ''}")
        if not ok:
            failures.append(label)

    with SimulatedNode(node_id=node_id) as sim:
        drive = RobocarDrive(channel="robocar-sim", interface="virtual",
                             node_id=node_id, eds=str(EDS))
        emcys = []
        drive.on_emcy(lambda e, line: emcys.append(e))

        try:
            print("\n--- identity (SDO uploads) ---")
            info = drive.info()
            check("1008h device name", info["device_name"] == "Robocar CiA402 Drive",
                  info["device_name"])
            check("1000h device type is a CiA 402 servo drive",
                  info["device_type"] == 0x00020192, f"0x{info['device_type']:08X}")
            check("100Ah software version", info["sw_version"] == "1.0.0-sim",
                  info["sw_version"])
            check("6502h reports profile velocity", info["supported_modes"] == 0x04)

            print("\n--- CiA 402 state machine (over SDO) ---")
            check("starts in SWITCH ON DISABLED",
                  drive.state(LEFT) == "SWITCH ON DISABLED", drive.state(LEFT))
            drive.start()
            ok = drive.enable_all()
            check("reaches OPERATION ENABLED on both axes", ok,
                  f"L={drive.state(LEFT)} R={drive.state(RIGHT)}")
            check("power stage energised", sim.powered(0) and sim.powered(1))

            print("\n--- PDO ---")
            drive.setup_pdos(command_period=0.02)
            drive.drive(linear_mps=0.25, angular_rps=0.0)
            time.sleep(0.6)
            snap = drive.tpdo_snapshot(timeout=1.0)
            left = snap.get(LEFT, {})
            check("TPDO1 carries statusword + velocity", "error" not in left, str(left))
            if "error" not in left:
                check("TPDO statusword decodes to OPERATION ENABLED",
                      left["state"] == "OPERATION ENABLED", left["state"])
                check("wheel is turning at roughly the commanded speed",
                      abs(left["mps"] - 0.25) < 0.05, f"{left['mps']:.3f} m/s")

            target_units = drive.geometry.mps_to_units(0.25)
            check("RPDO reached the drive (demand matches target)",
                  abs(sim.demand(0) - target_units) <= max(2, target_units // 20),
                  f"demand={sim.demand(0)} target={target_units}")

            print("\n--- quick stop and fault handling ---")
            drive.stop_pdos()
            drive.quick_stop(LEFT)
            time.sleep(0.4)
            check("quick stop leads to SWITCH ON DISABLED",
                  drive.state(LEFT) == "SWITCH ON DISABLED", drive.state(LEFT))

            emcys.clear()
            sim.inject_emcy(0xFF01, 0x80)      # motor stalled
            time.sleep(0.3)
            check("EMCY reaches the master", any(e.code == 0xFF01 for e in emcys),
                  f"{len(emcys)} received")
            hist = drive.error_history()
            check("1003h records the error", hist and hist[0][0] == 0xFF01, str(hist))

            sim.inject_fault(1)
            time.sleep(0.3)
            check("injected fault puts axis 2 in FAULT",
                  drive.state(RIGHT) == "FAULT", drive.state(RIGHT))
            drive.reset_fault(RIGHT)
            time.sleep(0.2)
            check("fault reset returns to SWITCH ON DISABLED",
                  drive.state(RIGHT) == "SWITCH ON DISABLED", drive.state(RIGHT))
            drive.clear_error_history()
            check("1003h cleared", drive.node.sdo[0x1003][0].raw == 0)

            print("\n--- SDO block download (the firmware update path) ---")
            drive.disable_all()
            time.sleep(0.2)
            image = bytes((i * 31 + (i >> 5)) & 0xFF for i in range(64 * 1024))
            crc = zlib.crc32(image) & 0xFFFFFFFF
            sdo = drive.node.sdo
            sdo[0x2100][1].raw = len(image)
            sdo[0x2100][2].raw = crc
            sdo[0x2100][3].raw = 1                       # BEGIN

            t0 = time.time()
            with sdo[0x2101].open("wb", size=len(image), block_transfer=True) as f:
                f.write(image)
            dt = time.time() - t0

            check("all bytes arrived", sim.image_len() == len(image),
                  f"{sim.image_len()} of {len(image)} in {dt:.2f} s")
            check("CRC-32 computed by the node matches", sim.image_crc() == crc,
                  f"0x{sim.image_crc():08X} vs 0x{crc:08X}")
            check("bytes received is reported in 2100h:05",
                  sdo[0x2100][5].raw == len(image))
            sdo[0x2100][3].raw = 2                       # ACTIVATE
            check("activation accepted", sim.fota_status() == 0x02,
                  f"status 0x{sim.fota_status():02X}")

            print("\n--- a corrupted image is rejected ---")
            sdo[0x2100][1].raw = len(image)
            sdo[0x2100][2].raw = crc ^ 0xFFFFFFFF        # wrong CRC on purpose
            sdo[0x2100][3].raw = 1
            with sdo[0x2101].open("wb", size=len(image), block_transfer=True) as f:
                f.write(image)
            rejected = False
            try:
                sdo[0x2100][3].raw = 2
            except Exception:
                rejected = True
            check("ACTIVATE aborts on a CRC mismatch", rejected,
                  f"status 0x{sim.fota_status():02X}")

            print("\n--- configuration and persistence ---")
            sdo[0x6083].raw = 8000
            check("6083h written and read back", sdo[0x6083].raw == 8000)
            drive.save_parameters()
            check("1010h store accepted", True)
            try:
                sdo[0x1010][1].raw = 0x12345678
                bad_sig_rejected = False
            except Exception:
                bad_sig_rejected = True
            check("1010h rejects a wrong signature", bad_sig_rejected)

            print(f"\nframes: {sim.tx_count} from the node, {sim.rx_count} to it")

        finally:
            drive.close()

    print()
    if failures:
        print(f"{len(failures)} check(s) failed:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("all checks passed")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--node", type=lambda s: int(s, 0), default=0x10)
    ap.add_argument("--hold", action="store_true",
                    help="run the node until interrupted instead of the demo")
    args = ap.parse_args()

    if args.hold:
        with SimulatedNode(node_id=args.node) as sim:
            print(f"simulated node 0x{args.node:02X} on virtual channel "
                  f"'robocar-sim'. Ctrl-C to stop.")
            print("NOTE: python-can's virtual bus is process-local, so a master "
                  "must run in this process. Import SimulatedNode instead of "
                  "launching robocar.py separately.")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
        return 0

    return demo(node_id=args.node)


if __name__ == "__main__":
    sys.exit(main())
