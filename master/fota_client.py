#!/usr/bin/env python3
"""
Firmware update over CANopen.

    ./fota_client.py ../firmware/build/robocar_canopen_drive.bin
    ./fota_client.py image.bin --no-confirm     # demonstrate the rollback

Sequence (mirrors firmware/components/fota/fota.c):

    2100h:01 = size            how many bytes are coming
    2100h:02 = CRC-32          zlib CRC of the whole image
    2100h:03 = 1 (BEGIN)       node erases the spare OTA slot
    2101h    = <image>         SDO block download, 7 bytes per frame
    2100h:03 = 2 (ACTIVATE)    node checks size + CRC, sets boot slot, reboots
    ... node reboots, comes up in "pending verify" ...
    2100h:03 = 4 (CONFIRM)     marks the new image good

Skipping the last step (--no-confirm) is how you prove the rollback works:
the node waits 2100h:08 seconds, marks the image invalid and reboots into the
previous one.
"""
from __future__ import annotations

import argparse
import sys
import time
import zlib
from pathlib import Path

try:
    import canopen
except ImportError:  # pragma: no cover
    sys.exit("python-canopen is not installed:  pip install -r requirements.txt")

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robocar import RobocarDrive, EDS_PATH, decode_state, LEFT, RIGHT  # noqa: E402

CMD_BEGIN, CMD_ACTIVATE, CMD_ABORT, CMD_CONFIRM, CMD_REBOOT = 1, 2, 3, 4, 5

STATUS = {
    0x00: "idle",
    0x01: "receiving",
    0x02: "activated",
    0x03: "pending confirm",
    0x81: "error: wrong state / drive not safe",
    0x82: "error: size",
    0x83: "error: CRC",
    0x84: "error: flash",
}


def human(n: int) -> str:
    return f"{n / 1024:.1f} KiB" if n >= 1024 else f"{n} B"


def upload(drive: RobocarDrive, image: bytes, chunk_progress=True,
           use_block_transfer=True) -> None:
    """Stream the image into object 2101h."""
    sdo = drive.node.sdo
    size = len(image)
    crc = zlib.crc32(image) & 0xFFFFFFFF

    print(f"image: {human(size)}  CRC-32 0x{crc:08X}")

    sdo[0x2100][1].raw = size
    sdo[0x2100][2].raw = crc

    print("erasing target slot ...", end="", flush=True)
    t0 = time.time()
    sdo[0x2100][3].raw = CMD_BEGIN          # blocks until the erase finishes
    print(f" {time.time() - t0:.1f} s")

    status = sdo[0x2100][4].raw
    if status != 0x01:
        raise RuntimeError(f"node refused BEGIN: {STATUS.get(status, status)}")

    t0 = time.time()
    written = 0
    step = max(size // 40, 4096)
    next_report = step

    # Block transfer moves one acknowledge per 127 frames instead of per
    # frame, which roughly doubles throughput. Fall back if the client or
    # the server says no.
    try:
        stream = sdo[0x2101].open("wb", size=size, block_transfer=use_block_transfer)
    except Exception as exc:
        if not use_block_transfer:
            raise
        print(f"block transfer unavailable ({exc}); falling back to segmented")
        stream = sdo[0x2101].open("wb", size=size, block_transfer=False)

    with stream as f:
        for off in range(0, size, 1024):
            part = image[off:off + 1024]
            f.write(part)
            written += len(part)
            if chunk_progress and written >= next_report:
                next_report += step
                rate = written / max(time.time() - t0, 1e-6)
                pct = 100.0 * written / size
                print(f"\r  {pct:5.1f} %  {human(written)}  "
                      f"{rate / 1024:.1f} KiB/s", end="", flush=True)

    elapsed = time.time() - t0
    print(f"\r  100.0 %  {human(size)} in {elapsed:.1f} s "
          f"({size / elapsed / 1024:.1f} KiB/s)      ")

    got = sdo[0x2100][5].raw
    if got != size:
        raise RuntimeError(f"node received {got} of {size} bytes")


def activate(drive: RobocarDrive) -> None:
    sdo = drive.node.sdo
    print("verifying and activating ...")
    try:
        sdo[0x2100][3].raw = CMD_ACTIVATE
    except canopen.SdoAbortedError as exc:
        status = STATUS.get(sdo[0x2100][4].raw, "?")
        raise RuntimeError(f"activation refused: {exc} ({status})") from exc
    print("  accepted; node is rebooting")


def wait_for_reboot(drive: RobocarDrive, timeout=20.0) -> bool:
    """Wait for the boot-up message, then for the SDO server to answer."""
    print("waiting for boot-up ...", end="", flush=True)
    t0 = time.time()
    drive.node.nmt.state = "PRE-OPERATIONAL"    # harmless; primes the state
    while time.time() - t0 < timeout:
        try:
            _ = drive.node.sdo[0x1000].raw
            print(f" back after {time.time() - t0:.1f} s")
            return True
        except Exception:
            time.sleep(0.25)
    print(" timed out")
    return False


def confirm(drive: RobocarDrive) -> None:
    sdo = drive.node.sdo
    status = sdo[0x2100][4].raw
    if status != 0x03:
        raise RuntimeError(
            f"node is not awaiting confirmation (status {STATUS.get(status, status)})")
    sdo[0x2100][3].raw = CMD_CONFIRM
    print("confirmed; rollback cancelled")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", help="path to the .bin produced by idf.py build")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--node", type=lambda s: int(s, 0), default=0x10)
    ap.add_argument("--eds", default=str(EDS_PATH))
    ap.add_argument("--no-confirm", action="store_true",
                    help="skip the confirm step, so the node rolls back")
    ap.add_argument("--no-block", action="store_true",
                    help="use segmented SDO instead of block transfer")
    ap.add_argument("--sdo-timeout", type=float, default=3.0,
                    help="seconds; raise it if the flash erase is slow")
    args = ap.parse_args()

    image = Path(args.image).read_bytes()
    if len(image) < 1024:
        sys.exit("that file is too small to be a firmware image")
    # An ESP-IDF application image starts with the 0xE9 magic byte.
    if image[0] != 0xE9:
        sys.exit("that does not look like an ESP-IDF image (first byte is not 0xE9)")

    with RobocarDrive(channel=args.channel, interface=args.interface,
                      node_id=args.node, bitrate=args.bitrate, eds=args.eds) as drive:
        drive.node.sdo.RESPONSE_TIMEOUT = args.sdo_timeout

        before = drive.node.sdo[0x2100][7].raw
        slot_before = drive.node.sdo[0x2100][6].raw
        print(f"running {before} from slot {slot_before}")

        # The node refuses an update while a power stage is live; make sure
        # it is not, rather than finding out from an abort code.
        for axis, name in ((LEFT, "left"), (RIGHT, "right")):
            state = drive.state(axis)
            if state not in ("SWITCH ON DISABLED", "READY TO SWITCH ON",
                             "FAULT", "NOT READY TO SWITCH ON"):
                print(f"{name} axis is {state}; disabling it first")
                drive.disable(axis)
        time.sleep(0.2)

        try:
            upload(drive, image, use_block_transfer=not args.no_block)
            activate(drive)
        except Exception as exc:
            print(f"failed: {exc}", file=sys.stderr)
            try:
                drive.node.sdo[0x2100][3].raw = CMD_ABORT
            except Exception:
                pass
            sys.exit(1)

        if not wait_for_reboot(drive):
            sys.exit("node did not come back; it will roll back on its own")

        after = drive.node.sdo[0x2100][7].raw
        slot_after = drive.node.sdo[0x2100][6].raw
        status = drive.node.sdo[0x2100][4].raw
        print(f"now running {after} from slot {slot_after} "
              f"({STATUS.get(status, status)})")

        if args.no_confirm:
            timeout = drive.node.sdo[0x2100][8].raw
            print(f"--no-confirm: leaving it unconfirmed. In {timeout} s the node "
                  f"will mark this image invalid and reboot into {slot_before}.")
            return

        confirm(drive)
        print(f"update complete: {before} -> {after}")


if __name__ == "__main__":
    main()
