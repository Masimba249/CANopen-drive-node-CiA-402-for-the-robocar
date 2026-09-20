# Firmware update over CAN

The node accepts a new application image over the fieldbus, verifies it,
boots it on probation, and reverts to the previous image if the new one never
reports back.

Firmware side:
[`firmware/components/fota/fota.c`](../firmware/components/fota/fota.c).
Master side: [`master/fota_client.py`](../master/fota_client.py).

## Protocol

| Step | Operation | What the node does |
|---|---|---|
| 1 | SDO write `2100h:01` = size | remembers how many bytes to expect |
| 2 | SDO write `2100h:02` = CRC-32 | zlib CRC of the whole image |
| 3 | SDO write `2100h:03` = 1 (BEGIN) | picks the spare OTA slot and erases it |
| 4 | SDO **block download** of `2101h` | streams into flash, 4 KiB at a time, CRC-32 running |
| 5 | SDO write `2100h:03` = 2 (ACTIVATE) | checks size and CRC, `esp_ota_end()`, sets the boot partition, reboots in 250 ms |
| 6 | — | node reboots into the new image in `PENDING_VERIFY` |
| 7 | SDO write `2100h:03` = 4 (CONFIRM) | `esp_ota_mark_app_valid_cancel_rollback()` |

Progress is readable throughout: `2100h:04` is the status and `2100h:05` the
byte count. `2100h:06` and `2100h:07` report which slot and version are
running, which is how the client proves the update actually took.

## Why block download

Segmented SDO carries 7 bytes per frame and needs a response frame for every
one of them. Block download carries the same 7 bytes per frame but
acknowledges once per sub-block of up to 127 frames, roughly halving the
frame count. A 900 KiB image at 500 kbit/s takes on the order of a minute
rather than two.

The server implementation is in
[`co_sdo.c`](../firmware/components/canopen/co_sdo.c). Two details are worth
knowing:

- Only **in-sequence** bytes are committed. A dropped frame makes the server
  acknowledge the last good sequence number, and the client resends from
  there — so a lost frame costs a sub-block, not the transfer.
- The **last 7 bytes are held back** until the "end block download" frame
  arrives, because only that frame says how many of those bytes are real data
  and how many are padding.

The CRC-16/XMODEM that CiA 301 specifies for block transfer protects the
transport. The CRC-32 in `2100h:02` protects the image end to end, and
`esp_ota_end()` additionally validates the ESP-IDF image header and its
SHA-256. Three independent checks, each catching something the others do not.

## Rollback

This is the part that makes the feature safe to use on a robot that is not on
your desk.

`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` makes the second-stage bootloader
mark a newly selected image `PENDING_VERIFY` on its first boot. If the
application does not call `esp_ota_mark_app_valid_cancel_rollback()`, the next
reset reverts to the previous slot.

`fota.c` turns that into a fieldbus-level check: on boot it notices the
`PENDING_VERIFY` state, reports status 3, and starts a countdown of
`2100h:08` seconds (60 by default). The only thing that stops the countdown is
a CONFIRM command arriving over CAN — which means the new image must have
booted, brought up TWAI at the right bit rate, answered NMT, and served an SDO
download. An image that boots but cannot talk to the bus fails all of that and
is rolled back automatically.

Try it:

```bash
master/fota_client.py new.bin --no-confirm
# ... wait 60 s ...
master/robocar.py info      # reports the previous version, from the other slot
```

## Safety interlocks

- BEGIN is refused (abort `0x08000022`) unless both axes are out of SWITCHED
  ON / OPERATION ENABLED / QUICK STOP ACTIVE. Erasing flash while the wheels
  are turning is not a thing this node will do.
- BEGIN is refused while an earlier update is still awaiting confirmation,
  so there is always a known-good image to fall back to.
- A non-sequential chunk aborts the transfer rather than leaving a hole in
  flash.
- More bytes than `2100h:01` promised aborts with `0x06070012`.
- The whole update happens in NMT PRE-OPERATIONAL or OPERATIONAL; the
  heartbeat keeps running throughout, so the master can tell the difference
  between "busy erasing" and "fell off the bus".

## Partition layout

```
0x00000  bootloader + partition table
0x09000  nvs        20 KB
0x0E000  otadata     8 KB
0x10000  ota_0    1920 KB
0x1F0000 ota_1    1920 KB
```

There is deliberately no `factory` partition: with two equal slots, whichever
one is not running is always available as the target, and the "previous"
image is always a real, recently working application rather than whatever was
flashed at the factory.

## Why not a hand-written bootloader

The ESP32 ROM loader and the ESP-IDF second-stage bootloader already
implement slot selection, image validation, secure boot and rollback, and they
live in a part of flash that a failed update cannot corrupt. Replacing them
with hand-written code would add a class of failure — a bricked node that
needs a cable and a person — without adding any capability this project
needs. Everything above the bootloader, which is where the interesting work
is, is implemented here: the transport, the object dictionary design, the
verification, the state machine, and the confirm/rollback policy.
