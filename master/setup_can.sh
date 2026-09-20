#!/usr/bin/env bash
# Bring up a CAN interface on the Raspberry Pi.
#
#   ./setup_can.sh                 # can0 at 500 kbit/s (MCP2515 HAT)
#   ./setup_can.sh can0 250000     # different bit rate
#   ./setup_can.sh vcan0           # virtual bus, for testing with no hardware
#
# For an MCP2515 HAT add this to /boot/firmware/config.txt and reboot first:
#
#   dtparam=spi=on
#   dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
#   dtoverlay=spi-bcm2835-overlay
#
# The oscillator value must match the crystal actually fitted to the HAT
# (8 MHz and 16 MHz are both common). Getting it wrong gives you a bus that
# looks alive but produces nothing but error frames - one of the classic
# afternoons lost to CAN.
#
# For a USB dongle that presents itself as slcan instead:
#
#   sudo slcand -o -c -s6 /dev/ttyUSB0 can0   # -s6 = 500 kbit/s
#   sudo ip link set can0 up
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-500000}"

if [[ "$IFACE" == vcan* ]]; then
    sudo modprobe vcan
    sudo ip link add dev "$IFACE" type vcan 2>/dev/null || true
    sudo ip link set up "$IFACE"
    echo "virtual CAN interface $IFACE is up"
    exit 0
fi

sudo ip link set "$IFACE" down 2>/dev/null || true
# txqueuelen matters: the default of 10 frames is easily overrun by an SDO
# block download and shows up as ENOBUFS in python-can.
sudo ip link set "$IFACE" up type can bitrate "$BITRATE" \
     restart-ms 100 berr-reporting on
sudo ip link set "$IFACE" txqueuelen 1000

echo "--- $IFACE ---"
ip -details -statistics link show "$IFACE"
