#include "co_crc.h"

uint16_t co_crc16(uint16_t crc, const uint8_t *data, size_t len)
{
    /* Bitwise CRC-16/XMODEM. A table would be faster, but SDO block transfer
     * tops out around 7 kB/s on a 500 kbit/s bus, so this costs nothing. */
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint32_t co_crc32(uint32_t crc, const uint8_t *data, size_t len)
{
    /* Reflected CRC-32/ISO-HDLC, identical to zlib.crc32(). Bitwise so the
     * 1 kB table is not burned in IRAM; the FOTA path is I/O bound anyway. */
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc;
}
