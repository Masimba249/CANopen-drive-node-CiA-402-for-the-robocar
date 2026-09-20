/**
 * @file co_crc.h
 * @brief CRC-16/XMODEM (SDO block transfer) and CRC-32/ISO-HDLC (FOTA image).
 */
#ifndef CO_CRC_H
#define CO_CRC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * CRC-16/XMODEM: poly 0x1021, init 0x0000, no reflection, no final xor.
 * This is the CRC mandated by CiA 301 for SDO block transfer.
 */
uint16_t co_crc16(uint16_t crc, const uint8_t *data, size_t len);

/**
 * CRC-32/ISO-HDLC ("zlib crc32"): poly 0xEDB88320 reflected, init 0xFFFFFFFF,
 * final xor 0xFFFFFFFF. Used to verify a downloaded firmware image; matches
 * Python's `zlib.crc32`.
 *
 * Seed the first call with CO_CRC32_INIT and finalise with co_crc32_final().
 */
#define CO_CRC32_INIT 0xFFFFFFFFu
uint32_t co_crc32(uint32_t crc, const uint8_t *data, size_t len);
static inline uint32_t co_crc32_final(uint32_t crc) { return crc ^ 0xFFFFFFFFu; }

#ifdef __cplusplus
}
#endif
#endif /* CO_CRC_H */
