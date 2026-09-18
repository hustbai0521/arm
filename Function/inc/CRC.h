#ifndef __CRC_H
#define __CRC_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief CRC-CCITT lookup table
 */
extern const uint16_t crc_ccitt_table[256];

/**
 * @brief Update CRC with one byte
 * @param crc Previous CRC value
 * @param c Byte to add
 * @return New CRC value
 */
uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);

/**
 * @brief Compute CRC-CCITT for a buffer
 * @param crc Initial CRC value (usually 0)
 * @param buffer Data buffer
 * @param len Length of data
 * @return Computed CRC
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const* buffer, size_t len);

/**
 * @brief Unitree A1 CRC32 Core Algorithm (Word-based)
 * @param ptr Data pointer (must be cast to uint32_t*)
 * @param len Length in 32-bit words
 * @return Computed CRC32
 */
uint32_t crc32_core(uint32_t* ptr, uint32_t len);

#endif
