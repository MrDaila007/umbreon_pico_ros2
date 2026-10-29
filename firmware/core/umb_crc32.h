/* umb_crc32.h - IEEE 802.3 CRC32 (reflected, init/xorout 0xFFFFFFFF) */
#pragma once
#include <stddef.h>
#include <stdint.h>

uint32_t umb_crc32(const uint8_t *data, size_t len);
