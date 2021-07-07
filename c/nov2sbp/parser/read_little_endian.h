/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef NOVATEL_PARSER_READ_LITTLE_ENDIAN_H_
#define NOVATEL_PARSER_READ_LITTLE_ENDIAN_H_

#if HAVE_BYTESWAP_H
#include <byteswap.h>
#else
#define bswap_16(value) ((((value)&0xff) << 8) | ((value) >> 8))

#define bswap_32(value)                                     \
  (((uint32_t)bswap_16((uint16_t)((value)&0xffff)) << 16) | \
   (uint32_t)bswap_16((uint16_t)((value) >> 16)))

#define bswap_64(value)                                         \
  (((uint64_t)bswap_32((uint32_t)((value)&0xffffffff)) << 32) | \
   (uint64_t)bswap_32((uint32_t)((value) >> 32)))
#endif

#include <cassert>
#include <cstddef>
#include <cstdint>

namespace Novatel {

namespace Util {

static const bool kIsBigEndianHost =
    // NOLINTNEXTLINE
    ((*reinterpret_cast<const uint16_t *>("\0\xff")) < 0x100);
static const size_t kBitsPerByte = 8;

static inline uint16_t _le16toh(uint16_t le);
static inline uint32_t _le32toh(uint32_t le);
static inline uint64_t _le64toh(uint64_t le);

static inline uint16_t read_le_uint16(const uint8_t *bytes) {
  // NOLINTNEXTLINE
  return _le16toh(*reinterpret_cast<const uint16_t *>(bytes));
}

static inline uint32_t read_le_uint32(const uint8_t *bytes) {
  // NOLINTNEXTLINE
  return _le32toh(*reinterpret_cast<const uint32_t *>(bytes));
}

static inline int32_t read_le_int32(const uint8_t *bytes) {
  // NOLINTNEXTLINE
  uint32_t swapped = _le32toh(*reinterpret_cast<const uint32_t *>(bytes));
  // NOLINTNEXTLINE
  return *reinterpret_cast<int32_t *>(&swapped);
}

static inline double read_le_double(const uint8_t *bytes) {
  static_assert(sizeof(double) == sizeof(uint64_t),
                "Bad assumption on double size.");
  // NOLINTNEXTLINE
  uint64_t swapped = _le64toh(*reinterpret_cast<const uint64_t *>(bytes));
  // NOLINTNEXTLINE
  return *reinterpret_cast<double *>(&swapped);
}

static inline float read_le_float(const uint8_t *bytes) {
  static_assert(sizeof(float) == sizeof(uint32_t),
                "Bad assumption on float size.");
  // NOLINTNEXTLINE
  uint32_t swapped = _le32toh(*reinterpret_cast<const uint32_t *>(bytes));
  // NOLINTNEXTLINE
  return *reinterpret_cast<float *>(&swapped);
}

static inline uint64_t read_le_bits64(const uint8_t *bytes,
                                      size_t bit_offset,
                                      size_t n_bits) {
  assert(n_bits <= 64);
  uint64_t val = 0;
  for (size_t i = 0; i < n_bits; ++i) {
    const size_t byte_index = (bit_offset + i) / kBitsPerByte;
    const size_t bit_index = (bit_offset + i) % kBitsPerByte;
    val |= (((bytes[byte_index] & (1 << bit_index)) != 0) ? (1LL << i) : 0LL);
  }
  return val;
}

static inline uint8_t read_le_bits8(const uint8_t *bytes,
                                    size_t bit_offset,
                                    size_t n_bits) {
  assert(n_bits <= 8);
  return static_cast<uint8_t>(read_le_bits64(bytes, bit_offset, n_bits));
}

static inline uint16_t read_le_bits16(const uint8_t *bytes,
                                      size_t bit_offset,
                                      size_t n_bits) {
  assert(n_bits <= 16);
  return static_cast<uint16_t>(read_le_bits64(bytes, bit_offset, n_bits));
}

static inline uint32_t read_le_bits32(const uint8_t *bytes,
                                      size_t bit_offset,
                                      size_t n_bits) {
  assert(n_bits <= 32);
  return static_cast<uint32_t>(read_le_bits64(bytes, bit_offset, n_bits));
}

static inline uint16_t _le16toh(uint16_t le) {
  if (kIsBigEndianHost) {
    return bswap_16(le);
  }
  return le;
}

static inline uint32_t _le32toh(uint32_t le) {
  if (kIsBigEndianHost) {
    return bswap_32(le);
  }
  return le;
}

static inline uint64_t _le64toh(uint64_t le) {
  if (kIsBigEndianHost) {
    return bswap_64(le);
  }
  return le;
}

}  // namespace Util

}  // namespace Novatel

#endif
