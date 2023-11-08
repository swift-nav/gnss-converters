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

#include "message_rangecmp.h"

#include <cassert>

#include "read_little_endian.h"

namespace Novatel {

/**
 * This is significantly trickier than the other message decoders because
 * all of the values are stored in bitfields.
 *
 * Constant numbers are taken from the manual:
 * https://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf
 *
 * Also of note, the signed values require some bitshift trickery to get the
 * proper sign extension.
 */
void Message::RANGECMP_t::FromBytes(const uint8_t *bytes, size_t n_bytes) {
  (void)n_bytes;
  assert(n_bytes >= sizeof(uint32_t));

  n_records = Util::read_le_uint32(bytes);
  bytes += sizeof(uint32_t);

  size_t n_record_bytes =
      n_records * Message::RANGECMP_record_t::kRecordBinarySize;
  (void)n_record_bytes;
  assert(n_bytes == sizeof(uint32_t) + n_record_bytes);

  if (n_records > MAX_CHANNELS) {
    n_records = MAX_CHANNELS;
  }

  for (size_t i = 0; i < n_records; ++i) {
    records[i].tracking_status = Novatel::Util::read_le_bits32(bytes, 0, 32);
    uint32_t doppler_bits = Novatel::Util::read_le_bits32(bytes, 32, 28);
    records[i].doppler =
        ((*reinterpret_cast<int32_t *>(&doppler_bits)) << 4) >> 4;  // NOLINT
    records[i].pseudorange = Novatel::Util::read_le_bits64(bytes, 60, 36);
    uint32_t ADR_bits = Novatel::Util::read_le_bits32(bytes, 96, 32);
    records[i].ADR = *reinterpret_cast<int32_t *>(&ADR_bits);  // NOLINT
    records[i].stddev_psr = Novatel::Util::read_le_bits8(bytes, 128, 4);
    records[i].stddev_adr = Novatel::Util::read_le_bits8(bytes, 132, 4);
    records[i].prn_slot = Novatel::Util::read_le_bits8(bytes, 136, 8);
    records[i].lock_time = Novatel::Util::read_le_bits32(bytes, 144, 21);
    records[i].CN0 = Novatel::Util::read_le_bits8(bytes, 165, 5);
    records[i].glo_freq_no = Novatel::Util::read_le_bits8(bytes, 170, 6);
    records[i].reserved = Novatel::Util::read_le_bits16(bytes, 176, 16);
    bytes += Message::RANGECMP_record_t::kRecordBinarySize;
  }
}
}  // namespace Novatel
