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

#ifndef NOVATEL_PARSER_MESSAGE_RANGECMP_H_
#define NOVATEL_PARSER_MESSAGE_RANGECMP_H_

#define MAX_CHANNELS 63

#include <array>
#include <cstddef>
#include <cstdint>

namespace Novatel {
namespace Message {

struct RANGECMP_record_t {
  static constexpr size_t kRecordBinarySize = 24;
  uint32_t tracking_status;
  int32_t doppler;
  uint64_t pseudorange;
  int32_t ADR;
  uint8_t stddev_psr;
  uint8_t stddev_adr;
  uint8_t prn_slot;
  uint32_t lock_time;
  uint8_t CN0;
  uint8_t glo_freq_no;
  uint16_t reserved;
};

struct RANGECMP_t {
  size_t n_records;
  std::array<RANGECMP_record_t, MAX_CHANNELS> records;

  void FromBytes(const uint8_t *bytes, size_t n_bytes);
};

}  // namespace Message
}  // namespace Novatel

#endif
