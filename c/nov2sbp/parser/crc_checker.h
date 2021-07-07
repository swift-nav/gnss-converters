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

#ifndef NOVATEL_PARSER_CRC_CHECKER_H_
#define NOVATEL_PARSER_CRC_CHECKER_H_

#include <cstdint>

namespace Novatel {

class CrcChecker {
 private:
  static constexpr uint32_t kCrc32Polynomial = 0xEDB88320;

  static uint32_t crc32_byte_value(uint8_t byte);

 public:
  static uint32_t get_crc(const uint8_t *bytes, uint32_t n_bytes);
};

}  // namespace Novatel

#endif
