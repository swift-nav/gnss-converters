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

#include "crc_checker.h"

namespace Novatel {

/**
 * Implementation taken directly from the Novatel manual:
 *
 * https://www.novatel.com/assets/Documents/Manulas/om-20000129.pdf
 */
uint32_t CrcChecker::crc32_byte_value(uint8_t byte) {
  uint32_t crc_temp = byte;
  for (int j = 8; j > 0; j--) {
    if ((crc_temp & 1) != 0) {
      crc_temp = (crc_temp >> 1) ^ kCrc32Polynomial;
    } else {
      crc_temp >>= 1;
    }
  }
  return crc_temp;
}

uint32_t CrcChecker::get_crc(const uint8_t *bytes, uint32_t n_bytes) {
  uint32_t crc = 0;

  while (n_bytes-- != 0) {
    uint32_t temp1 = (crc >> 8) & 0x00FFFFFFL;
    uint32_t temp2 = crc32_byte_value((crc ^ *bytes++) & 0xFF);
    crc = temp1 ^ temp2;
  }

  return crc;
}

}  // namespace Novatel
