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

#include "message_insatt.h"
#include "read_little_endian.h"

#include <cassert>

namespace Novatel {

void Message::INSATT_t::FromBytes(const uint8_t *bytes, size_t n_bytes) {
  (void)n_bytes;
  size_t i = 0;
  gnss_week = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  tow = Util::read_le_double(bytes + i);
  i += sizeof(double);
  roll = Util::read_le_double(bytes + i);
  i += sizeof(double);
  pitch = Util::read_le_double(bytes + i);
  i += sizeof(double);
  azimuth = Util::read_le_double(bytes + i);
  i += sizeof(double);
  status = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);

  (void)i;
  assert(i == n_bytes);
}
}  // namespace Novatel
