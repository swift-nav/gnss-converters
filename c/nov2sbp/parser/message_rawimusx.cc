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

#include "message_rawimusx.h"
#include "read_little_endian.h"

#include <cassert>

namespace Novatel {

void Message::RAWIMUSX_t::FromBytes(const uint8_t *bytes, size_t n_bytes) {
  (void)n_bytes;
  size_t i = 0;
  imu_info = bytes[i++];
  imu_type = bytes[i++];
  gnss_week = Util::read_le_uint16(bytes + i);
  i += sizeof(uint16_t);
  gnss_week_seconds = Util::read_le_double(bytes + i);
  i += sizeof(double);
  imu_status = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  z_accel = Util::read_le_int32(bytes + i);
  i += sizeof(int32_t);
  y_accel = -Util::read_le_int32(bytes + i);
  i += sizeof(int32_t);
  x_accel = Util::read_le_int32(bytes + i);
  i += sizeof(int32_t);
  z_gyro = Util::read_le_int32(bytes + i);
  i += sizeof(int32_t);
  y_gyro = -Util::read_le_int32(bytes + i);
  i += sizeof(int32_t);
  x_gyro = Util::read_le_int32(bytes + i);
  i += sizeof(int32_t);

  (void)i;
  assert(i == n_bytes);
}
}  // namespace Novatel
