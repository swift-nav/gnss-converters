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

#ifndef NOVATEL_PARSER_MESSAGE_RAWIMUSX_H_
#define NOVATEL_PARSER_MESSAGE_RAWIMUSX_H_

#include <cstddef>
#include <cstdint>

namespace Novatel {
namespace Message {

struct RAWIMUSX_t {
  uint8_t imu_info;
  uint8_t imu_type;
  uint16_t gnss_week;
  double gnss_week_seconds;
  uint32_t imu_status;
  int32_t z_accel;
  int32_t y_accel;
  int32_t x_accel;
  int32_t z_gyro;
  int32_t y_gyro;
  int32_t x_gyro;

  void FromBytes(const uint8_t *bytes, size_t n_bytes);
};

}  // namespace Message
}  // namespace Novatel

#endif
