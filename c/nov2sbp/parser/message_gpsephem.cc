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

#include "message_gpsephem.h"

#include <cassert>

#include "read_little_endian.h"

namespace Novatel {

void Message::GPSEPHEM_t::FromBytes(const uint8_t *bytes, size_t n_bytes) {
  (void)n_bytes;
  size_t i = 0;

  prn = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  tow = Util::read_le_double(bytes + i);
  i += sizeof(double);
  health = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  iode1 = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  iode2 = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  week = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  z_week = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  toe = Util::read_le_double(bytes + i);
  i += sizeof(double);
  A = Util::read_le_double(bytes + i);
  i += sizeof(double);
  delta_N = Util::read_le_double(bytes + i);
  i += sizeof(double);
  M0 = Util::read_le_double(bytes + i);
  i += sizeof(double);
  ecc = Util::read_le_double(bytes + i);
  i += sizeof(double);
  omega = Util::read_le_double(bytes + i);
  i += sizeof(double);
  cuc = Util::read_le_double(bytes + i);
  i += sizeof(double);
  cus = Util::read_le_double(bytes + i);
  i += sizeof(double);
  crc = Util::read_le_double(bytes + i);
  i += sizeof(double);
  crs = Util::read_le_double(bytes + i);
  i += sizeof(double);
  cic = Util::read_le_double(bytes + i);
  i += sizeof(double);
  cis = Util::read_le_double(bytes + i);
  i += sizeof(double);
  I0 = Util::read_le_double(bytes + i);
  i += sizeof(double);
  I_dot = Util::read_le_double(bytes + i);
  i += sizeof(double);
  omega0 = Util::read_le_double(bytes + i);
  i += sizeof(double);
  omega_dot = Util::read_le_double(bytes + i);
  i += sizeof(double);
  iodc = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  toc = Util::read_le_double(bytes + i);
  i += sizeof(double);
  tgd = Util::read_le_double(bytes + i);
  i += sizeof(double);
  af0 = Util::read_le_double(bytes + i);
  i += sizeof(double);
  af1 = Util::read_le_double(bytes + i);
  i += sizeof(double);
  af2 = Util::read_le_double(bytes + i);
  i += sizeof(double);
  AS = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  N = Util::read_le_double(bytes + i);
  i += sizeof(double);
  URA = Util::read_le_double(bytes + i);
  i += sizeof(double);

  (void)i;
  assert(i == n_bytes);
}
}  // namespace Novatel
