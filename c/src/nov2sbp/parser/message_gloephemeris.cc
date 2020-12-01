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

#include "message_gloephemeris.h"
#include "read_little_endian.h"

#include <cassert>

namespace Novatel {

void Message::GLOEPHEMERIS_t::FromBytes(const uint8_t *bytes, size_t n_bytes) {
  (void)n_bytes;
  size_t i = 0;

  // clang-format off
  sloto        = Util::read_le_uint16(bytes + i); i += sizeof(uint16_t);
  freqo        = Util::read_le_uint16(bytes + i); i += sizeof(uint16_t);
  sat_type     = bytes[i++];
  reserved_1   = bytes[i++];
  e_week       = Util::read_le_uint16(bytes + i); i += sizeof(uint16_t);
  e_time       = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  t_offset     = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  Nt           = Util::read_le_uint16(bytes + i); i += sizeof(uint16_t);
  reserved_2   = Util::read_le_uint16(bytes + i); i += sizeof(uint16_t);
  issue        = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  health       = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  pos_x        = Util::read_le_double(bytes + i); i += sizeof(double);
  pos_y        = Util::read_le_double(bytes + i); i += sizeof(double);
  pos_z        = Util::read_le_double(bytes + i); i += sizeof(double);
  vel_x        = Util::read_le_double(bytes + i); i += sizeof(double);
  vel_y        = Util::read_le_double(bytes + i); i += sizeof(double);
  vel_z        = Util::read_le_double(bytes + i); i += sizeof(double);
  LS_acc_x     = Util::read_le_double(bytes + i); i += sizeof(double);
  LS_acc_y     = Util::read_le_double(bytes + i); i += sizeof(double);
  LS_acc_z     = Util::read_le_double(bytes + i); i += sizeof(double);
  tau_n        = Util::read_le_double(bytes + i); i += sizeof(double);
  delta_tau_n  = Util::read_le_double(bytes + i); i += sizeof(double);
  gamma        = Util::read_le_double(bytes + i); i += sizeof(double);
  Tk           = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  P            = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  Ft           = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  age          = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  flags        = Util::read_le_uint32(bytes + i); i += sizeof(uint32_t);
  // clang-format on

  (void)i;
  assert(i == n_bytes);
}
}  // namespace Novatel
