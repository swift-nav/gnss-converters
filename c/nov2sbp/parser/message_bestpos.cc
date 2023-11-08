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

#include "message_bestpos.h"

#include <cassert>

#include "read_little_endian.h"

namespace Novatel {

void Message::BESTPOS_t::FromBytes(const uint8_t *bytes, size_t n_bytes) {
  (void)n_bytes;
  size_t i = 0;
  sol_stat = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  pos_type = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  lat = Util::read_le_double(bytes + i);
  i += sizeof(double);
  lon = Util::read_le_double(bytes + i);
  i += sizeof(double);
  hgt = Util::read_le_double(bytes + i);
  i += sizeof(double);
  undulation = Util::read_le_float(bytes + i);
  i += sizeof(float);
  datum_id = Util::read_le_uint32(bytes + i);
  i += sizeof(uint32_t);
  lat_stddev = Util::read_le_float(bytes + i);
  i += sizeof(float);
  lon_stddev = Util::read_le_float(bytes + i);
  i += sizeof(float);
  hgt_stddev = Util::read_le_float(bytes + i);
  i += sizeof(float);

  for (size_t j = 0; j < sizeof(stn_id); ++j) {  // NOLINT
    stn_id[j] = bytes[i++];
  }

  diff_age = Util::read_le_float(bytes + i);
  i += sizeof(float);
  sol_age = Util::read_le_float(bytes + i);
  i += sizeof(float);
  num_sv_tracked = bytes[i++];
  num_sv_in_soln = bytes[i++];
  num_sv_in_soln_L1 = bytes[i++];
  num_sv_in_soln_multi = bytes[i++];
  reserved = bytes[i++];
  ext_sol_stat = bytes[i++];
  galileo_beidou_mask = bytes[i++];
  gps_glonass_mask = bytes[i++];

  (void)i;
  assert(i == n_bytes);
}
}  // namespace Novatel
