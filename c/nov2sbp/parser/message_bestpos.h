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

#ifndef NOVATEL_PARSER_MESSAGE_BESTPOS_H_
#define NOVATEL_PARSER_MESSAGE_BESTPOS_H_

#include <cstddef>
#include <cstdint>

namespace Novatel {
namespace Message {

struct BESTPOS_t {
  uint32_t sol_stat;
  uint32_t pos_type;
  double lat;
  double lon;
  double hgt;
  float undulation;
  uint32_t datum_id;
  float lat_stddev;
  float lon_stddev;
  float hgt_stddev;
  char stn_id[4];
  float diff_age;
  float sol_age;
  uint8_t num_sv_tracked;
  uint8_t num_sv_in_soln;
  uint8_t num_sv_in_soln_L1;
  uint8_t num_sv_in_soln_multi;
  uint8_t reserved;
  uint8_t ext_sol_stat;
  uint8_t galileo_beidou_mask;
  uint8_t gps_glonass_mask;

  void FromBytes(const uint8_t *bytes, size_t n_bytes);
};

}  // namespace Message
}  // namespace Novatel

#endif
