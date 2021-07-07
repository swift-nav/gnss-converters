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

#ifndef NOVATEL_PARSER_MESSAGE_GLOEPHEMERIS_H_
#define NOVATEL_PARSER_MESSAGE_GLOEPHEMERIS_H_

#include <cstddef>
#include <cstdint>

namespace Novatel {
namespace Message {

struct GLOEPHEMERIS_t {
  uint16_t sloto;
  uint16_t freqo;
  uint8_t sat_type;
  uint8_t reserved_1;
  uint16_t e_week;
  uint32_t e_time;
  uint32_t t_offset;
  uint16_t Nt;
  uint16_t reserved_2;
  uint32_t issue;
  uint32_t health;
  double pos_x;
  double pos_y;
  double pos_z;
  double vel_x;
  double vel_y;
  double vel_z;
  double LS_acc_x;
  double LS_acc_y;
  double LS_acc_z;
  double tau_n;
  double delta_tau_n;
  double gamma;
  uint32_t Tk;
  uint32_t P;
  uint32_t Ft;
  uint32_t age;
  uint32_t flags;

  void FromBytes(const uint8_t *bytes, size_t n_bytes);
};

}  // namespace Message
}  // namespace Novatel

#endif
