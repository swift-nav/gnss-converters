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

#ifndef NOVATEL_PARSER_MESSAGE_GPSEPHEM_H_
#define NOVATEL_PARSER_MESSAGE_GPSEPHEM_H_

#include <cstddef>
#include <cstdint>

namespace Novatel {
namespace Message {

struct GPSEPHEM_t {
  uint32_t prn;
  double tow;
  uint32_t health;
  uint32_t iode1;
  uint32_t iode2;
  uint32_t week;
  uint32_t z_week;
  double toe;
  double A;
  double delta_N;
  double M0;
  double ecc;
  double omega;
  double cuc;
  double cus;
  double crc;
  double crs;
  double cic;
  double cis;
  double I0;
  double I_dot;
  double omega0;
  double omega_dot;
  uint32_t iodc;
  double toc;
  double tgd;
  double af0;
  double af1;
  double af2;
  uint32_t AS;
  double N;
  double URA;

  void FromBytes(const uint8_t *bytes, size_t n_bytes);
};

}  // namespace Message
}  // namespace Novatel

#endif
