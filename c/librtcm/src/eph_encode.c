/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtcm3/eph_encode.h"
#include <assert.h>
#include <string.h>
#include "rtcm3/bits.h"

uint16_t rtcm3_encode_gps_eph(const rtcm_msg_eph *msg_1019, uint8_t buff[]) {
  assert(msg_1019);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1019);
  bit += 12;
  rtcm_setbitu(buff, bit, 6, msg_1019->sat_id);
  bit += 6;
  rtcm_setbitu(buff, bit, 10, msg_1019->wn);
  bit += 10;
  rtcm_setbitu(buff, bit, 4, msg_1019->ura);
  bit += 4;
  rtcm_setbitu(buff, bit, 2, msg_1019->data.kepler.codeL2);
  bit += 2;
  rtcm_setbits(buff, bit, 14, msg_1019->data.kepler.inc_dot);
  bit += 14;
  rtcm_setbitu(buff, bit, 8, msg_1019->data.kepler.iode);
  bit += 8;
  rtcm_setbitu(buff, bit, 16, msg_1019->toe);
  bit += 16;
  rtcm_setbits(buff, bit, 8, msg_1019->data.kepler.af2);
  bit += 8;
  rtcm_setbits(buff, bit, 16, msg_1019->data.kepler.af1);
  bit += 16;
  rtcm_setbits(buff, bit, 22, msg_1019->data.kepler.af0);
  bit += 22;
  rtcm_setbitu(buff, bit, 10, msg_1019->data.kepler.iodc);
  bit += 10;
  rtcm_setbits(buff, bit, 16, msg_1019->data.kepler.crs);
  bit += 16;
  rtcm_setbits(buff, bit, 16, msg_1019->data.kepler.dn);
  bit += 16;
  rtcm_setbits(buff, bit, 32, msg_1019->data.kepler.m0);
  bit += 32;
  rtcm_setbits(buff, bit, 16, msg_1019->data.kepler.cuc);
  bit += 16;
  rtcm_setbitu(buff, bit, 32, msg_1019->data.kepler.ecc);
  bit += 32;
  rtcm_setbits(buff, bit, 16, msg_1019->data.kepler.cus);
  bit += 16;
  rtcm_setbitu(buff, bit, 32, msg_1019->data.kepler.sqrta);
  bit += 32;
  rtcm_setbitu(buff, bit, 16, msg_1019->data.kepler.toc);
  bit += 16;
  rtcm_setbits(buff, bit, 16, msg_1019->data.kepler.cic);
  bit += 16;
  rtcm_setbits(buff, bit, 32, msg_1019->data.kepler.omega0);
  bit += 32;
  rtcm_setbits(buff, bit, 16, msg_1019->data.kepler.cis);
  bit += 16;
  rtcm_setbits(buff, bit, 32, msg_1019->data.kepler.inc);
  bit += 32;
  rtcm_setbits(buff, bit, 16, msg_1019->data.kepler.crc);
  bit += 16;
  rtcm_setbits(buff, bit, 32, msg_1019->data.kepler.w);
  bit += 32;
  rtcm_setbits(buff, bit, 24, msg_1019->data.kepler.omegadot);
  bit += 24;
  rtcm_setbits(buff, bit, 8, msg_1019->data.kepler.tgd.gps_s);
  bit += 8;
  rtcm_setbitu(buff, bit, 6, msg_1019->health_bits);
  bit += 6;
  rtcm_setbitu(buff, bit, 1, msg_1019->data.kepler.L2_data_bit);
  bit += 1;
  rtcm_setbitu(buff, bit, 1, msg_1019->fit_interval);
  bit += 1;

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_glo_eph(const rtcm_msg_eph *msg_1020, uint8_t buff[]) {
  assert(msg_1020);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1020);
  bit += 12;
  rtcm_setbitu(buff, bit, 6, msg_1020->sat_id);
  bit += 6;
  rtcm_setbitu(buff, bit, 5, msg_1020->data.glo.fcn);
  bit += 5;
  // Almanac health
  rtcm_setbitu(buff, bit, 1, 1);
  bit += 1;
  // Almanac health availability
  rtcm_setbitu(buff, bit, 1, 0);
  bit += 1;
  rtcm_setbitu(buff, bit, 2, msg_1020->fit_interval);
  bit += 2;
  // T_k
  rtcm_setbitu(buff, bit, 12, 0);
  bit += 12;
  rtcm_setbitu(buff, bit, 1, msg_1020->health_bits);
  bit += 1;
  // P2
  rtcm_setbitu(buff, bit, 1, 0);
  bit += 1;
  rtcm_setbitu(buff, bit, 7, msg_1020->data.glo.t_b);
  bit += 7;
  rtcm_set_sign_magnitude_bit(buff, bit, 24, msg_1020->data.glo.vel[0]);
  bit += 24;
  rtcm_set_sign_magnitude_bit(buff, bit, 27, msg_1020->data.glo.pos[0]);
  bit += 27;
  rtcm_set_sign_magnitude_bit(buff, bit, 5, msg_1020->data.glo.acc[0]);
  bit += 5;
  rtcm_set_sign_magnitude_bit(buff, bit, 24, msg_1020->data.glo.vel[1]);
  bit += 24;
  rtcm_set_sign_magnitude_bit(buff, bit, 27, msg_1020->data.glo.pos[1]);
  bit += 27;
  rtcm_set_sign_magnitude_bit(buff, bit, 5, msg_1020->data.glo.acc[1]);
  bit += 5;
  rtcm_set_sign_magnitude_bit(buff, bit, 24, msg_1020->data.glo.vel[2]);
  bit += 24;
  rtcm_set_sign_magnitude_bit(buff, bit, 27, msg_1020->data.glo.pos[2]);
  bit += 27;
  rtcm_set_sign_magnitude_bit(buff, bit, 5, msg_1020->data.glo.acc[2]);
  bit += 5;
  // P3
  rtcm_setbitu(buff, bit, 1, 0);
  bit += 1;
  rtcm_set_sign_magnitude_bit(buff, bit, 11, msg_1020->data.glo.gamma);
  bit += 11;
  // P
  rtcm_setbitu(buff, bit, 2, 0);
  bit += 2;
  rtcm_setbitu(buff, bit, 1, msg_1020->health_bits);
  bit += 1;
  rtcm_set_sign_magnitude_bit(buff, bit, 22, msg_1020->data.glo.tau);
  bit += 22;
  rtcm_set_sign_magnitude_bit(buff, bit, 5, msg_1020->data.glo.d_tau);
  bit += 5;
  // EN
  rtcm_setbitu(buff, bit, 5, 0);
  bit += 5;
  // P4
  rtcm_setbitu(buff, bit, 1, 0);
  bit += 1;
  rtcm_setbitu(buff, bit, 4, msg_1020->ura);
  bit += 4;
  // NT
  rtcm_setbitu(buff, bit, 11, 0);
  bit += 11;
  // M
  rtcm_setbitu(buff, bit, 2, 0);
  bit += 2;
  // Additional data
  rtcm_setbitu(buff, bit, 32, 0);
  bit += 32;
  rtcm_setbitu(buff, bit, 32, 0);
  bit += 32;
  rtcm_setbitu(buff, bit, 15, 0);
  bit += 15;

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_bds_eph(const rtcm_msg_eph *msg_1042, uint8_t buff[]) {
  assert(msg_1042);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1042);
  bit += 12;
  rtcm_setbitu(buff, bit, 6, msg_1042->sat_id);
  bit += 6;
  rtcm_setbitu(buff, bit, 13, msg_1042->wn);
  bit += 13;
  rtcm_setbitu(buff, bit, 4, msg_1042->ura);
  bit += 4;
  rtcm_setbits(buff, bit, 14, msg_1042->data.kepler.inc_dot);
  bit += 14;
  rtcm_setbitu(buff, bit, 5, msg_1042->data.kepler.iode);
  bit += 5;
  rtcm_setbitu(buff, bit, 17, msg_1042->data.kepler.toc);
  bit += 17;
  rtcm_setbits(buff, bit, 11, msg_1042->data.kepler.af2);
  bit += 11;
  rtcm_setbits(buff, bit, 22, msg_1042->data.kepler.af1);
  bit += 22;
  rtcm_setbits(buff, bit, 24, msg_1042->data.kepler.af0);
  bit += 24;
  rtcm_setbitu(buff, bit, 5, msg_1042->data.kepler.iodc);
  bit += 5;
  rtcm_setbits(buff, bit, 18, msg_1042->data.kepler.crs);
  bit += 18;
  rtcm_setbits(buff, bit, 16, msg_1042->data.kepler.dn);
  bit += 16;
  rtcm_setbits(buff, bit, 32, msg_1042->data.kepler.m0);
  bit += 32;
  rtcm_setbits(buff, bit, 18, msg_1042->data.kepler.cuc);
  bit += 18;
  rtcm_setbitu(buff, bit, 32, msg_1042->data.kepler.ecc);
  bit += 32;
  rtcm_setbits(buff, bit, 18, msg_1042->data.kepler.cus);
  bit += 18;
  rtcm_setbitu(buff, bit, 32, msg_1042->data.kepler.sqrta);
  bit += 32;
  rtcm_setbitu(buff, bit, 17, msg_1042->toe);
  bit += 17;
  rtcm_setbits(buff, bit, 18, msg_1042->data.kepler.cic);
  bit += 18;
  rtcm_setbits(buff, bit, 32, msg_1042->data.kepler.omega0);
  bit += 32;
  rtcm_setbits(buff, bit, 18, msg_1042->data.kepler.cis);
  bit += 18;
  rtcm_setbits(buff, bit, 32, msg_1042->data.kepler.inc);
  bit += 32;
  rtcm_setbits(buff, bit, 18, msg_1042->data.kepler.crc);
  bit += 18;
  rtcm_setbits(buff, bit, 32, msg_1042->data.kepler.w);
  bit += 32;
  rtcm_setbits(buff, bit, 24, msg_1042->data.kepler.omegadot);
  bit += 24;
  rtcm_setbits(buff, bit, 10, msg_1042->data.kepler.tgd.bds_s[0]);
  bit += 10;
  rtcm_setbits(buff, bit, 10, msg_1042->data.kepler.tgd.bds_s[1]);
  bit += 10;
  rtcm_setbitu(buff, bit, 1, msg_1042->health_bits);
  bit += 1;

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

/** Decode an RTCMv3 GAL (common part) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param msg_eph RTCM message struct
 * \return bit position in the RTCM frame
 */
static void rtcm3_encode_gal_eph_common(const rtcm_msg_eph *msg_eph,
                                        uint8_t buff[],
                                        uint16_t *bit) {
  assert(msg_eph);
  rtcm_setbitu(buff, *bit, 6, msg_eph->sat_id);
  *bit += 6;
  rtcm_setbitu(buff, *bit, 12, msg_eph->wn);
  *bit += 12;
  rtcm_setbitu(buff, *bit, 10, msg_eph->data.kepler.iode);
  *bit += 10;
  rtcm_setbitu(buff, *bit, 8, msg_eph->ura);
  *bit += 8;
  rtcm_setbits(buff, *bit, 14, msg_eph->data.kepler.inc_dot);
  *bit += 14;
  rtcm_setbitu(buff, *bit, 14, msg_eph->data.kepler.toc);
  *bit += 14;
  rtcm_setbits(buff, *bit, 6, msg_eph->data.kepler.af2);
  *bit += 6;
  rtcm_setbits(buff, *bit, 21, msg_eph->data.kepler.af1);
  *bit += 21;
  rtcm_setbits(buff, *bit, 31, msg_eph->data.kepler.af0);
  *bit += 31;
  rtcm_setbits(buff, *bit, 16, msg_eph->data.kepler.crs);
  *bit += 16;
  rtcm_setbits(buff, *bit, 16, msg_eph->data.kepler.dn);
  *bit += 16;
  rtcm_setbits(buff, *bit, 32, msg_eph->data.kepler.m0);
  *bit += 32;
  rtcm_setbits(buff, *bit, 16, msg_eph->data.kepler.cuc);
  *bit += 16;
  rtcm_setbitu(buff, *bit, 32, msg_eph->data.kepler.ecc);
  *bit += 32;
  rtcm_setbits(buff, *bit, 16, msg_eph->data.kepler.cus);
  *bit += 16;
  rtcm_setbitu(buff, *bit, 32, msg_eph->data.kepler.sqrta);
  *bit += 32;
  rtcm_setbitu(buff, *bit, 14, msg_eph->toe);
  *bit += 14;
  rtcm_setbits(buff, *bit, 16, msg_eph->data.kepler.cic);
  *bit += 16;
  rtcm_setbits(buff, *bit, 32, msg_eph->data.kepler.omega0);
  *bit += 32;
  rtcm_setbits(buff, *bit, 16, msg_eph->data.kepler.cis);
  *bit += 16;
  rtcm_setbits(buff, *bit, 32, msg_eph->data.kepler.inc);
  *bit += 32;
  rtcm_setbits(buff, *bit, 16, msg_eph->data.kepler.crc);
  *bit += 16;
  rtcm_setbits(buff, *bit, 32, msg_eph->data.kepler.w);
  *bit += 32;
  rtcm_setbits(buff, *bit, 24, msg_eph->data.kepler.omegadot);
  *bit += 24;
}

/** Decode an RTCMv3 GAL (I/NAV message) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
uint16_t rtcm3_encode_gal_eph_inav(const rtcm_msg_eph *msg_eph,
                                   uint8_t buff[]) {
  assert(msg_eph);

  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1046);
  bit += 12;

  /* parse common I/NAV and F/NAV part */
  rtcm3_encode_gal_eph_common(msg_eph, buff, &bit);

  rtcm_setbits(buff, bit, 10, msg_eph->data.kepler.tgd.gal_s[0]);
  bit += 10;
  rtcm_setbits(buff, bit, 10, msg_eph->data.kepler.tgd.gal_s[1]);
  bit += 10;
  rtcm_setbits(buff, bit, 6, msg_eph->health_bits);
  bit += 6;
  /* reserved */
  rtcm_setbits(buff, bit, 2, 0);
  bit += 2;

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

/** Decode an RTCMv3 GAL (F/NAV message) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
uint16_t rtcm3_encode_gal_eph_fnav(const rtcm_msg_eph *msg_eph,
                                   uint8_t buff[]) {
  assert(msg_eph);

  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1045);
  bit += 12;

  /* parse common F/NAV and I/NAV part */
  rtcm3_encode_gal_eph_common(msg_eph, buff, &bit);

  rtcm_setbits(buff, bit, 10, msg_eph->data.kepler.tgd.gal_s[0]);
  bit += 10;
  rtcm_setbits(buff, bit, 3, msg_eph->health_bits);
  bit += 3;
  /* reserved */
  rtcm_setbits(buff, bit, 7, 0);
  bit += 7;

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}
