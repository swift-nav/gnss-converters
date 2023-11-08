/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtcm3/eph_decode.h"
#include <assert.h>
#include <string.h>
#include "rtcm3/bits.h"

#include "decode_helpers.h"

/** Decode an RTCMv3 GPS Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_gps_eph_bitstream(swiftnav_in_bitstream_t *buff,
                                        rtcm_msg_eph *msg_eph) {
  assert(msg_eph);
  memset(msg_eph, 0, sizeof(*msg_eph));
  msg_eph->constellation = RTCM_CONSTELLATION_GPS;
  uint16_t msg_num;
  BITSTREAM_DECODE_U16(buff, msg_num, 12);
  if (msg_num != 1019) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }
  BITSTREAM_DECODE_U8(buff, msg_eph->sat_id, 6);
  BITSTREAM_DECODE_U16(buff, msg_eph->wn, 10);
  BITSTREAM_DECODE_U16(buff, msg_eph->ura, 4);
  BITSTREAM_DECODE_U8(buff, msg_eph->data.kepler.codeL2, 2);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.inc_dot, 14);
  BITSTREAM_DECODE_U16(buff, msg_eph->data.kepler.iode, 8);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.toc, 16);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.af2, 8);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.af1, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.af0, 22);
  BITSTREAM_DECODE_U16(buff, msg_eph->data.kepler.iodc, 10);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.crs, 16);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.dn, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.m0, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cuc, 16);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.ecc, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cus, 16);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.sqrta, 32);
  BITSTREAM_DECODE_U32(buff, msg_eph->toe, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cic, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.omega0, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cis, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.inc, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.crc, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.w, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.omegadot, 24);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.tgd.gps_s, 8);
  BITSTREAM_DECODE_U8(buff, msg_eph->health_bits, 6);
  uint8_t L2_data_bit;
  BITSTREAM_DECODE_U8(buff, L2_data_bit, 1);
  msg_eph->data.kepler.L2_data_bit = (bool)L2_data_bit;
  BITSTREAM_DECODE_U32(buff, msg_eph->fit_interval, 1);

  return RC_OK;
}

/** Decode an RTCMv3 QZSS Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_qzss_eph_bitstream(swiftnav_in_bitstream_t *buff,
                                         rtcm_msg_eph *msg_eph) {
  assert(msg_eph);
  memset(msg_eph, 0, sizeof(*msg_eph));
  msg_eph->constellation = RTCM_CONSTELLATION_QZS;
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);
  if (msg_num != 1044) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }
  BITSTREAM_DECODE_U8(buff, msg_eph->sat_id, 4);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.toc, 16);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.af2, 8);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.af1, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.af0, 22);
  BITSTREAM_DECODE_U16(buff, msg_eph->data.kepler.iode, 8);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.crs, 16);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.dn, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.m0, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cuc, 16);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.ecc, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cus, 16);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.sqrta, 32);
  BITSTREAM_DECODE_U32(buff, msg_eph->toe, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cic, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.omega0, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cis, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.inc, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.crc, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.w, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.omegadot, 24);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.inc_dot, 14);
  /* L2 data bit */
  swiftnav_in_bitstream_remove(buff, 2);
  BITSTREAM_DECODE_U16(buff, msg_eph->wn, 10);
  BITSTREAM_DECODE_U16(buff, msg_eph->ura, 4);
  BITSTREAM_DECODE_U8(buff, msg_eph->health_bits, 6);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.tgd.gps_s, 8);
  BITSTREAM_DECODE_U16(buff, msg_eph->data.kepler.iodc, 10);
  BITSTREAM_DECODE_U32(buff, msg_eph->fit_interval, 1);
  return RC_OK;
}

/** Decode an RTCMv3 GLO Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_glo_eph_bitstream(swiftnav_in_bitstream_t *buff,
                                        rtcm_msg_eph *msg_eph) {
  assert(msg_eph);
  memset(msg_eph, 0, sizeof(*msg_eph));
  msg_eph->constellation = RTCM_CONSTELLATION_GLO;
  uint16_t msg_num;
  BITSTREAM_DECODE_U16(buff, msg_num, 12);
  if (msg_num != 1020) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }
  BITSTREAM_DECODE_U8(buff, msg_eph->sat_id, 6);
  BITSTREAM_DECODE_U8(buff, msg_eph->data.glo.fcn, 5);
  /*alm health ind = */
  swiftnav_in_bitstream_remove(buff, 1);
  /*alm health ind valid = */
  swiftnav_in_bitstream_remove(buff, 1);
  BITSTREAM_DECODE_U32(buff, msg_eph->fit_interval, 2);
  /* tk */
  swiftnav_in_bitstream_remove(buff, 12);
  /* most significant bit of Bn */
  uint8_t bn_msb;
  BITSTREAM_DECODE_U8(buff, bn_msb, 1);
  /* P2 */
  swiftnav_in_bitstream_remove(buff, 1);
  BITSTREAM_DECODE_U8(buff, msg_eph->data.glo.t_b, 7);
  rtcm3_rc ret =
      rtcm_get_sign_magnitude_bitstream(buff, 24, &msg_eph->data.glo.vel[0]);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_get_sign_magnitude_bitstream(buff, 27, &msg_eph->data.glo.pos[0]);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_get_sign_magnitude_bitstream(buff, 5, &msg_eph->data.glo.acc[0]);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_get_sign_magnitude_bitstream(buff, 24, &msg_eph->data.glo.vel[1]);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_get_sign_magnitude_bitstream(buff, 27, &msg_eph->data.glo.pos[1]);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_get_sign_magnitude_bitstream(buff, 5, &msg_eph->data.glo.acc[1]);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_get_sign_magnitude_bitstream(buff, 24, &msg_eph->data.glo.vel[2]);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_get_sign_magnitude_bitstream(buff, 27, &msg_eph->data.glo.pos[2]);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_get_sign_magnitude_bitstream(buff, 5, &msg_eph->data.glo.acc[2]);
  if (ret != RC_OK) {
    return ret;
  }
  /* P3 */
  swiftnav_in_bitstream_remove(buff, 1);
  s32 gamma;
  ret = rtcm_get_sign_magnitude_bitstream(buff, 11, &gamma);
  if (ret != RC_OK) {
    return ret;
  }
  msg_eph->data.glo.gamma = (s16)gamma;
  /* P */
  swiftnav_in_bitstream_remove(buff, 2);
  /* health flag in string 3 */
  uint8_t mln3;
  BITSTREAM_DECODE_U8(buff, mln3, 1);
  ret = rtcm_get_sign_magnitude_bitstream(buff, 22, &msg_eph->data.glo.tau);
  if (ret != RC_OK) {
    return ret;
  }
  s32 d_tau;
  ret = rtcm_get_sign_magnitude_bitstream(buff, 5, &d_tau);
  if (ret != RC_OK) {
    return ret;
  }
  msg_eph->data.glo.d_tau = (uint8_t)d_tau;
  /* EN */
  swiftnav_in_bitstream_remove(buff, 5);
  /* P4 */
  swiftnav_in_bitstream_remove(buff, 1);
  BITSTREAM_DECODE_U16(buff, msg_eph->ura, 4);
  /* NT */
  swiftnav_in_bitstream_remove(buff, 11);
  /* M */
  swiftnav_in_bitstream_remove(buff, 2);
  uint32_t additional_data;
  BITSTREAM_DECODE_U32(buff, additional_data, 1);
  uint8_t mln5 = 0;
  if (additional_data) {
    /* NA  */
    swiftnav_in_bitstream_remove(buff, 11);
    /* Tc */
    swiftnav_in_bitstream_remove(buff, 32);
    /* N4 */
    swiftnav_in_bitstream_remove(buff, 5);
    /* Tgps */
    swiftnav_in_bitstream_remove(buff, 22);
    /* health flag in string 5 */
    BITSTREAM_DECODE_U8(buff, mln5, 1);
    /* reserved */
    swiftnav_in_bitstream_remove(buff, 7);
  }
  msg_eph->health_bits = bn_msb | mln3 | mln5;
  return RC_OK;
}

/** Decode an RTCMv3 BDS Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Satellite is geostationary
 */
rtcm3_rc rtcm3_decode_bds_eph_bitstream(swiftnav_in_bitstream_t *buff,
                                        rtcm_msg_eph *msg_eph) {
  assert(msg_eph);
  memset(msg_eph, 0, sizeof(*msg_eph));
  msg_eph->constellation = RTCM_CONSTELLATION_BDS;
  uint16_t msg_num;
  BITSTREAM_DECODE_U16(buff, msg_num, 12);
  if (msg_num != 1042) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }
  BITSTREAM_DECODE_U8(buff, msg_eph->sat_id, 6);
  if (msg_eph->sat_id <= BEIDOU_GEOS_MAX_PRN) {
    /* We do not support Beidou GEO satellites */
    return RC_INVALID_MESSAGE;
  }
  BITSTREAM_DECODE_U16(buff, msg_eph->wn, 13);
  BITSTREAM_DECODE_U16(buff, msg_eph->ura, 4);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.inc_dot, 14);
  BITSTREAM_DECODE_U16(buff, msg_eph->data.kepler.iode, 5);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.toc, 17);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.af2, 11);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.af1, 22);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.af0, 24);
  BITSTREAM_DECODE_U16(buff, msg_eph->data.kepler.iodc, 5);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.crs, 18);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.dn, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.m0, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cuc, 18);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.ecc, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cus, 18);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.sqrta, 32);
  BITSTREAM_DECODE_U32(buff, msg_eph->toe, 17);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cic, 18);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.omega0, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cis, 18);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.inc, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.crc, 18);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.w, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.omegadot, 24);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.tgd.bds_s[0], 10);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.tgd.bds_s[1], 10);
  BITSTREAM_DECODE_U8(buff, msg_eph->health_bits, 1);
  return RC_OK;
}

/** Decode an RTCMv3 GAL (common part) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param msg_eph RTCM message struct
 * \return bit position in the RTCM frame
 */
static uint16_t rtcm3_decode_gal_eph_common(swiftnav_in_bitstream_t *buff,
                                            rtcm_msg_eph *msg_eph) {
  assert(msg_eph);
  BITSTREAM_DECODE_U8(buff, msg_eph->sat_id, 6);
  BITSTREAM_DECODE_U16(buff, msg_eph->wn, 12);
  BITSTREAM_DECODE_U16(buff, msg_eph->data.kepler.iode, 10);
  BITSTREAM_DECODE_U16(buff, msg_eph->ura, 8);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.inc_dot, 14);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.toc, 14);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.af2, 6);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.af1, 21);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.af0, 31);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.crs, 16);
  BITSTREAM_DECODE_S16(buff, msg_eph->data.kepler.dn, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.m0, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cuc, 16);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.ecc, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cus, 16);
  BITSTREAM_DECODE_U32(buff, msg_eph->data.kepler.sqrta, 32);
  BITSTREAM_DECODE_U32(buff, msg_eph->toe, 14);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cic, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.omega0, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.cis, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.inc, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.crc, 16);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.w, 32);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.omegadot, 24);
  return RC_OK;
}

/** Decode an RTCMv3 GAL (I/NAV message) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_gal_eph_inav_bitstream(swiftnav_in_bitstream_t *buff,
                                             rtcm_msg_eph *msg_eph) {
  assert(msg_eph);
  memset(msg_eph, 0, sizeof(*msg_eph));
  msg_eph->constellation = RTCM_CONSTELLATION_GAL;
  uint16_t msg_num;
  BITSTREAM_DECODE_U16(buff, msg_num, 12);
  if (msg_num != 1046) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  /* parse common I/NAV and F/NAV part */
  rtcm3_rc ret = rtcm3_decode_gal_eph_common(buff, msg_eph);
  if (ret != RC_OK) {
    return ret;
  }

  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.tgd.gal_s[0], 10);
  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.tgd.gal_s[1], 10);
  BITSTREAM_DECODE_S8(buff, msg_eph->health_bits, 6);
  /* reserved */
  swiftnav_in_bitstream_remove(buff, 2);

  return RC_OK;
}

/** Decode an RTCMv3 GAL (F/NAV message) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_gal_eph_fnav_bitstream(swiftnav_in_bitstream_t *buff,
                                             rtcm_msg_eph *msg_eph) {
  assert(msg_eph);
  memset(msg_eph, 0, sizeof(*msg_eph));
  msg_eph->constellation = RTCM_CONSTELLATION_GAL;
  uint16_t msg_num;
  BITSTREAM_DECODE_U16(buff, msg_num, 12);
  if (msg_num != 1045) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  /* parse common F/NAV and I/NAV part */
  rtcm3_rc ret = rtcm3_decode_gal_eph_common(buff, msg_eph);
  if (ret != RC_OK) {
    return ret;
  }

  BITSTREAM_DECODE_S32(buff, msg_eph->data.kepler.tgd.gal_s[0], 10);
  msg_eph->data.kepler.tgd.gal_s[1] = 0;
  BITSTREAM_DECODE_S8(buff, msg_eph->health_bits, 3);
  /* reserved */
  swiftnav_in_bitstream_remove(buff, 7);

  return RC_OK;
}
