/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <math.h>

#include "sbp_rtcm3_internal.h"
#include "rtcm3_utils.h"
#include "gnss-converters/sbp_rtcm3.h"

void sbp_to_rtcm3_gps_eph(const msg_ephemeris_gps_t *sbp_gps_eph,
                       rtcm_msg_eph *msg_eph,
                       const struct rtcm3_out_state *state) {
  (void)state;
  assert(sbp_gps_eph);
  assert(msg_eph);
  assert(is_gps(sbp_gps_eph->common.sid.code));

  msg_eph->wn = sbp_gps_eph->common.toe.wn % 1024;
  msg_eph->toe = sbp_gps_eph->common.toe.tow / GPS_TOE_RESOLUTION;
  msg_eph->sat_id = sbp_gps_eph->common.sid.sat;
  msg_eph->ura = convert_uri_to_ura(sbp_gps_eph->common.ura);
  msg_eph->fit_interval = rtcm3_encode_fit_interval_gps(sbp_gps_eph->common.fit_interval);

  // SBP doesn't include this information so it is hardcoded
  msg_eph->kepler.codeL2 = 3; /* L2C enabled */
  msg_eph->kepler.L2_data_bit = true; /* L2P Nav data off */


  msg_eph->health_bits = sbp_gps_eph->common.health_bits;
  msg_eph->kepler.tgd_gps_s = (int32_t)(round(sbp_gps_eph->tgd / C_1_2P31));

  msg_eph->kepler.crs = (int32_t)(round(sbp_gps_eph->c_rs / C_1_2P5));
  msg_eph->kepler.crc = (int32_t)(round(sbp_gps_eph->c_rc / C_1_2P5));
  msg_eph->kepler.cuc = (int32_t)(round(sbp_gps_eph->c_uc / C_1_2P29));
  msg_eph->kepler.cus = (int32_t)(round(sbp_gps_eph->c_us / C_1_2P29));
  msg_eph->kepler.cic = (int32_t)(round(sbp_gps_eph->c_ic / C_1_2P29));
  msg_eph->kepler.cis = (int32_t)(round(sbp_gps_eph->c_is / C_1_2P29));

  msg_eph->kepler.dn = (int16_t)(round(sbp_gps_eph->dn / (C_1_2P43 * M_PI)));
  msg_eph->kepler.m0 = (int32_t)(round(sbp_gps_eph->m0 / (C_1_2P31 * M_PI)));
  msg_eph->kepler.ecc = (uint32_t)(round(sbp_gps_eph->ecc / C_1_2P33));
  msg_eph->kepler.sqrta = (uint32_t)(round(sbp_gps_eph->sqrta / C_1_2P19));
  msg_eph->kepler.omega0 = (int32_t)(round(sbp_gps_eph->omega0 / (C_1_2P31 * M_PI)));
  msg_eph->kepler.omegadot = (int32_t)(round(sbp_gps_eph->omegadot / (C_1_2P43 * M_PI)));
  msg_eph->kepler.w = (int32_t)(round(sbp_gps_eph->w / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc = (int32_t)(round(sbp_gps_eph->inc / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc_dot = (int16_t)(round(sbp_gps_eph->inc_dot / (C_1_2P43 * M_PI)));

  msg_eph->kepler.af0 = (int32_t)(round(sbp_gps_eph->af0 / C_1_2P31));
  msg_eph->kepler.af1 = (int32_t)(round(sbp_gps_eph->af1 / C_1_2P43));
  msg_eph->kepler.af2 = (int32_t)(round(sbp_gps_eph->af2 / C_1_2P55));

  msg_eph->kepler.iode = sbp_gps_eph->iode;
  msg_eph->kepler.iodc = sbp_gps_eph->iodc;

  msg_eph->wn = sbp_gps_eph->toc.wn % 1024;
  msg_eph->kepler.toc = sbp_gps_eph->toc.tow / GPS_TOC_RESOLUTION;
  
}

void sbp_to_rtcm3_gal_eph(const msg_ephemeris_gal_t *sbp_gal_eph,
                            rtcm_msg_eph *msg_eph,
                            const struct rtcm3_out_state *state) {
  (void)state;
  assert(msg_eph);
  assert(sbp_gal_eph);
  assert(is_gal(sbp_gal_eph->common.sid.code));

  /* Galileo week is 12 bit (DF289) and starts from GPS WN 1024 */
  msg_eph->wn = (sbp_gal_eph->common.toe.wn - GAL_WEEK_TO_GPS_WEEK);
  msg_eph->toe = sbp_gal_eph->common.toe.tow / GALILEO_TOE_RESOLUTION;

  msg_eph->sat_id = sbp_gal_eph->common.sid.sat;
  msg_eph->ura = convert_meters_to_sisa(sbp_gal_eph->common.ura);

  msg_eph->health_bits = sbp_gal_eph->common.health_bits;

  msg_eph->kepler.tgd_gal_s[0] = (int32_t)(round(sbp_gal_eph->bgd_e1e5a / C_1_2P32));
  msg_eph->kepler.tgd_gal_s[1] = (int32_t)(round(sbp_gal_eph->bgd_e1e5b / C_1_2P32));

  msg_eph->kepler.crs = (int32_t)(round(sbp_gal_eph->c_rs / C_1_2P5));
  msg_eph->kepler.crc = (int32_t)(round(sbp_gal_eph->c_rc / C_1_2P5));
  msg_eph->kepler.cuc = (int32_t)(round(sbp_gal_eph->c_uc / C_1_2P29));
  msg_eph->kepler.cus = (int32_t)(round(sbp_gal_eph->c_us / C_1_2P29));
  msg_eph->kepler.cic = (int32_t)(round(sbp_gal_eph->c_ic / C_1_2P29));
  msg_eph->kepler.cis = (int32_t)(round(sbp_gal_eph->c_is / C_1_2P29));

  msg_eph->kepler.dn = (int16_t)(round(sbp_gal_eph->dn / (C_1_2P43 * M_PI)));
  msg_eph->kepler.m0 = (int32_t)(round(sbp_gal_eph->m0 / (C_1_2P31 * M_PI)));
  msg_eph->kepler.ecc = (uint32_t)(round(sbp_gal_eph->ecc / C_1_2P33));
  msg_eph->kepler.sqrta = (uint32_t)(round(sbp_gal_eph->sqrta / C_1_2P19));
  msg_eph->kepler.omega0 = (int32_t)(round(sbp_gal_eph->omega0 / (C_1_2P31 * M_PI)));
  msg_eph->kepler.omegadot = (int32_t)(round(sbp_gal_eph->omegadot / (C_1_2P43 * M_PI)));
  msg_eph->kepler.w = (int32_t)(round(sbp_gal_eph->w / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc = (int32_t)(round(sbp_gal_eph->inc / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc_dot = (int16_t)(round(sbp_gal_eph->inc_dot / (C_1_2P43 * M_PI)));

  msg_eph->kepler.af0 = (int32_t)(round(sbp_gal_eph->af0 / C_1_2P34));
  msg_eph->kepler.af1 = (int32_t)(round(sbp_gal_eph->af1 / C_1_2P46));
  msg_eph->kepler.af2 = (int16_t)(round(sbp_gal_eph->af2 / C_1_2P59));

  msg_eph->kepler.iode = sbp_gal_eph->iode;
  msg_eph->kepler.iode = sbp_gal_eph->iodc;

  msg_eph->kepler.toc = sbp_gal_eph->toc.tow / GALILEO_TOC_RESOLUTION;
}

