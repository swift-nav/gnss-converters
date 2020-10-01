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

#include "common.h"
#include "gnss-converters/sbp_rtcm3.h"
#include "rtcm3_utils.h"
#include "sbp_rtcm3_internal.h"

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
  msg_eph->constellation = RTCM_CONSTELLATION_GPS;
  msg_eph->ura = convert_gps_uri_to_ura(sbp_gps_eph->common.ura);
  msg_eph->fit_interval =
      rtcm3_encode_fit_interval_gps(sbp_gps_eph->common.fit_interval);

  // SBP doesn't include this information so it is hardcoded
  msg_eph->kepler.codeL2 = 1;          /* P-code enabled */
  msg_eph->kepler.L2_data_bit = false; /* L2P Nav data on */

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
  msg_eph->kepler.omega0 =
      (int32_t)(round(sbp_gps_eph->omega0 / (C_1_2P31 * M_PI)));
  msg_eph->kepler.omegadot =
      (int32_t)(round(sbp_gps_eph->omegadot / (C_1_2P43 * M_PI)));
  msg_eph->kepler.w = (int32_t)(round(sbp_gps_eph->w / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc = (int32_t)(round(sbp_gps_eph->inc / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc_dot =
      (int16_t)(round(sbp_gps_eph->inc_dot / (C_1_2P43 * M_PI)));

  msg_eph->kepler.af0 = (int32_t)(round(sbp_gps_eph->af0 / C_1_2P31));
  msg_eph->kepler.af1 = (int32_t)(round(sbp_gps_eph->af1 / C_1_2P43));
  msg_eph->kepler.af2 = (int32_t)(round(sbp_gps_eph->af2 / C_1_2P55));

  msg_eph->kepler.iode = sbp_gps_eph->iode;
  msg_eph->kepler.iodc = sbp_gps_eph->iodc;

  msg_eph->wn = sbp_gps_eph->toc.wn % 1024;
  msg_eph->kepler.toc = sbp_gps_eph->toc.tow / GPS_TOC_RESOLUTION;
}

static bool compute_glo_time_of_day(const gps_time_t *obs_time,
                                    const struct rtcm3_out_state *state,
                                    u8 *t_b) {
  if (!state->leap_second_known) {
    return false;
  }
  assert(gps_time_valid(obs_time));
  s32 tod_glo_s = (s32)(obs_time->tow - state->leap_seconds) % DAY_SECS;
  tod_glo_s += UTC_SU_OFFSET * HOUR_SECS;

  if (tod_glo_s > DAY_SECS) {
    tod_glo_s -= DAY_SECS;
  }

  *t_b = tod_glo_s / (15 * MINUTE_SECS);
  return true;
}

void sbp_to_rtcm3_glo_eph(const msg_ephemeris_glo_t *sbp_glo_eph,
                          rtcm_msg_eph *msg_eph,
                          const struct rtcm3_out_state *state) {
  (void)state;
  assert(sbp_glo_eph);
  assert(msg_eph);
  assert(is_glo(sbp_glo_eph->common.sid.code));

  msg_eph->sat_id = sbp_glo_eph->common.sid.sat;
  msg_eph->constellation = RTCM_CONSTELLATION_GLO;
  msg_eph->wn = sbp_glo_eph->common.toe.wn;
  msg_eph->toe = sbp_glo_eph->common.toe.tow;
  msg_eph->ura = convert_glo_uri_to_ura(sbp_glo_eph->common.ura);

  msg_eph->fit_interval =
      rtcm3_encode_fit_interval_glo(sbp_glo_eph->common.fit_interval);
  msg_eph->health_bits = sbp_glo_eph->common.health_bits;
  gps_time_t time = sbp_gps_time_2_gps_time(&sbp_glo_eph->common.toe);
  if (!compute_glo_time_of_day(&time, state, &msg_eph->glo.t_b)) {
    msg_eph->health_bits = 1;
  }

  msg_eph->glo.gamma = (int16_t)(round(sbp_glo_eph->gamma / C_1_2P40));
  msg_eph->glo.tau = (int32_t)(round(sbp_glo_eph->tau / C_1_2P30));
  msg_eph->glo.d_tau = (int8_t)(round(sbp_glo_eph->d_tau / C_1_2P30));

  msg_eph->glo.pos[0] = (int32_t)(round(sbp_glo_eph->pos[0] / C_1_2P11 / 1000));
  msg_eph->glo.pos[1] = (int32_t)(round(sbp_glo_eph->pos[1] / C_1_2P11 / 1000));
  msg_eph->glo.pos[2] = (int32_t)(round(sbp_glo_eph->pos[2] / C_1_2P11 / 1000));

  msg_eph->glo.vel[0] = (int32_t)(round(sbp_glo_eph->vel[0] / C_1_2P20 / 1000));
  msg_eph->glo.vel[1] = (int32_t)(round(sbp_glo_eph->vel[1] / C_1_2P20 / 1000));
  msg_eph->glo.vel[2] = (int32_t)(round(sbp_glo_eph->vel[2] / C_1_2P20 / 1000));

  msg_eph->glo.acc[0] = (int32_t)(round(sbp_glo_eph->acc[0] / C_1_2P30 / 1000));
  msg_eph->glo.acc[1] = (int32_t)(round(sbp_glo_eph->acc[1] / C_1_2P30 / 1000));
  msg_eph->glo.acc[2] = (int32_t)(round(sbp_glo_eph->acc[2] / C_1_2P30 / 1000));

  msg_eph->glo.fcn = sbp_glo_eph->fcn - 1;
  msg_eph->glo.iod = sbp_glo_eph->iod;
}

static void get_bds_wn_tow(const gps_time_t *input,
                           const u32 time_resolution,
                           uint16_t *wn,
                           uint32_t *tow) {
  if (input->tow > BDS_SECOND_TO_GPS_SECOND) {
    *wn = (input->wn - BDS_WEEK_TO_GPS_WEEK);
    *tow = (uint32_t)round((input->tow - BDS_SECOND_TO_GPS_SECOND) /
                           time_resolution);
  } else {
    *wn = (input->wn - BDS_WEEK_TO_GPS_WEEK - 1);
    *tow = (uint32_t)round((input->tow - BDS_SECOND_TO_GPS_SECOND + WEEK_SECS) /
                           time_resolution);
  }
  // Deal with rollover every 8192 weeks
  *wn = *wn % 8192;
}

void sbp_to_rtcm3_bds_eph(const msg_ephemeris_bds_t *sbp_bds_eph,
                          rtcm_msg_eph *msg_eph,
                          const struct rtcm3_out_state *state) {
  (void)state;
  assert(msg_eph);
  assert(sbp_bds_eph);
  assert(is_bds2(sbp_bds_eph->common.sid.code));

  msg_eph->sat_id = sbp_bds_eph->common.sid.sat;
  msg_eph->constellation = RTCM_CONSTELLATION_BDS;
  gps_time_t input_time;
  input_time.wn = sbp_bds_eph->common.toe.wn;
  input_time.tow = sbp_bds_eph->common.toe.tow;
  get_bds_wn_tow(
      &input_time, BEIDOU_TOE_RESOLUTION, &msg_eph->wn, &msg_eph->toe);

  msg_eph->ura = convert_bds_uri_to_ura(sbp_bds_eph->common.ura);

  // Not present in RTCM format
  msg_eph->fit_interval = 0;

  msg_eph->health_bits = sbp_bds_eph->common.health_bits;

  msg_eph->kepler.tgd_bds_s[0] = (int32_t)(round(sbp_bds_eph->tgd1 / 1e-10));
  msg_eph->kepler.tgd_bds_s[1] = (int32_t)(round(sbp_bds_eph->tgd2 / 1e-10));

  msg_eph->kepler.crs = (int32_t)(round(sbp_bds_eph->c_rs / C_1_2P6));
  msg_eph->kepler.crc = (int32_t)(round(sbp_bds_eph->c_rc / C_1_2P6));
  msg_eph->kepler.cuc = (int32_t)(round(sbp_bds_eph->c_uc / C_1_2P31));
  msg_eph->kepler.cus = (int32_t)(round(sbp_bds_eph->c_us / C_1_2P31));
  msg_eph->kepler.cic = (int32_t)(round(sbp_bds_eph->c_ic / C_1_2P31));
  msg_eph->kepler.cis = (int32_t)(round(sbp_bds_eph->c_is / C_1_2P31));

  msg_eph->kepler.dn = (int16_t)(round(sbp_bds_eph->dn / (C_1_2P43 * M_PI)));
  msg_eph->kepler.m0 = (int32_t)(round(sbp_bds_eph->m0 / (C_1_2P31 * M_PI)));
  msg_eph->kepler.ecc = (uint32_t)(round(sbp_bds_eph->ecc / C_1_2P33));
  msg_eph->kepler.sqrta = (uint32_t)(round(sbp_bds_eph->sqrta / C_1_2P19));
  msg_eph->kepler.omega0 =
      (int32_t)(round(sbp_bds_eph->omega0 / (C_1_2P31 * M_PI)));
  msg_eph->kepler.omegadot =
      (int32_t)(round(sbp_bds_eph->omegadot / (C_1_2P43 * M_PI)));
  msg_eph->kepler.w = (int32_t)(round(sbp_bds_eph->w / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc = (int32_t)(round(sbp_bds_eph->inc / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc_dot =
      (int16_t)(round(sbp_bds_eph->inc_dot / (C_1_2P43 * M_PI)));

  msg_eph->kepler.af0 = (int32_t)(round(sbp_bds_eph->af0 / C_1_2P33));
  msg_eph->kepler.af1 = (int32_t)(round(sbp_bds_eph->af1 / C_1_2P50));
  msg_eph->kepler.af2 = (int16_t)(round(sbp_bds_eph->af2 / C_1_2P66));

  msg_eph->kepler.iode = sbp_bds_eph->iode;
  msg_eph->kepler.iodc = sbp_bds_eph->iodc;

  // What happens if toc and toe straddle the week boundary?
  input_time.wn = sbp_bds_eph->toc.wn;
  input_time.tow = sbp_bds_eph->toc.tow;
  get_bds_wn_tow(
      &input_time, BEIDOU_TOC_RESOLUTION, &msg_eph->wn, &msg_eph->kepler.toc);
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
  msg_eph->constellation = RTCM_CONSTELLATION_GAL;
  msg_eph->ura = convert_meters_to_sisa(sbp_gal_eph->common.ura);

  msg_eph->health_bits = sbp_gal_eph->common.health_bits;

  msg_eph->kepler.tgd_gal_s[0] =
      (int32_t)(round(sbp_gal_eph->bgd_e1e5a / C_1_2P32));
  msg_eph->kepler.tgd_gal_s[1] =
      (int32_t)(round(sbp_gal_eph->bgd_e1e5b / C_1_2P32));

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
  msg_eph->kepler.omega0 =
      (int32_t)(round(sbp_gal_eph->omega0 / (C_1_2P31 * M_PI)));
  msg_eph->kepler.omegadot =
      (int32_t)(round(sbp_gal_eph->omegadot / (C_1_2P43 * M_PI)));
  msg_eph->kepler.w = (int32_t)(round(sbp_gal_eph->w / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc = (int32_t)(round(sbp_gal_eph->inc / (C_1_2P31 * M_PI)));
  msg_eph->kepler.inc_dot =
      (int16_t)(round(sbp_gal_eph->inc_dot / (C_1_2P43 * M_PI)));

  msg_eph->kepler.af0 = (int32_t)(round(sbp_gal_eph->af0 / C_1_2P34));
  msg_eph->kepler.af1 = (int32_t)(round(sbp_gal_eph->af1 / C_1_2P46));
  msg_eph->kepler.af2 = (int16_t)(round(sbp_gal_eph->af2 / C_1_2P59));

  msg_eph->kepler.iode = sbp_gal_eph->iode;
  msg_eph->kepler.iode = sbp_gal_eph->iodc;

  msg_eph->kepler.toc = sbp_gal_eph->toc.tow / GALILEO_TOC_RESOLUTION;
}
