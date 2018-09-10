/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_SBP_NMEA_INTERFACE_H
#define GNSS_CONVERTERS_SBP_NMEA_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <libsbp/gnss.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>

/* Max number of sats visible in an epoch */
#define MAX_SATS 256

struct sbp_nmea_state {
  msg_gps_time_t sbp_gps_time;
  msg_utc_time_t sbp_utc_time;
  msg_pos_llh_t sbp_pos_llh;
  msg_vel_ned_t sbp_vel_ned;
  msg_dops_t sbp_dops;
  msg_age_corrections_t sbp_age_corr;
  msg_baseline_heading_t sbp_heading;

  uint8_t num_obs;
  uint8_t count;
  uint8_t total;
  sbp_gnss_signal_t nav_sids[MAX_SATS];
  sbp_gps_time_t obs_time;

  uint16_t base_sender_id;

  uint32_t gpgga_last_tow;
  uint32_t gpgsv_last_tow;
  uint32_t gprmc_last_tow;
  uint32_t gpvtg_last_tow;
  uint32_t gphdt_last_tow;
  uint32_t gpgll_last_tow;
  uint32_t gpzda_last_tow;
  uint32_t gsa_last_tow;

  int gpgga_rate;
  int gpgsv_rate;
  int gprmc_rate;
  int gpvtg_rate;
  int gphdt_rate;
  int gpgll_rate;
  int gpzda_rate;
  int gsa_rate;

  float soln_freq;

  void (*cb_sbp_to_nmea)();
};

void sbp2nmea_gps_time(const msg_gps_time_t *sbp_gps_time,
                       struct sbp_nmea_state *state);

void sbp2nmea_utc_time(const msg_utc_time_t *sbp_utc_time,
                       struct sbp_nmea_state *state);

void sbp2nmea_pos_llh(const msg_pos_llh_t *sbp_pos_llh,
                      struct sbp_nmea_state *state);

void sbp2nmea_vel_ned(const msg_vel_ned_t *sbp_vel_ned,
                      struct sbp_nmea_state *state);

void sbp2nmea_dops(const msg_dops_t *sbp_dops, struct sbp_nmea_state *state);

void sbp2nmea_age_corrections(const msg_age_corrections_t *sbp_age_corr,
                              struct sbp_nmea_state *state);

void sbp2nmea_baseline_heading(const msg_baseline_heading_t *sbp_heading,
                               struct sbp_nmea_state *state);

void sbp2nmea_set_base_id(const uint16_t base_sender_id,
                          struct sbp_nmea_state *state);

void sbp2nmea_obs(const msg_obs_t *sbp_obs,
                  uint8_t num_obs,
                  struct sbp_nmea_state *state);

void sbp2nmea_set_gpgga_rate(const int gpgga_rate, struct sbp_nmea_state *state);
void sbp2nmea_set_gpgsv_rate(const int gpgsv_rate, struct sbp_nmea_state *state);
void sbp2nmea_set_gprmc_rate(const int gprmc_rate, struct sbp_nmea_state *state);
void sbp2nmea_set_gpvtg_rate(const int gpvtg_rate, struct sbp_nmea_state *state);
void sbp2nmea_set_gphdt_rate(const int gphdt_rate, struct sbp_nmea_state *state);
void sbp2nmea_set_gpgll_rate(const int gpgll_rate, struct sbp_nmea_state *state);
void sbp2nmea_set_gpzda_rate(const int gpzda_rate, struct sbp_nmea_state *state);
void sbp2nmea_set_gsa_rate(const int gsa_rate, struct sbp_nmea_state *state);

void sbp2nmea_set_soln_freq(const float soln_freq, struct sbp_nmea_state *state);

void sbp2nmea_init(struct sbp_nmea_state *state,
                   void (*cb_sbp_to_nmea)(u8 msg_id[]));

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_SBP_NMEA_INTERFACE_H */
