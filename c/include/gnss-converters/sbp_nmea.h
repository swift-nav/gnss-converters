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

#include <libsbp/gnss.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/sbp.h>
#include <libsbp/tracking.h>

/* Max number of sats visible in an epoch */
#define MAX_SATS 256
#define MAX_FUSED_WAGON_MSGS (SBP_MAX_PAYLOAD_LEN / sizeof(uint16_t))

typedef enum sbp2nmea_nmea_id {
  SBP2NMEA_NMEA_GGA = 0,
  SBP2NMEA_NMEA_RMC = 1,
  SBP2NMEA_NMEA_VTG = 2,
  SBP2NMEA_NMEA_HDT = 3,
  SBP2NMEA_NMEA_GLL = 4,
  SBP2NMEA_NMEA_ZDA = 5,
  SBP2NMEA_NMEA_GSA = 6,
  SBP2NMEA_NMEA_GST = 7,
  SBP2NMEA_NMEA_GSV = 8,
  SBP2NMEA_NMEA_PUBX = 9,
  SBP2NMEA_NMEA_CNT = 10,
} sbp2nmea_nmea_id_t;

typedef enum sbp2nmea_sbp_id {
  SBP2NMEA_SBP_GPS_TIME = 0,
  SBP2NMEA_SBP_UTC_TIME = 1,
  SBP2NMEA_SBP_POS_LLH_COV = 2,
  SBP2NMEA_SBP_VEL_NED = 3,
  SBP2NMEA_SBP_DOPS = 4,
  SBP2NMEA_SBP_AGE_CORR = 5,
  SBP2NMEA_SBP_HDG = 6,
  SBP2NMEA_SBP_SV_AZ_EL = 7,
  SBP2NMEA_SBP_MEASUREMENT_STATE = 8,
  SBP2NMEA_SBP_GROUP_META = 9,
  SBP2NMEA_SBP_SOLN_META = 10,
  SBP2NMEA_SBP_GPS_TIME_GNSS = 11,
  SBP2NMEA_SBP_UTC_TIME_GNSS = 12,
  SBP2NMEA_SBP_POS_LLH_COV_GNSS = 13,
  SBP2NMEA_SBP_VEL_NED_GNSS = 14,
  SBP2NMEA_SBP_CNT = 15,
} sbp2nmea_sbp_id_t;

typedef struct nmea_state_entry {
  uint32_t last_tow;
  int rate;
} nmea_state_entry_t;

typedef struct sbp2nmea_msg {
  uint8_t data[SBP_MAX_PAYLOAD_LEN];
  uint8_t length;
} sbp2nmea_msg_t;

typedef struct sbp_state_entry {
  sbp2nmea_msg_t msg;
} sbp_state_entry_t;

typedef enum sbp2nmea_mode {
  SBP2NMEA_MODE_BEST,
  SBP2NMEA_MODE_GNSS,
  SBP2NMEA_MODE_FUSED,
} sbp2nmea_mode_t;

typedef struct sbp2nmea_state {
  uint8_t num_obs;
  uint8_t obs_seq_count;
  uint8_t obs_seq_total;
  sbp_gnss_signal_t nav_sids[MAX_SATS];
  sbp_gps_time_t obs_time;

  sbp2nmea_mode_t requested_mode;
  sbp2nmea_mode_t actual_mode;
  bool fused_wagon_seen_msgs[MAX_FUSED_WAGON_MSGS];
  bool fused_wagon_complete;

  uint16_t base_sender_id;

  nmea_state_entry_t nmea_state[SBP2NMEA_NMEA_CNT];
  sbp_state_entry_t sbp_state[SBP2NMEA_SBP_CNT];

  float soln_freq;

  void (*cb_sbp_to_nmea)(char *msg, void *ctx);
  void *ctx;

  /* Only send the COG if the SOG exceeds this value */
  float cog_threshold_mps;

  /* Only recalculate the COG if the SOG exceeds this value
   * otherwise use the last_non_stationary_cog.
   * This allows filtering out variations in the COG due to small amounts of
   * noise in the SOG. */
  float cog_update_threshold_mps;
  double last_non_stationary_cog;
} sbp2nmea_t;

#ifdef __cplusplus
extern "C" {
#endif

void sbp2nmea_init(sbp2nmea_t *state,
                   sbp2nmea_mode_t mode,
                   void (*cb_sbp_to_nmea)(char *msg, void *ctx),
                   void *ctx);

void sbp2nmea(sbp2nmea_t *state,
              u8 len,
              const void *sbp_msg,
              sbp2nmea_sbp_id_t sbp_id);
void sbp2nmea_obs(sbp2nmea_t *state, const msg_obs_t *sbp_obs, uint8_t num_obs);

void sbp2nmea_base_id_set(sbp2nmea_t *state, uint16_t base_sender_id);
uint16_t sbp2nmea_base_id_get(const sbp2nmea_t *state);

uint8_t sbp2nmea_num_obs_get(const sbp2nmea_t *state);
const sbp_gnss_signal_t *sbp2nmea_nav_sids_get(const sbp2nmea_t *state);

void sbp2nmea_to_str(const sbp2nmea_t *state, char *sentence);

const void *sbp2nmea_msg_get(const sbp2nmea_t *state,
                             sbp2nmea_sbp_id_t id,
                             bool consider_mode);

uint8_t sbp2nmea_msg_length(const sbp2nmea_t *state, sbp2nmea_sbp_id_t id);

void sbp2nmea_msg_set(sbp2nmea_t *state,
                      u8 len,
                      const void *sbp_msg,
                      sbp2nmea_sbp_id_t id);

void sbp2nmea_rate_set(sbp2nmea_t *state, int rate, sbp2nmea_nmea_id_t id);

void sbp2nmea_soln_freq_set(sbp2nmea_t *state, float soln_freq);

void sbp2nmea_cog_threshold_set(sbp2nmea_t *state, float cog_thd_mps);

void sbp2nmea_cog_stationary_threshold_set(sbp2nmea_t *state,
                                           double cog_stationary_thd_mps);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_SBP_NMEA_INTERFACE_H */
