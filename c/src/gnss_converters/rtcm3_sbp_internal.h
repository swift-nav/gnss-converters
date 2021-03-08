/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_RTCM3_SBP_INTERNAL_H
#define GNSS_CONVERTERS_RTCM3_SBP_INTERNAL_H

#include <rtcm3/messages.h>
#include <rtcm3/msm_utils.h>
#include <swiftnav/bitstream.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>
#include "common.h"
#include "gnss-converters/rtcm3_sbp.h"

/* message type range reserved for MSM */
#define MSM_MSG_TYPE_MIN 1070
#define MSM_MSG_TYPE_MAX 1229
/* bit offset of the multiple message flag, regardless of MSM type */
#define MSM_MULTIPLE_BIT_OFFSET 54

#define RTCM_1029_LOGGING_LEVEL (6u)        /* This represents LOG_INFO */
#define RTCM_MSM_LOGGING_LEVEL (4u)         /* This represents LOG_WARN */
#define RTCM_BUFFER_FULL_LOGGING_LEVEL (3u) /* This represents LOG_ERROR */
#define RTCM_CODE_LOGGING_LEVEL (4u)        /* This represents LOG_WARN */

#define CODE_WARNING_BUFFER_SIZE (100u)
#define CODE_WARNING_FMT_STRING "Unsupported code received from base: %s"

/** Threshold for base measurements coming from the future */
#define BASE_FUTURE_THRESHOLD_S 10
/** Threshold for GLO time difference from rover */
#define GLO_SANITY_THRESHOLD_S 30

/** Constant difference of Galileo time from GPS time */
#define GAL_WEEK_TO_GPS_WEEK 1024

/* PREAMBLE to append to an RTCM3 log message */
#define RTCM_LOG_PREAMBLE "RTCM: "

/* Multiplier for glonass bias resolution scaling */
#define GLO_BIAS_RESOLUTION 50.0

/* How long to wait after receiving a 1230 message before accepting the 1033
 * message again */
#define MSG_1230_TIMEOUT_SEC 120

/* How long to wait after receiving an MSM message before accepting the legacy
 * 1002/1004/1010/1012 observation messages again */
#define MSM_TIMEOUT_SEC 5

/* Third party receiver bias value - these have been sourced from RTCM1230
 * message, the data can be found with the unit tests*/
#define TRIMBLE_BIAS_M 19.06
#define NOVATEL_BIAS_M (-71.94)
#define SEPTENTRIO_BIAS_M 0.0
#define TOPCON_BIAS_M 0.0
#define HEMISPHERE_BIAS_L1CA_M (-0.2)
#define HEMISPHERE_BIAS_L2P_M 3.4
#define JAVAD_BIAS_L1CA_M (-1.5)
#define JAVAD_BIAS_L2P_M (-8.1)
#define NAVCOM_BIAS_L1CA_M 0.4
#define NAVCOM_BIAS_L2P_M 2.1

/* Third party receiver values as seen on a GEO++ stream from the RTCM1230
 * message
 * Note: these are GEO++ simulated values and not direct from the manufacturer
 */
#define GPP_ASH1_BIAS_L1CA_M (-14.7)
#define GPP_ASH1_BIAS_L2P_M (-16.2)
#define GPP_HEM_BIAS_L1CA_M (-0.3)
#define GPP_HEM_BIAS_L2P_M 3.5
#define GPP_JAV_BIAS_L1CA_M (-1.5)
#define GPP_JAV_BIAS_L2P_M (-8.1)
#define GPP_JPS_BIAS_L1CA_M (-3.7)
#define GPP_JPS_BIAS_L2P_M 0.7
#define GPP_NOV_BIAS_L1CA_M (-70.7)
#define GPP_NOV_BIAS_L2P_M (-66.3)
#define GPP_NAV_BIAS_L1CA_M 0.4
#define GPP_NAV_BIAS_L2P_M 2.1
#define GPP_NVR_BIAS_L1CA_M 116.7
#define GPP_NVR_BIAS_L2P_M 187
#define GPP_SEP0_BIAS_L1CA_M (-119.9)
#define GPP_SEP0_BIAS_L2P_M (-116.6)
#define GPP_SEP_BIAS_L1CA_M 0.0
#define GPP_SEP_BIAS_L2P_M 0.0
#define GPP_SOK_BIAS_L1CA_M (-70.7)
#define GPP_SOK_BIAS_L2P_M (-66.3)
#define GPP_TPS0_BIAS_L1CA_M (-3.7)
#define GPP_TPS0_BIAS_L2P_M 0.7
#define GPP_TPS_BIAS_L1CA_M (-2.56)
#define GPP_TPS_BIAS_L2P_M 3.74
#define GPP_TRM_BIAS_L1CA_M 18.8
#define GPP_TRM_BIAS_L2P_M 23.2

void rtcm3_1005_to_sbp(const rtcm_msg_1005 *rtcm_1005,
                       msg_base_pos_ecef_t *sbp_base_pos);

void rtcm3_1006_to_sbp(const rtcm_msg_1006 *rtcm_1006,
                       msg_base_pos_ecef_t *sbp_base_pos);

void rtcm3_1033_to_sbp(const rtcm_msg_1033 *rtcm_1033,
                       msg_glo_biases_t *sbp_glo_bias);

void rtcm3_1230_to_sbp(const rtcm_msg_1230 *rtcm_1230,
                       msg_glo_biases_t *sbp_glo_bias);

void add_gps_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state);

void add_glo_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state);

void add_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                       gps_time_t *obs_time,
                       struct rtcm3_sbp_state *state);

void compute_gps_message_time(u32 tow_ms,
                              gps_time_t *obs_time,
                              const gps_time_t *rover_time);

void compute_gps_time(u32 tow_ms,
                      gps_time_t *obs_time,
                      const gps_time_t *rover_time,
                      struct rtcm3_sbp_state *state);

bool compute_glo_time(u32 tod_ms,
                      gps_time_t *obs_time,
                      const gps_time_t *rover_time,
                      struct rtcm3_sbp_state *state);

void beidou_tow_to_gps_tow(u32 *tow_ms);

void send_observations(struct rtcm3_sbp_state *state);

bool no_1230_received(struct rtcm3_sbp_state *state);

void send_1029(rtcm_msg_1029 *msg_1029, struct rtcm3_sbp_state *state);

void send_sbp_log_message(uint8_t level,
                          const uint8_t *message,
                          uint16_t length,
                          uint16_t stn_id,
                          const struct rtcm3_sbp_state *state);

void send_MSM_warning(const swiftnav_bitstream_t *bitstream,
                      struct rtcm3_sbp_state *state);

void send_buffer_full_error(const struct rtcm3_sbp_state *state);

void send_unsupported_code_warning(unsupported_code_t unsupported_code,
                                   struct rtcm3_sbp_state *state);

void add_msm_obs_to_buffer(const rtcm_msm_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state);

void rtcm3_msm_to_sbp(const rtcm_msm_message *msg,
                      msg_obs_t *new_sbp_obs,
                      struct rtcm3_sbp_state *state);

void rtcm_log_callback_fn(uint8_t level,
                          uint8_t *message,
                          uint16_t length,
                          void *context);

static inline bool gps_time_sec_valid(const gps_time_sec_t *t) {
  return (t->wn != INVALID_TIME) && (t->wn < MAX_WN) && (t->tow < WEEK_SECS);
};

bool rtcm3_gps_eph_to_sbp(rtcm_msg_eph *msg_eph,
                          msg_ephemeris_gps_t *sbp_gps_eph,
                          struct rtcm3_sbp_state *state);
void rtcm3_qzss_eph_to_sbp(rtcm_msg_eph *msg_eph,
                           msg_ephemeris_qzss_t *sbp_qzss_eph,
                           struct rtcm3_sbp_state *state);
bool rtcm3_glo_eph_to_sbp(rtcm_msg_eph *msg_eph,
                          msg_ephemeris_glo_t *sbp_glo_eph,
                          struct rtcm3_sbp_state *state);
bool rtcm3_gal_eph_to_sbp(const rtcm_msg_eph *msg_eph,
                          u8 source,
                          msg_ephemeris_gal_t *sbp_gal_eph,
                          struct rtcm3_sbp_state *state);
void rtcm3_bds_eph_to_sbp(rtcm_msg_eph *msg_eph,
                          msg_ephemeris_bds_t *sbp_bds_eph,
                          struct rtcm3_sbp_state *state);

void rtcm3_ssr_separate_orbit_clock_to_sbp(rtcm_msg_clock *msg_clock,
                                           rtcm_msg_orbit *msg_orbit,
                                           struct rtcm3_sbp_state *state);
void rtcm3_ssr_orbit_clock_to_sbp(rtcm_msg_orbit_clock *msg_orbit_clock,
                                  struct rtcm3_sbp_state *state);
void rtcm3_ssr_code_bias_to_sbp(rtcm_msg_code_bias *msg_code_biases,
                                struct rtcm3_sbp_state *state);
void rtcm3_ssr_phase_bias_to_sbp(rtcm_msg_phase_bias *msg_phase_biases,
                                 struct rtcm3_sbp_state *state);

static inline u16 rtcm_stn_to_sbp_sender_id(u16 rtcm_id) {
  /* To avoid conflicts with reserved low number sender ID's we or
   * on the highest nibble as RTCM sender ID's are 12 bit */
  return rtcm_id | 0xF080;
}

void handle_ndf_frame(const rtcm_msg_ndf *msg_ndf,
                      struct rtcm3_sbp_state *state);

#endif /* GNSS_CONVERTERS_RTCM3_SBP_INTERNAL_H */
