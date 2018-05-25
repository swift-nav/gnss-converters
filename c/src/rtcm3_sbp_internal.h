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

#ifndef GNSS_CONVERTERS_RTCM3_SBP_H
#define GNSS_CONVERTERS_RTCM3_SBP_H

#include <rtcm3_messages.h>
#include <rtcm3_sbp.h>

#define MSG_OBS_P_MULTIPLIER ((double)5e1)
#define MSG_OBS_CN0_MULTIPLIER ((float)4)
#define MSG_OBS_LF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_DF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_FLAGS_CODE_VALID ((u8)(1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID ((u8)(1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN ((u8)(1 << 2))
#define MSG_OBS_FLAGS_DOPPLER_VALID ((u8)(1 << 3))

#define RTCM_1029_LOGGING_LEVEL (6u)        /* This represents LOG_INFO */
#define RTCM_MSM_LOGGING_LEVEL (4u)         /* This represents LOG_WARN */
#define RTCM_BUFFER_FULL_LOGGING_LEVEL (3u) /* This represents LOG_ERROR */

#define MS_TO_S 1e-3
#define S_TO_MS 1e3

extern bool rtcm3_debug;

/** Number of milliseconds in a second. */
#define SECS_MS 1000
#define SEC_IN_DAY 86400
#define SEC_IN_WEEK 604800
#define SEC_IN_HOUR 3600

/** Constant difference of Beidou time from GPS time */
#define BDS_SECOND_TO_GPS_SECOND 14

/* PREAMBLE to append to an RTCM3 log message */
#define RTCM_LOG_PREAMBLE "RTCM: "

/* Multiplier for glonass bias resolution scaling */
#define GLO_BIAS_RESOLUTION 50.0

/* How long to wait after receiving a 1230 message before accepting the 1033
 * message again */
#define MSG_1230_TIMEOUT_SEC 120

/* How long to wait after receiving an MSM message before accepting the legacy
 * 1002/1004/1010/1012 observation messages again */
#define MSM_TIMEOUT_SEC 60

/* Third party receiver bias value - these have been sourced from RTCM1230
 * message, the data can be found with the unit tests*/
#define TRIMBLE_BIAS_M 19.06
#define NOVATEL_BIAS_M -71.94
#define SEPTENTRIO_BIAS_M 0.0
#define TOPCON_BIAS_M 0.0
#define HEMISPHERE_BIAS_L1CA_M -0.2
#define HEMISPHERE_BIAS_L2P_M 3.4
#define JAVAD_BIAS_L1CA_M -1.5
#define JAVAD_BIAS_L2P_M -8.1
#define NAVCOM_BIAS_L1CA_M 0.4
#define NAVCOM_BIAS_L2P_M 2.1

/* Third party receiver values as seen on a GEO++ stream from the RTCM1230
 * message
 * Note: these are GEO++ simulated values and not direct from the manufacturer
 */
#define GPP_ASH1_BIAS_L1CA_M -14.7
#define GPP_ASH1_BIAS_L2P_M -16.2
#define GPP_HEM_BIAS_L1CA_M -0.3
#define GPP_HEM_BIAS_L2P_M 3.5
#define GPP_JAV_BIAS_L1CA_M -1.5
#define GPP_JAV_BIAS_L2P_M -8.1
#define GPP_JPS_BIAS_L1CA_M -3.7
#define GPP_JPS_BIAS_L2P_M 0.7
#define GPP_NOV_BIAS_L1CA_M -70.7
#define GPP_NOV_BIAS_L2P_M -66.3
#define GPP_NAV_BIAS_L1CA_M 0.4
#define GPP_NAV_BIAS_L2P_M 2.1
#define GPP_NVR_BIAS_L1CA_M 116.7
#define GPP_NVR_BIAS_L2P_M 187
#define GPP_SEP0_BIAS_L1CA_M -119.9
#define GPP_SEP0_BIAS_L2P_M -116.6
#define GPP_SEP_BIAS_L1CA_M 0.0
#define GPP_SEP_BIAS_L2P_M 0.0
#define GPP_SOK_BIAS_L1CA_M -70.7
#define GPP_SOK_BIAS_L2P_M -66.3
#define GPP_TPS0_BIAS_L1CA_M -3.7
#define GPP_TPS0_BIAS_L2P_M 0.7
#define GPP_TPS_BIAS_L1CA_M -2.56
#define GPP_TPS_BIAS_L2P_M 3.74
#define GPP_TRM_BIAS_L1CA_M 18.8
#define GPP_TRM_BIAS_L2P_M 23.2

u8 encode_lock_time(double nm_lock_time);
double decode_lock_time(u8 sbp_lock_time);

void sbp_to_rtcm3_obs(const msg_obs_t *sbp_obs,
                      const u8 msg_size,
                      rtcm_obs_message *rtcm_obs);

void rtcm3_1005_to_sbp(const rtcm_msg_1005 *rtcm_1005,
                       msg_base_pos_ecef_t *sbp_base_pos);
void sbp_to_rtcm3_1005(const msg_base_pos_ecef_t *sbp_base_pos,
                       rtcm_msg_1005 *rtcm_1005);

void rtcm3_1006_to_sbp(const rtcm_msg_1006 *rtcm_1006,
                       msg_base_pos_ecef_t *sbp_base_pos);
void sbp_to_rtcm3_1006(const msg_base_pos_ecef_t *sbp_base_pos,
                       rtcm_msg_1006 *rtcm_1006);

void rtcm3_1033_to_sbp(const rtcm_msg_1033 *rtcm_1033,
                       msg_glo_biases_t *sbp_glo_bias);

void rtcm3_1230_to_sbp(const rtcm_msg_1230 *rtcm_1230,
                       msg_glo_biases_t *sbp_glo_bias);
void sbp_to_rtcm3_1230(const msg_glo_biases_t *sbp_glo_bias,
                       rtcm_msg_1230 *rtcm_1230);

void encode_RTCM_obs(const rtcm_obs_message *rtcm_msg);

void rtcm3_to_sbp(const rtcm_obs_message *rtcm_obs,
                  msg_obs_t *sbp_obs,
                  struct rtcm3_sbp_state *state);

void add_gps_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state);

void add_glo_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state);

void add_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                       gps_time_sec_t *new_sbp_obs,
                       struct rtcm3_sbp_state *state);

void compute_gps_time(double tow_ms,
                      gps_time_sec_t *new_sbp_obs,
                      const gps_time_sec_t *rover_time,
                      struct rtcm3_sbp_state *state);

void compute_glo_time(double tod_ms,
                      gps_time_sec_t *obs_time,
                      const gps_time_sec_t *rover_time,
                      const s8 leap_second);

void send_observations(struct rtcm3_sbp_state *state);

bool no_1230_received(struct rtcm3_sbp_state *state);

void send_1029(rtcm_msg_1029 *msg_1029, struct rtcm3_sbp_state *state);

void send_sbp_log_message(const uint8_t level,
                          const uint8_t *message,
                          const uint16_t length,
                          const uint16_t stn_id,
                          const struct rtcm3_sbp_state *state);

void send_MSM_warning(const uint8_t *frame, struct rtcm3_sbp_state *state);

void send_buffer_full_error(const struct rtcm3_sbp_state *state);

void add_msm_obs_to_buffer(const rtcm_msm_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state);

void rtcm3_msm_to_sbp(const rtcm_msm_message *msg,
                      msg_obs_t *new_sbp_obs,
                      const struct rtcm3_sbp_state *state);

#endif /* GNSS_CONVERTERS_RTCM3_SBP_H */
