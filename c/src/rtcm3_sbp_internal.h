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

#include <rtcm3/messages.h>
#include <rtcm3/msm_utils.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>
#include "gnss-converters/rtcm3_sbp.h"

#define MSG_OBS_P_MULTIPLIER ((double)5e1)
#define MSG_OBS_CN0_MULTIPLIER ((float)4)
#define MSG_OBS_LF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_DF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_FLAGS_CODE_VALID ((u8)(1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID ((u8)(1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN ((u8)(1 << 2))
#define MSG_OBS_FLAGS_DOPPLER_VALID ((u8)(1 << 3))
#define MSG_OBS_FLAGS_RAIM_EXCLUSION ((u8)(1 << 7))

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

#define MS_TO_S 1e-3
#define S_TO_MS 1e3

extern bool rtcm3_debug;

/** Number of milliseconds in a second. */
#define SECS_MS 1000
#define SEC_IN_DAY 86400
#define SEC_IN_WEEK 604800
#define SEC_IN_HOUR 3600
#define SEC_IN_MINUTE 60
#define SEC_IN_15MINUTES (60 * 15)

/** Maximum time difference representable by s32 seconds */
#define MAX_WEEK_DIFF (INT32_MAX / SEC_IN_WEEK - 1)

/** Threshold for base measurements coming from the future */
#define BASE_FUTURE_THRESHOLD_S 10
/** Threshold for GLO time difference from rover */
#define GLO_SANITY_THRESHOLD_S 30

/** UTC (SU) offset (hours) */
#define UTC_SU_OFFSET 3

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
#define MSM_TIMEOUT_SEC 5

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

/** Receiver-dependent flags for the SBP to RTCM conversion */

/* GPS Indicator DF022 */
#define PIKSI_GPS_SERVICE_SUPPORTED 1

/* GLONASS Indicator DF023 */
#define PIKSI_GLO_SERVICE_SUPPORTED 1

/* Galileo Indicator DF024 */
#define PIKSI_GAL_SERVICE_SUPPORTED 1

/* Single Receiver Oscillator Indicator DF142
 * 0 - All raw data observations in messages 1001-1004 and 1009-1012 may be
 *     measured at different instants. This indicator should be set to “0”
 *     unless all the conditions for “1” are clearly met.
 * 1 - All raw data observations in messages 1001-1004 and 1009-1012
 *     are measured at the same instant, as described in Section 3.1.4. */
#define PIKSI_SINGLE_OSCILLATOR_INDICATOR 1

/* Quarter Cycle Indicator DF364
 * 00 - Correction status unspecified
 * 01 - PhaseRanges in Message Types 1001, 1002, 1003, 1004, 1009,
 *      1010, 1011, 1012 are corrected in such a way that whenever
 *      PhaseRanges for different signals on the same frequency are
 *      present in these messages, they are guaranteed to be in phase and
 *      thus shall show no Quarter-Cycle bias between them (see Table
 *      3.1-5 for details on the adjustments made). Double differences
 *      of PhaseRanges tracked with different signals shall show no
 *      Quarter- Cycle differences.
 * 10 - Phase observations are not corrected. Double differences may
 *      show Quarter-Cycle differences for PhaseRanges based on
 *      different signals on the same frequency. Processing will require
 *      appropriate corrections. */
#define PIKSI_QUARTER_CYCLE_INDICATOR 0

/* Reference-Station Indicator DF141
 *   0 - Real, Physical Reference Station
 *   1 - Non-Physical or Computed Reference Station */
#define PIKSI_REFERENCE_STATION_INDICATOR 0

/* GLONASS Code-Phase bias indicator DF421
 * 0 = The GLONASS Pseudorange and Phaserange observations in the
 *     data stream are not aligned to the same measurement epoch.
 * 1 = The GLONASS Pseudorange and Phaserange observations in the
 *     data stream are aligned to the same measurement epoch.
 * Note: must be 0 when transmitting Legacy observations */
#define PIKSI_GLO_CODE_PHASE_BIAS_INDICATOR 0

/* Divergence free flag DF007 (GPS) and DF036 (GLO)
 * 0 - Divergence-free smoothing not used
 * 1 - Divergence-free smoothing used */
#define PIKSI_DIVERGENCE_FREE_INDICATOR 0

/* Smoothing Interval DF008 (GPS) and DF037 (GLO)
 * Integration period over which reference station pseudorange code phase
 * measurements are averaged using carrier phase information. */
#define PIKSI_SMOOTHING_INTERVAL 0

/* Clock Steering Indicator DF411
 * 0 – clock steering is not applied
 *     In this case receiver clock must be kept in the range of ±1 ms
 *     (approximately ±300 km)
 * 1 – clock steering has been applied
 *     In this case receiver clock must be kept in the range of ±1 microsecond
 *     (approximately ±300 meters).
 * 2 – unknown clock steering status
 * 3 – reserved */
#define PIKSI_CLOCK_STEERING_INDICATOR 1

/* External Clock Indicator DF412
 * 0 – internal clock is used
 * 1 – external clock is used, clock status is “locked”
 * 2 – external clock is used, clock status is “not locked”, which may
 *     indicate external clock failure and that the transmitted data may not be
 *     reliable.
 * 3 – unknown clock is used */
#define PIKSI_EXT_CLOCK_INDICATOR 0

void rtcm3_1005_to_sbp(const rtcm_msg_1005 *rtcm_1005,
                       msg_base_pos_ecef_t *sbp_base_pos);
void sbp_to_rtcm3_1005(const msg_base_pos_ecef_t *sbp_base_pos,
                       u16 sender_id,
                       rtcm_msg_1005 *rtcm_1005);

void rtcm3_1006_to_sbp(const rtcm_msg_1006 *rtcm_1006,
                       msg_base_pos_ecef_t *sbp_base_pos);
void sbp_to_rtcm3_1006(const msg_base_pos_ecef_t *sbp_base_pos,
                       u16 station_id,
                       rtcm_msg_1006 *rtcm_1006);

void rtcm3_1033_to_sbp(const rtcm_msg_1033 *rtcm_1033,
                       msg_glo_biases_t *sbp_glo_bias);

void rtcm3_1230_to_sbp(const rtcm_msg_1230 *rtcm_1230,
                       msg_glo_biases_t *sbp_glo_bias);
void sbp_to_rtcm3_1230(const msg_glo_biases_t *sbp_glo_bias,
                       u16 sender_id,
                       rtcm_msg_1230 *rtcm_1230);

void rtcm3_to_sbp(const rtcm_obs_message *rtcm_obs,
                  msg_obs_t *new_sbp_obs,
                  struct rtcm3_sbp_state *state);

u16 encode_rtcm3_frame(const void *rtcm_msg, u16 message_type, u8 *frame);

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

void compute_glo_time(u32 tod_ms,
                      gps_time_t *obs_time,
                      const gps_time_t *rover_time,
                      struct rtcm3_sbp_state *state);

double compute_glo_tod(uint32_t gps_tow_ms,
                       const struct rtcm3_out_state *state);

void sbp_buffer_to_msm(const struct rtcm3_out_state *state);

void beidou_tow_to_gps_tow(u32 *tow_ms);

void gps_tow_to_beidou_tow(u32 *tow_ms);

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

void send_unsupported_code_warning(const unsupported_code_t unsupported_code,
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

double sbp_diff_time(const sbp_gps_time_t *end,
                     const sbp_gps_time_t *beginning);

static inline bool gps_time_sec_valid(const gps_time_sec_t *t) {
  return (t->wn != INVALID_TIME) && (t->wn < MAX_WN) && (t->tow < SEC_IN_WEEK);
};

void rtcm3_gps_eph_to_sbp(rtcm_msg_eph *msg_eph,
                          msg_ephemeris_gps_t *sbp_gps_eph,
                          struct rtcm3_sbp_state *state);
void rtcm3_glo_eph_to_sbp(rtcm_msg_eph *msg_eph,
                          msg_ephemeris_glo_t *sbp_glo_eph,
                          struct rtcm3_sbp_state *state);
void rtcm3_gal_eph_to_sbp(rtcm_msg_eph *msg_eph,
                          msg_ephemeris_gal_t *sbp_glo_eph,
                          struct rtcm3_sbp_state *state);
void rtcm3_bds_eph_to_sbp(rtcm_msg_eph *msg_eph,
                          msg_ephemeris_bds_t *sbp_glo_eph,
                          struct rtcm3_sbp_state *state);

void rtcm3_ssr_orbit_clock_to_sbp(rtcm_msg_orbit_clock *msg_orbit_clock,
                                  struct rtcm3_sbp_state *state);
void rtcm3_ssr_code_bias_to_sbp(rtcm_msg_code_bias *msg_code_biases,
                                struct rtcm3_sbp_state *state);
void rtcm3_ssr_phase_bias_to_sbp(rtcm_msg_phase_bias *msg_phase_biases,
                                 struct rtcm3_sbp_state *state);

#endif /* GNSS_CONVERTERS_RTCM3_SBP_H */
