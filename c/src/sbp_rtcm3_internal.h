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

#ifndef GNSS_CONVERTERS_SBP_RTCM3_INTERNAL_H
#define GNSS_CONVERTERS_SBP_RTCM3_INTERNAL_H

#include <rtcm3/messages.h>
#include <rtcm3/msm_utils.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>
#include "gnss-converters/sbp_rtcm3.h"

#define MSG_OBS_P_MULTIPLIER ((double)5e1)
#define MSG_OBS_CN0_MULTIPLIER ((float)4)
#define MSG_OBS_LF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_DF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_FLAGS_CODE_VALID ((u8)(1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID ((u8)(1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN ((u8)(1 << 2))
#define MSG_OBS_FLAGS_DOPPLER_VALID ((u8)(1 << 3))
#define MSG_OBS_FLAGS_RAIM_EXCLUSION ((u8)(1 << 7))

#define MS_TO_S 1e-3
#define S_TO_MS 1000

/** Number of milliseconds in a second. */
#define SECS_MS 1000
#define SEC_IN_DAY 86400
#define SEC_IN_WEEK 604800
#define SEC_IN_HOUR 3600
#define SEC_IN_MINUTE 60
#define SEC_IN_15MINUTES (60 * 15)

/** UTC (SU) offset (hours) */
#define UTC_SU_OFFSET 3

/* Multiplier for glonass bias resolution scaling */
#define GLO_BIAS_RESOLUTION 50.0

/** Receiver-dependent flags for the SBP to RTCM conversion */

/* GPS Indicator DF022 */
#define PIKSI_GPS_SERVICE_SUPPORTED 1

/* GLONASS Indicator DF023 */
#define PIKSI_GLO_SERVICE_SUPPORTED 1

/* Galileo Indicator DF024 */
#define PIKSI_GAL_SERVICE_SUPPORTED 1

/* Constant difference of Galileo time from GPS time */
#define GAL_WEEK_TO_GPS_WEEK 1024

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

void sbp_to_rtcm3_1005(const msg_base_pos_ecef_t *sbp_base_pos,
                       rtcm_msg_1005 *rtcm_1005,
                       const struct rtcm3_out_state *state);

void sbp_to_rtcm3_1006(const msg_base_pos_ecef_t *sbp_base_pos,
                       rtcm_msg_1006 *rtcm_1006,
                       const struct rtcm3_out_state *state);

void generate_rtcm3_1033(rtcm_msg_1033 *rtcm_1033,
                         const struct rtcm3_out_state *state);
void rtcm3_1033_to_1008(const rtcm_msg_1033 *rtcm_1033,
                        rtcm_msg_1008 *rtcm_1008);

void sbp_to_rtcm3_1230(const msg_glo_biases_t *sbp_glo_bias,
                       rtcm_msg_1230 *rtcm_1230,
                       const struct rtcm3_out_state *state);

void sbp_to_rtcm3_gps_eph(const msg_ephemeris_gps_t *sbp_gps_eph,
                          rtcm_msg_eph *msg_eph,
                          const struct rtcm3_out_state *state);

void sbp_to_rtcm3_gal_eph(const msg_ephemeris_gal_t *sbp_gps_eph,
                          rtcm_msg_eph *msg_eph,
                          const struct rtcm3_out_state *state);

u16 encode_rtcm3_frame(const void *rtcm_msg, u16 message_type, u8 *frame);

uint32_t compute_glo_tod_ms(uint32_t gps_tow_ms,
                            const struct rtcm3_out_state *state);

void sbp_buffer_to_msm(const struct rtcm3_out_state *state);

void gps_tow_to_beidou_tow(u32 *tow_ms);

double sbp_diff_time(const sbp_gps_time_t *end,
                     const sbp_gps_time_t *beginning);

#endif /* GNSS_CONVERTERS_SBP_RTCM3_INTERNAL_H */
