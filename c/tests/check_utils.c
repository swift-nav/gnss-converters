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

#include <check.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "check_suites.h"
#include "check_utils.h"
#include "config.h"

#define FREQ_TOL 1e-3

static struct rtcm3_sbp_state state;
static struct rtcm3_out_state out_state;

/* fixture globals and functions */

void utils_setup(void) {
  memset(&state, 0, sizeof(state));
  memset(&out_state, 0, sizeof(out_state));
  state.time_from_rover_obs.wn = 1945;
  state.time_from_rover_obs.tow = 211190;

  rtcm2sbp_set_leap_second(18, &state);
  sbp2rtcm_set_leap_second(18, &out_state);
}

/* end fixtures */

START_TEST(test_compute_glo_time) {
  for (u8 day = 0; day < 7; day++) {
    for (u8 hour = 0; hour < 24; hour++) {
      for (u8 min = 0; min < 60; min++) {
        for (u8 sec = 0; sec < 60; sec++) {
          u32 tod = hour * SEC_IN_HOUR + min * SEC_IN_MINUTE + sec;
          gps_time_t rover_time = {.tow = day * SEC_IN_DAY + tod, .wn = 1945};
          rtcm2sbp_set_gps_time(&rover_time, &state);
          u32 glo_tod_ms = (tod + UTC_SU_OFFSET * SEC_IN_HOUR) * S_TO_MS;
          if (glo_tod_ms > SEC_IN_DAY * S_TO_MS) {
            glo_tod_ms -= SEC_IN_DAY * S_TO_MS;
          }
          gps_time_t expected_time = rover_time;
          expected_time.tow += state.leap_seconds;
          if (expected_time.tow >= SEC_IN_WEEK) {
            expected_time.tow -= SEC_IN_WEEK;
            expected_time.wn++;
          }

          gps_time_t obs_time;
          compute_glo_time(glo_tod_ms, &obs_time, &rover_time, &state);
          ck_assert_uint_eq(obs_time.wn, expected_time.wn);
          ck_assert_uint_eq((u32)rint(obs_time.tow),
                            (u32)rint(expected_time.tow));
        }
      }
    }
  }
}
END_TEST

START_TEST(test_glo_time_conversion) {
  for (u32 tow = 0; tow < SEC_IN_WEEK; tow++) {
    gps_time_t rover_time = {.tow = tow, .wn = 1945};
    rtcm2sbp_set_gps_time(&rover_time, &state);

    u32 glo_tod_ms =
        compute_glo_tod_ms((u32)rint(rover_time.tow * S_TO_MS), &out_state);

    gps_time_t obs_time;
    compute_glo_time(glo_tod_ms, &obs_time, &rover_time, &state);
    ck_assert_uint_eq(obs_time.wn, rover_time.wn);
    ck_assert_uint_eq((u32)rint(obs_time.tow), (u32)rint(rover_time.tow));
  }
}
END_TEST

START_TEST(test_msm_sid_conversion) {
  rtcm_msm_header header;
  /* GPS message */

  header.msg_num = 1074;
  /* PRNs 1, 2 and 20 */
  memset((void *)&header.satellite_mask, 0, sizeof(header.satellite_mask));
  header.satellite_mask[0] = true;  /* PRN 1 */
  header.satellite_mask[1] = true;  /* PRN 2 */
  header.satellite_mask[63] = true; /* invalid PRN 64 */
  /* signal ids 2 (L1CA) and 15 (L2CM) */
  memset((void *)&header.signal_mask, 0, sizeof(header.signal_mask));
  header.signal_mask[1] = true;
  header.signal_mask[14] = true;

  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_GPS);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 1);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 2);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_GPS_L1CA);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_GPS_L2CM);
  double freq;
  ck_assert(msm_signal_frequency(&header, 0, 0, false, &freq) &&
            fabs(freq - GPS_L1_HZ) < FREQ_TOL);
  ck_assert(msm_signal_frequency(&header, 1, 0, false, &freq) &&
            fabs(freq - GPS_L2_HZ) < FREQ_TOL);

  /* GLO */
  header.msg_num = 1084;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_GLO);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 1);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 2);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_GLO_L1OF);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_INVALID);
  uint8_t fcn = 3;
  ck_assert(
      msm_signal_frequency(&header, 0, fcn + MSM_GLO_FCN_OFFSET, true, &freq) &&
      fabs(freq - (GLO_L1_HZ + fcn * GLO_L1_DELTA_HZ)) < FREQ_TOL);
  ck_assert(
      !msm_signal_frequency(&header, 1, fcn + MSM_GLO_FCN_OFFSET, true, &freq));

  /* GAL */
  header.msg_num = 1094;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_GAL);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 1);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 2);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_GAL_E1C);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_GAL_E7Q);
  ck_assert(msm_signal_frequency(&header, 0, 0, false, &freq) &&
            fabs(freq - GAL_E1_HZ) < FREQ_TOL);
  ck_assert(msm_signal_frequency(&header, 1, 0, false, &freq) &&
            fabs(freq - GAL_E7_HZ) < FREQ_TOL);

  /* SBAS */
  header.msg_num = 1104;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_SBAS);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 120);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 121);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_SBAS_L1CA);
  ck_assert(msm_signal_frequency(&header, 0, 0, false, &freq) &&
            fabs(freq - SBAS_L1_HZ) < FREQ_TOL);

  /* QZS PRNs start from 193 */
  header.msg_num = 1114;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_QZS);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 193);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 194);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_QZS_L1CA);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_QZS_L2CM);
  ck_assert(msm_signal_frequency(&header, 0, 0, false, &freq) &&
            fabs(freq - QZS_L1_HZ) < FREQ_TOL);
  ck_assert(msm_signal_frequency(&header, 1, 0, false, &freq) &&
            fabs(freq - QZS_L2_HZ) < FREQ_TOL);

  /* BDS */
  header.msg_num = 1124;
  header.signal_mask[13] = true;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_BDS);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 1);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 2);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_BDS2_B1);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_BDS2_B2);
  ck_assert(msm_signal_frequency(&header, 0, 0, false, &freq) &&
            fabs(freq - BDS2_B11_HZ) < FREQ_TOL);
  ck_assert(msm_signal_frequency(&header, 1, 0, false, &freq) &&
            fabs(freq - BDS2_B2_HZ) < FREQ_TOL);
}
END_TEST

START_TEST(test_msm_code_prn_conversion) {
  rtcm_msm_header header;

  for (u8 cons = 0; cons < RTCM_CONSTELLATION_COUNT; cons++) {
    header.msg_num = to_msm_msg_num(cons, MSM4);
    for (u8 sat_id = 0; sat_id < MSM_SATELLITE_MASK_SIZE; sat_id++) {
      header.satellite_mask[sat_id] = true;
      u8 prn = msm_sat_to_prn(&header, sat_id);
      if (PRN_INVALID == prn) {
        continue;
      }
      /* check that each valid PRN convers back to the original satellite id */
      ck_assert_uint_eq(sat_id, prn_to_msm_sat_id(prn, cons));
      /* index is the same in this case when all mask bits are filled */
      ck_assert_uint_eq(sat_id, prn_to_msm_sat_index(&header, prn));
    }

    for (u8 signal_id = 0; signal_id < MSM_SIGNAL_MASK_SIZE; signal_id++) {
      header.signal_mask[signal_id] = true;
      code_t code = msm_signal_to_code(&header, signal_id);
      if (CODE_INVALID == code) {
        continue;
      }
      /* check that each valid code converts back to the original signal id */
      u8 converted_signal_id = code_to_msm_signal_id(code, cons);

      if (RTCM_CONSTELLATION_GPS == cons && signal_id == 3) {
        /* RTCM GPS 1W maps into L1P */
        ck_assert_uint_eq(converted_signal_id, 2);
      } else if (RTCM_CONSTELLATION_GPS == cons && signal_id == 9) {
        /* RTCM GPS 1W maps into L2P */
        ck_assert_uint_eq(converted_signal_id, 8);
      } else {
        /* everything else is 1-to-1 */
        ck_assert_uint_eq(converted_signal_id, signal_id);
        ck_assert_uint_eq(code_to_msm_signal_index(&header, code), signal_id);
      }
    }
  }
}
END_TEST

START_TEST(test_msm_glo_fcn) {
  rtcm_msm_header header;

  header.msg_num = 1084;
  memset((void *)&header.satellite_mask, 0, sizeof(header.satellite_mask));
  header.satellite_mask[0] = true;  /* PRN 1 */
  header.satellite_mask[2] = true;  /* PRN 3 */
  header.satellite_mask[63] = true; /* invalid PRN 64 */
  /* signal id 2 (L1CA)) */
  memset((void *)&header.signal_mask, 0, sizeof(header.signal_mask));
  header.signal_mask[1] = true;

  uint8_t sat_info[2] = {3 + MSM_GLO_FCN_OFFSET, -6 + MSM_GLO_FCN_OFFSET};

  uint8_t glo_fcn;

  /* FCN for both satellites available in sat_info */
  ck_assert(msm_get_glo_fcn(&header, 0, sat_info[0], NULL, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, 3 + MSM_GLO_FCN_OFFSET);

  ck_assert(msm_get_glo_fcn(&header, 1, sat_info[1], NULL, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, -6 + MSM_GLO_FCN_OFFSET);

  /* sat info invalid for first satellite, FCN not available */
  sat_info[0] = MSM_GLO_FCN_UNKNOWN;
  ck_assert(!msm_get_glo_fcn(&header, 0, sat_info[0], NULL, &glo_fcn));

  /* FCN map indexed by PRN */
  uint8_t fcn_map[RTCM_MAX_SATS];
  fcn_map[0] = MSM_GLO_FCN_UNKNOWN;
  fcn_map[1] = 2 + MSM_GLO_FCN_OFFSET; /* PRN 1 */
  fcn_map[2] = MSM_GLO_FCN_UNKNOWN;
  fcn_map[3] = MSM_GLO_FCN_UNKNOWN; /* PRN 3 */

  /* FCN for first satellite from FCN MAP */
  ck_assert(msm_get_glo_fcn(&header, 0, sat_info[0], fcn_map, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, 2 + MSM_GLO_FCN_OFFSET);

  /* FCN MAP invalid for second satellite, value in sat_info used */
  ck_assert(msm_get_glo_fcn(&header, 1, sat_info[1], fcn_map, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, -6 + MSM_GLO_FCN_OFFSET);

  /* both FCN map and sat_info invalid, invalid FCN returned */
  sat_info[1] = MSM_GLO_FCN_UNKNOWN;
  ck_assert(!msm_get_glo_fcn(&header, 1, sat_info[1], fcn_map, &glo_fcn));

  /* add valid value in the FCN map */
  fcn_map[3] = -5 + MSM_GLO_FCN_OFFSET;
  ck_assert(msm_get_glo_fcn(&header, 1, sat_info[1], fcn_map, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, -5 + MSM_GLO_FCN_OFFSET);
}
END_TEST

START_TEST(test_msm_add_to_header) {
  rtcm_msm_header header;
  memset(&header, 0, sizeof(header));
  header.msg_num = 1074;
  msm_add_to_header(&header, CODE_GPS_L1CA, 1);
  msm_add_to_header(&header, CODE_GPS_L1CA, 2);
  msm_add_to_header(&header, CODE_GPS_L1CA, 3);
  msm_add_to_header(&header, CODE_GPS_L2CM, 1);

  ck_assert_uint_eq(msm_get_num_satellites(&header), 3);
  ck_assert_uint_eq(msm_get_num_signals(&header), 2);

  /* invalid code, should get rejected */
  ck_assert(!msm_add_to_header(&header, CODE_INVALID, 1));
  /* invalid PRN, should get rejected */
  ck_assert(!msm_add_to_header(&header, CODE_GPS_L1CA, 50));

  msm_add_to_header(&header, CODE_GPS_L1P, 1);
  msm_add_to_header(&header, CODE_GPS_L2P, 1);
  msm_add_to_header(&header, CODE_GPS_L2CL, 1);
  msm_add_to_header(&header, CODE_GPS_L2CX, 1);
  msm_add_to_header(&header, CODE_GPS_L5I, 1);
  msm_add_to_header(&header, CODE_GPS_L5Q, 1);
  msm_add_to_header(&header, CODE_GPS_L5X, 1);
  msm_add_to_header(&header, CODE_GPS_L1CI, 1);
  msm_add_to_header(&header, CODE_GPS_L1CQ, 1);
  ck_assert_uint_eq(msm_get_num_signals(&header), 11);

  ck_assert(msm_add_to_header(&header, CODE_GPS_L1CA, 4));
  ck_assert_uint_eq(msm_get_num_satellites(&header), 4);
  ck_assert(msm_add_to_header(&header, CODE_GPS_L1CA, 5));
  ck_assert_uint_eq(msm_get_num_satellites(&header), 5);

  /* adding sixth satellite fails because cell mask holds only 64 items */
  ck_assert(!msm_add_to_header(&header, CODE_GPS_L1CA, 6));

  /* can still add signals to existing satellites */
  ck_assert(msm_add_to_header(&header, CODE_GPS_L1P, 5));

  msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 1);
  msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 2);
  msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 3);
  msm_add_to_cell_mask(&header, CODE_GPS_L2CM, 1);
  ck_assert_uint_eq(msm_get_num_cells(&header), 4);

  /* satellite not in cell mask, should get rejected */
  ck_assert(!msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 6));
  /* signal not in cell mask, should get rejected */
  ck_assert(!msm_add_to_cell_mask(&header, CODE_GPS_L1CX, 1));
  /* invalid code, should get rejected */
  ck_assert(!msm_add_to_cell_mask(&header, CODE_INVALID, 1));
  /* invalid PRN, should get rejected */
  ck_assert(!msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 50));
}
END_TEST

Suite *utils_suite(void) {
  Suite *s = suite_create("Utils");

  TCase *tc_utils = tcase_create("Utilities");
  tcase_add_checked_fixture(tc_utils, utils_setup, NULL);
  tcase_add_test(tc_utils, test_compute_glo_time);
  tcase_add_test(tc_utils, test_glo_time_conversion);
  tcase_add_test(tc_utils, test_msm_sid_conversion);
  tcase_add_test(tc_utils, test_msm_code_prn_conversion);
  tcase_add_test(tc_utils, test_msm_glo_fcn);
  tcase_add_test(tc_utils, test_msm_add_to_header);

  suite_add_tcase(s, tc_utils);

  return s;
}
