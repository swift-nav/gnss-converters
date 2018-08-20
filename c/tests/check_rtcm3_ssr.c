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

#include <check.h>
#include <libsbp/ssr.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <config.h>

#include "check_rtcm3.h"
#include "check_suites.h"

static bool gps_orbit_clock_processed = false;
static bool gps_code_bias_processed = false;
static bool gps_phase_bias_processed = false;
static bool glo_orbit_clock_processed = false;
static bool glo_code_bias_processed = false;
static bool gal_orbit_clock_processed = false;
static bool gal_code_bias_processed = false;
static bool gal_phase_bias_processed = false;
static bool bds_orbit_clock_processed = false;
static bool bds_code_bias_processed = false;
static bool bds_phase_bias_processed = false;

void sbp_callback_gps_orbit_clock(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_ORBIT_CLOCK && !msg_checked) {
    msg_ssr_orbit_clock_t *sbp_orbit_clock = (msg_ssr_orbit_clock_t *)buffer;
    if (sbp_orbit_clock->sid.code !=
        constellation_to_l1_code(CONSTELLATION_GPS)) {
      // This is not a GPS message, wait for first GPS message
      return;
    }
    msg_checked = true;
    gps_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 171680);
    ck_assert(sbp_orbit_clock->sid.sat == 1);
    ck_assert(sbp_orbit_clock->sid.code == 0);
    ck_assert(sbp_orbit_clock->update_interval == 2);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 30);
    ck_assert(sbp_orbit_clock->radial == 4432);
    ck_assert(sbp_orbit_clock->along == -559);
    ck_assert(sbp_orbit_clock->cross == 1394);
    ck_assert(sbp_orbit_clock->dot_radial == 14);
    ck_assert(sbp_orbit_clock->dot_along == 18);
    ck_assert(sbp_orbit_clock->dot_cross == -22);
    ck_assert(sbp_orbit_clock->c0 == -268);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_gps_code_bias(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_CODE_BIASES && !msg_checked) {
    msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t *)buffer;
    if (sbp_code_bias->sid.code !=
        constellation_to_l1_code(CONSTELLATION_GPS)) {
      // This is not a GPS message, wait for first GPS message
      return;
    }
    msg_checked = true;
    gps_code_bias_processed = true;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 1);
    ck_assert(sbp_code_bias->sid.code == 0);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);

    ck_assert(sbp_code_bias->biases[0].value == -340);
    ck_assert(sbp_code_bias->biases[0].code == 0);
    ck_assert(sbp_code_bias->biases[1].value == -368);
    ck_assert(sbp_code_bias->biases[1].code == 2);
    ck_assert(sbp_code_bias->biases[2].value == -338);
    ck_assert(sbp_code_bias->biases[2].code == 19);
    ck_assert(sbp_code_bias->biases[3].value == -567);
    ck_assert(sbp_code_bias->biases[3].code == 5);
    ck_assert(sbp_code_bias->biases[4].value == -566);
    ck_assert(sbp_code_bias->biases[4].code == 8);
    ck_assert(sbp_code_bias->biases[5].value == -563);
    ck_assert(sbp_code_bias->biases[5].code == 7);
    ck_assert(sbp_code_bias->biases[6].value == -607);
    ck_assert(sbp_code_bias->biases[6].code == 11);
    ck_assert(sbp_code_bias->biases[7].value == -567);
    ck_assert(sbp_code_bias->biases[7].code == 9);
    ck_assert(sbp_code_bias->biases[8].value == -315);
    ck_assert(sbp_code_bias->biases[8].code == 15);
    ck_assert(sbp_code_bias->biases[9].value == -306);
    ck_assert(sbp_code_bias->biases[9].code == 16);
  }
}

void sbp_callback_gps_phase_bias(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_PHASE_BIASES && !msg_checked) {
    msg_ssr_phase_biases_t *sbp_phase_bias = (msg_ssr_phase_biases_t *)buffer;
    if (sbp_phase_bias->sid.code !=
        constellation_to_l1_code(CONSTELLATION_GPS)) {
      // This is not a GPS message, wait for first GPS message
      return;
    }
    msg_checked = true;
    gps_phase_bias_processed = true;

    ck_assert(sbp_phase_bias->time.wn == 2013);
    ck_assert(sbp_phase_bias->time.tow == 171680);
    ck_assert(sbp_phase_bias->sid.sat == 1);
    ck_assert(sbp_phase_bias->sid.code == 0);
    ck_assert(sbp_phase_bias->update_interval == 2);
    ck_assert(sbp_phase_bias->iod_ssr == 0);
    ck_assert(sbp_phase_bias->dispersive_bias == 0);
    ck_assert(sbp_phase_bias->mw_consistency == 1);
    ck_assert(sbp_phase_bias->yaw == 41);
    ck_assert(sbp_phase_bias->yaw_rate == 0);

    ck_assert(sbp_phase_bias->biases[0].discontinuity_counter == 14);
    ck_assert(sbp_phase_bias->biases[0].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[0].bias == 6159);
    ck_assert(sbp_phase_bias->biases[0].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[0].code == 0);

    ck_assert(sbp_phase_bias->biases[1].discontinuity_counter == 14);
    ck_assert(sbp_phase_bias->biases[1].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[1].bias == 8922);
    ck_assert(sbp_phase_bias->biases[1].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[1].code == 11);

    ck_assert(sbp_phase_bias->biases[2].discontinuity_counter == 14);
    ck_assert(sbp_phase_bias->biases[2].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[2].bias == 9542);
    ck_assert(sbp_phase_bias->biases[2].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[2].code == 14);
  }
}

void sbp_callback_glo_orbit_clock(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_ORBIT_CLOCK && !msg_checked) {
    msg_ssr_orbit_clock_t *sbp_orbit_clock = (msg_ssr_orbit_clock_t *)buffer;
    if (sbp_orbit_clock->sid.code !=
        constellation_to_l1_code(CONSTELLATION_GLO)) {
      // This is not a GLO message, wait for first GLO message
      return;
    }
    msg_checked = true;
    glo_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 171680);
    ck_assert(sbp_orbit_clock->sid.sat == 1);
    ck_assert(sbp_orbit_clock->sid.code == 3);
    ck_assert(sbp_orbit_clock->update_interval == 2);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 11);
    ck_assert(sbp_orbit_clock->radial == 2170);
    ck_assert(sbp_orbit_clock->along == 1621);
    ck_assert(sbp_orbit_clock->cross == 1805);
    ck_assert(sbp_orbit_clock->dot_radial == -831);
    ck_assert(sbp_orbit_clock->dot_along == 96);
    ck_assert(sbp_orbit_clock->dot_cross == -19);
    ck_assert(sbp_orbit_clock->c0 == 20868);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_glo_code_bias(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_CODE_BIASES && !msg_checked) {
    msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t *)buffer;
    if (sbp_code_bias->sid.code !=
        constellation_to_l1_code(CONSTELLATION_GLO)) {
      // This is not a GLO message, wait for first GLO message
      return;
    }
    msg_checked = true;
    glo_code_bias_processed = true;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 1);
    ck_assert(sbp_code_bias->sid.code == 3);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);

    ck_assert(sbp_code_bias->biases[0].value == -260);
    ck_assert(sbp_code_bias->biases[0].code == 0);
    ck_assert(sbp_code_bias->biases[1].value == -257);
    ck_assert(sbp_code_bias->biases[1].code == 1);
    ck_assert(sbp_code_bias->biases[2].value == -350);
    ck_assert(sbp_code_bias->biases[2].code == 2);
    ck_assert(sbp_code_bias->biases[3].value == -424);
    ck_assert(sbp_code_bias->biases[3].code == 3);
  }
}

void sbp_callback_gal_orbit_clock(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_ORBIT_CLOCK && !msg_checked) {
    msg_ssr_orbit_clock_t *sbp_orbit_clock = (msg_ssr_orbit_clock_t *)buffer;
    if (sbp_orbit_clock->sid.code !=
        constellation_to_l1_code(CONSTELLATION_GAL)) {
      // This is not a GAL message, wait for first GAL message
      return;
    }
    msg_checked = true;
    gal_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 171680);
    ck_assert(sbp_orbit_clock->sid.sat == 1);
    ck_assert(sbp_orbit_clock->sid.code == CODE_GAL_E1B);
    ck_assert(sbp_orbit_clock->update_interval == 2);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 6);
    ck_assert(sbp_orbit_clock->radial == -1049048);
    ck_assert(sbp_orbit_clock->along == 262379);
    ck_assert(sbp_orbit_clock->cross == -262047);
    ck_assert(sbp_orbit_clock->dot_radial == -524290);
    ck_assert(sbp_orbit_clock->dot_along == -131078);
    ck_assert(sbp_orbit_clock->dot_cross == -262143);
    ck_assert(sbp_orbit_clock->c0 == 178);
    ck_assert(sbp_orbit_clock->c1 == -1048576);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_gal_code_bias(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_CODE_BIASES && !msg_checked) {
    msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t *)buffer;
    if (sbp_code_bias->sid.code !=
        constellation_to_l1_code(CONSTELLATION_GAL)) {
      // This is not a GAL message, wait for first GAL message
      return;
    }
    msg_checked = true;
    gal_code_bias_processed = true;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 1);
    ck_assert(sbp_code_bias->sid.code == CODE_GAL_E1B);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);

    ck_assert(sbp_code_bias->biases[0].value == 143);
    ck_assert(sbp_code_bias->biases[0].code == 3);
    ck_assert(sbp_code_bias->biases[1].value == 257);
    ck_assert(sbp_code_bias->biases[1].code == 7);
    ck_assert(sbp_code_bias->biases[2].value == 265);
    ck_assert(sbp_code_bias->biases[2].code == 10);
    ck_assert(sbp_code_bias->biases[3].value == 262);
    ck_assert(sbp_code_bias->biases[3].code == 13);
  }
}

void sbp_callback_gal_phase_bias(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_PHASE_BIASES && !msg_checked) {
    msg_ssr_phase_biases_t *sbp_phase_bias = (msg_ssr_phase_biases_t *)buffer;
    if (sbp_phase_bias->sid.code !=
        constellation_to_l1_code(CONSTELLATION_GAL)) {
      // This is not a GAL message, wait for first GAL message
      return;
    }
    msg_checked = true;
    gal_phase_bias_processed = true;

    ck_assert(sbp_phase_bias->time.wn == 2013);
    ck_assert(sbp_phase_bias->time.tow == 171680);
    ck_assert(sbp_phase_bias->sid.sat == 1);
    ck_assert(sbp_phase_bias->sid.code == 14);
    ck_assert(sbp_phase_bias->update_interval == 2);
    ck_assert(sbp_phase_bias->iod_ssr == 0);
    ck_assert(sbp_phase_bias->dispersive_bias == 0);
    ck_assert(sbp_phase_bias->mw_consistency == 1);
    ck_assert(sbp_phase_bias->yaw == 461);
    ck_assert(sbp_phase_bias->yaw_rate == 0);

    ck_assert(sbp_phase_bias->biases[0].discontinuity_counter == 10);
    ck_assert(sbp_phase_bias->biases[0].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[0].bias == -8261);
    ck_assert(sbp_phase_bias->biases[0].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[0].code == 3);

    ck_assert(sbp_phase_bias->biases[1].discontinuity_counter == 10);
    ck_assert(sbp_phase_bias->biases[1].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[1].bias == -10853);
    ck_assert(sbp_phase_bias->biases[1].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[1].code == 7);

    ck_assert(sbp_phase_bias->biases[2].discontinuity_counter == 10);
    ck_assert(sbp_phase_bias->biases[2].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[2].bias == -10494);
    ck_assert(sbp_phase_bias->biases[2].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[2].code == 10);
  }
}

void sbp_callback_bds_orbit_clock(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_ORBIT_CLOCK && !msg_checked) {
    msg_ssr_orbit_clock_t *sbp_orbit_clock = (msg_ssr_orbit_clock_t *)buffer;
    if (sbp_orbit_clock->sid.code !=
        constellation_to_l1_code(CONSTELLATION_BDS2)) {
      // This is not a BDS message, wait for first BDS message
      return;
    }
    msg_checked = true;
    bds_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 171680);
    ck_assert(sbp_orbit_clock->sid.sat == 5);
    ck_assert(sbp_orbit_clock->sid.code == CODE_BDS2_B1);
    ck_assert(sbp_orbit_clock->update_interval == 2);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 0);
    ck_assert(sbp_orbit_clock->radial == 821817);
    ck_assert(sbp_orbit_clock->along == 96);
    ck_assert(sbp_orbit_clock->cross == -196802);
    ck_assert(sbp_orbit_clock->dot_radial == 230505);
    ck_assert(sbp_orbit_clock->dot_along == 212990);
    ck_assert(sbp_orbit_clock->dot_cross == -155648);
    ck_assert(sbp_orbit_clock->c0 == 98304);
    ck_assert(sbp_orbit_clock->c1 == -524782);
    ck_assert(sbp_orbit_clock->c2 == 33554432);
  }
}

void sbp_callback_bds_code_bias(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_CODE_BIASES && !msg_checked) {
    msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t *)buffer;
    if (sbp_code_bias->sid.code !=
        constellation_to_l1_code(CONSTELLATION_BDS2)) {
      // This is not a BDS message, wait for first BDS message
      return;
    }
    msg_checked = true;
    bds_code_bias_processed = true;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 5);
    ck_assert(sbp_code_bias->sid.code == CODE_BDS2_B1);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);

    ck_assert(sbp_code_bias->biases[0].value == 32);
    ck_assert(sbp_code_bias->biases[0].code == 0);
    ck_assert(sbp_code_bias->biases[1].value == -103);
    ck_assert(sbp_code_bias->biases[1].code == 3);
    ck_assert(sbp_code_bias->biases[2].value == 53);
    ck_assert(sbp_code_bias->biases[2].code == 6);
  }
}

void sbp_callback_bds_phase_bias(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_PHASE_BIASES && !msg_checked) {
    msg_ssr_phase_biases_t *sbp_phase_bias = (msg_ssr_phase_biases_t *)buffer;
    if (sbp_phase_bias->sid.code !=
        constellation_to_l1_code(CONSTELLATION_BDS2)) {
      // This is not a GAL message, wait for first GAL message
      return;
    }
    msg_checked = true;
    bds_phase_bias_processed = true;

    ck_assert(sbp_phase_bias->time.wn == 2013);
    ck_assert(sbp_phase_bias->time.tow == 171675);
    ck_assert(sbp_phase_bias->sid.sat == 5);
    ck_assert(sbp_phase_bias->sid.code == CODE_BDS2_B1);
    ck_assert(sbp_phase_bias->update_interval == 2);
    ck_assert(sbp_phase_bias->iod_ssr == 0);
    ck_assert(sbp_phase_bias->dispersive_bias == 0);
    ck_assert(sbp_phase_bias->mw_consistency == 1);
    ck_assert(sbp_phase_bias->yaw == 0);
    ck_assert(sbp_phase_bias->yaw_rate == 0);

    ck_assert(sbp_phase_bias->biases[0].discontinuity_counter == 0);
    ck_assert(sbp_phase_bias->biases[0].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[0].bias == 1382);
    ck_assert(sbp_phase_bias->biases[0].integer_indicator == 0);
    ck_assert(sbp_phase_bias->biases[0].code == 0);

    ck_assert(sbp_phase_bias->biases[1].discontinuity_counter == 0);
    ck_assert(sbp_phase_bias->biases[1].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[1].bias == 2311);
    ck_assert(sbp_phase_bias->biases[1].integer_indicator == 0);
    ck_assert(sbp_phase_bias->biases[1].code == 6);

    ck_assert(sbp_phase_bias->biases[2].discontinuity_counter == 0);
    ck_assert(sbp_phase_bias->biases[2].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[2].bias == 1588);
    ck_assert(sbp_phase_bias->biases[2].integer_indicator == 0);
    ck_assert(sbp_phase_bias->biases[2].code == 3);
  }
}

START_TEST(test_ssr_gps_orbit_clock) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gps_orbit_clock,
             current_time);
  ck_assert(gps_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_gps_code_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gps_code_bias,
             current_time);
  ck_assert(gps_code_bias_processed);
}
END_TEST

START_TEST(test_ssr_gps_phase_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gps_phase_bias,
             current_time);
  ck_assert(gps_phase_bias_processed);
}
END_TEST

START_TEST(test_ssr_glo_orbit_clock) {
  current_time.wn = 2013;
  current_time.tow = 171680;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_glo_orbit_clock,
             current_time);
  ck_assert(glo_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_glo_code_bias) {
  current_time.wn = 2013;
  current_time.tow = 171680;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_glo_code_bias,
             current_time);
  ck_assert(glo_code_bias_processed);
}
END_TEST

START_TEST(test_ssr_gal_orbit_clock) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gal_orbit_clock,
             current_time);
  ck_assert(gal_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_gal_code_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gal_code_bias,
             current_time);
  ck_assert(gal_code_bias_processed);
}
END_TEST

START_TEST(test_ssr_gal_phase_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gal_phase_bias,
             current_time);
  ck_assert(gal_phase_bias_processed);
}
END_TEST

START_TEST(test_ssr_bds_orbit_clock) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_bds_orbit_clock,
             current_time);
  ck_assert(bds_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_bds_code_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_bds_code_bias,
             current_time);
  ck_assert(bds_code_bias_processed);
}
END_TEST

START_TEST(test_ssr_bds_phase_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_bds_phase_bias,
             current_time);
  ck_assert(bds_phase_bias_processed);
}
END_TEST

Suite *rtcm3_ssr_suite(void) {
  Suite *s = suite_create("RTCMv3_ssr");

  TCase *tc_ssr = tcase_create("SSR");
  tcase_add_checked_fixture(tc_ssr, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_ssr, test_ssr_gps_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_gps_code_bias);
  tcase_add_test(tc_ssr, test_ssr_gps_phase_bias);
  tcase_add_test(tc_ssr, test_ssr_glo_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_glo_code_bias);
  tcase_add_test(tc_ssr, test_ssr_gal_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_gal_code_bias);
  tcase_add_test(tc_ssr, test_ssr_gal_phase_bias);
  tcase_add_test(tc_ssr, test_ssr_bds_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_bds_code_bias);
  tcase_add_test(tc_ssr, test_ssr_bds_phase_bias);
  suite_add_tcase(s, tc_ssr);

  return s;
}
