/*
 * Copyright (C) 2019,2020 Swift Navigation Inc.
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

#include <swiftnav/nav_meas.h>
#include <swiftnav/signal.h>

#include <libsbp/edc.h>
#include <libsbp/imu.h>
#include <libsbp/orientation.h>
#include <libsbp/vehicle.h>

#include <gnss-converters/ubx_sbp.h>

#include "check_suites.h"
#include "check_ubx.h"
#include "config.h"

FILE *fp;

static const uint16_t hnr_pvt_crc[] = {57519};
static void ubx_sbp_callback_hnr_pvt(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  /* This test depends on nav_pvt working correctly, since the first message is
   * a nav_pvt message */
  ck_assert(msg_id == SBP_MSG_POS_LLH);
  ck_assert(length == sizeof(msg_pos_llh_t));

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == hnr_pvt_crc[msg_index]);

  msg_index++;
}

static const uint16_t hnr_pvt_disabled_crc[] = {41409};
static void ubx_sbp_callback_hnr_pvt_disabled(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  /* This test depends on nav_pvt working correctly, since the first message is
   * a nav_pvt message */
  ck_assert(msg_id == SBP_MSG_POS_LLH);
  ck_assert(length == sizeof(msg_pos_llh_t));

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == hnr_pvt_disabled_crc[msg_index]);

  msg_index++;
}

static const uint16_t rxm_rawx_crc[] = {14708, 41438, 2564, 52301, 10854};
static void ubx_sbp_callback_rxm_rawx(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  ck_assert(msg_id == SBP_MSG_OBS);

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == rxm_rawx_crc[msg_index]);
  msg_index++;
}

static const u16 rxm_sfrbx_crc = 0xC040;
static void ubx_sbp_callback_rxm_sfrbx(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;

  ck_assert(msg_id == SBP_MSG_EPHEMERIS_GPS);

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == rxm_sfrbx_crc);
}

static void ubx_sbp_callback_nav_status(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  (void)sender_id;
  (void)buff;
  (void)length;
  (void)msg_id;
  /* If this gets called, fail the test. We should not generate an SBP message
   * from NAV-STATUS */
  ck_assert_msg(false, "UBX-NAV-STATUS should not generate a SBP message");
}

static const u16 esf_meas_crc[] = {
    0xDA8D, 0x6238, 0x7009, 0xF101, 0xE330, 0x6238, 0xF101};
static void ubx_sbp_callback_esf_meas(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;

  static int msg_index = 0;
  const u8 vel_sources[] = {0, 0, 2, 3, 1, 0, 3};

  ck_assert(msg_id == SBP_MSG_ODOMETRY);
  msg_odometry_t *msg = (msg_odometry_t *)buff;
  const u8 vel_source_mask = 0b11000;
  const u8 time_source_mask = 0x3;
  const u8 velocity_source = (msg->flags & vel_source_mask) >> 3;
  const u8 time_source = (msg->flags & time_source_mask);

  /* The first message in the .ubx file won't have a valid time because no
   * UBX-NAV-STATUS has been received to calculate the offset between
   * milliseconds since startup and GNSS time of week */
  if (msg_index == 0) {
    ck_assert_int_eq(msg->tow, 25269459);
    ck_assert_int_eq(time_source, 0);
  } else { /* all following messages should have a valid GPS timestamp */
    ck_assert_int_eq(msg->tow, 349740106);
    ck_assert_int_eq(time_source, 1);
  }

  /* Check that the velocity source is correctly set - RR = 0, RL = 1, FL = 2,
   * FR = 3, single tick = 0, speed = 3 */
  ck_assert(velocity_source == vel_sources[msg_index]);
  ck_assert_int_eq(msg->velocity, 25677);

  /* Check the CRCs */
  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert_int_eq(crc, esf_meas_crc[msg_index]);
  msg_index++;
}

static const u16 esf_raw_crc[] = {0x598C,
                                  0x4A06,
                                  0x2DDF,
                                  0x13BB,
                                  0xE40B,
                                  0x51A1,
                                  0x2C19,
                                  0xEAC8,
                                  0x8D0A,
                                  0x8560,
                                  0x8EC4};
static void ubx_sbp_callback_esf_raw(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  /* Check that the first message we receive contains the IMU configuration with
     the expected IMU ranges */
  if (msg_index == 0) {
    ck_assert(msg_id == SBP_MSG_IMU_AUX);
    msg_imu_aux_t *msg = (msg_imu_aux_t *)buff;
    ck_assert(msg->imu_type == 0);
    const u8 gyro_mask = 0xF0;
    const u8 acc_mask = 0x0F;
    const u8 gyro_mode = (msg->imu_conf & gyro_mask) >> 4;
    const u8 acc_mode = msg->imu_conf & acc_mask;
    /* Check that gyro range is set to 125 deg/s */
    ck_assert(gyro_mode == 4);
    /* Check that accelerometer range is set to 4 g */
    ck_assert(acc_mode == 1);
  } else {
    ck_assert(msg_id == SBP_MSG_IMU_RAW);
    msg_imu_raw_t *msg = (msg_imu_raw_t *)buff;
    ck_assert_int_ge(msg->tow, 325305109);
    ck_assert_int_le(msg->tow, 325305198);
  }

  /* Check that we won't receive more than 11 messages from this test file */
  ck_assert_int_lt(msg_index, 11);
  /* Check the CRCs */
  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == esf_raw_crc[msg_index]);
  msg_index++;
}

static const uint16_t nav_att_crc[] = {63972, 57794};
static void ubx_sbp_callback_nav_att(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  /* This test depends on nav_pvt working correctly, since the first message is
   * a nav_pvt message */
  if (msg_index == 0) {
    ck_assert(msg_id == SBP_MSG_POS_LLH);
    ck_assert(length == sizeof(msg_pos_llh_t));
  } else {
    ck_assert(msg_id == SBP_MSG_ORIENT_EULER);
    ck_assert(length == sizeof(msg_orient_euler_t));
  }

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == nav_att_crc[msg_index]);
  msg_index++;
}

static const uint16_t nav_pvt_crc = 23845;
static void ubx_sbp_callback_nav_pvt(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  ck_assert(msg_id == SBP_MSG_POS_LLH);
  ck_assert(length == sizeof(msg_pos_llh_t));

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == nav_pvt_crc);
}

static const uint16_t nav_pvt_corrupted_crc = 23845;
static void ubx_sbp_callback_nav_pvt_corrupted(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  /* message 2 has a malformed length field
   * message 3 is incomplete
   * Both should be dropped and not passed to this callback
   */
  ck_assert(msg_index == 0);

  ck_assert(msg_id == SBP_MSG_POS_LLH);
  ck_assert(length == sizeof(msg_pos_llh_t));

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == nav_pvt_corrupted_crc);

  msg_index++;
}

static const uint16_t nav_pvt_fix_type_crc[] = {32103,
                                                44234,
                                                23845,
                                                23845,
                                                56365,
                                                32103,
                                                44234,
                                                19716,
                                                19716,
                                                52236,
                                                32103,
                                                44234,
                                                32103,
                                                32103,
                                                64623,
                                                27974};
static void ubx_sbp_callback_nav_pvt_fix_type(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  /* Some combinations of fix_type and flags do not make sense (i.e. no_fix and
   * gnssFix true simultaneously). If we add semantic sanity checks to ubx_sbp
   * in the future, this test will need to be updated.
   */
  ck_assert(msg_id == SBP_MSG_POS_LLH);
  ck_assert(length == sizeof(msg_pos_llh_t));

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == nav_pvt_fix_type_crc[msg_index]);

  msg_index++;
}

static const uint16_t nav_pvt_set_sender_id_crc[] = {19827};
static void ubx_sbp_callback_nav_pvt_set_sender_id(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  ck_assert(msg_id == SBP_MSG_POS_LLH);
  ck_assert(length == sizeof(msg_pos_llh_t));

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == nav_pvt_set_sender_id_crc[msg_index]);

  msg_index++;
}

static const uint16_t nav_vel_ecef_crc[] = {57757, 36858};
static void ubx_sbp_callback_nav_vel_ecef(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static int msg_index = 0;

  /* This test depends on nav_pvt working correctly, since the first message is
   * a nav_pvt message */
  if (msg_index == 0) {
    ck_assert(msg_id == SBP_MSG_POS_LLH);
    ck_assert(length == sizeof(msg_pos_llh_t));
  } else if (msg_index == 1) {
    ck_assert(msg_id == SBP_MSG_VEL_ECEF);
    ck_assert(length == sizeof(msg_vel_ecef_t));
  } else {
    ck_assert(false && "unexpected message");
  }

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == nav_vel_ecef_crc[msg_index]);

  msg_index++;
}

int read_file_check_ubx(uint8_t *buf, size_t len, void *ctx) {
  (void)ctx;
  return fread(buf, sizeof(uint8_t), len, fp);
}

void test_UBX(struct ubx_sbp_state state, const char *filename) {
  fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  int ret;
  do {
    ret = ubx_sbp_process(&state, &read_file_check_ubx);
  } while (ret > 0);
}

START_TEST(test_hnr_pvt) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_hnr_pvt, NULL);

  ubx_set_hnr_flag(&state, true);
  test_UBX(state, RELATIVE_PATH_PREFIX "/data/hnr_pvt.ubx");
}
END_TEST

START_TEST(test_hnr_pvt_disabled) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_hnr_pvt_disabled, NULL);

  ubx_set_hnr_flag(&state, false);
  test_UBX(state, RELATIVE_PATH_PREFIX "/data/hnr_pvt.ubx");
}
END_TEST

START_TEST(test_nav_att) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_att, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/nav_att.ubx");
}
END_TEST

START_TEST(test_nav_pvt) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_pvt, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/nav_pvt.ubx");
}
END_TEST

START_TEST(test_nav_pvt_corrupted) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_pvt_corrupted, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/nav_pvt_corrupted.ubx");
}
END_TEST

START_TEST(test_nav_pvt_fix_type) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_pvt_fix_type, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/nav_pvt_fix_type.ubx");
}
END_TEST

START_TEST(test_nav_pvt_set_sender_id) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_pvt_set_sender_id, NULL);

  ubx_set_sender_id(&state, 12345);
  test_UBX(state, RELATIVE_PATH_PREFIX "/data/nav_pvt.ubx");
}
END_TEST

START_TEST(test_nav_vel_ecef) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_vel_ecef, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/nav_velecef.ubx");
}
END_TEST

START_TEST(test_rxm_rawx) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_rxm_rawx, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/rxm_rawx.ubx");
}
END_TEST

START_TEST(test_rxm_sfrbx) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_rxm_sfrbx, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/rxm_gps_sfrbx.ubx");
}
END_TEST

START_TEST(test_esf_meas) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_esf_meas, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/esf_meas.ubx");
}
END_TEST

START_TEST(test_nav_status) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_status, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/nav_status.ubx");
}
END_TEST

START_TEST(test_esf_raw) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_esf_raw, NULL);

  test_UBX(state, RELATIVE_PATH_PREFIX "/data/esf_raw.ubx");
}
END_TEST

START_TEST(test_convert_msss_to_tow) {
  struct ubx_esf_state state = {.tow_offset_set = false,
                                .time_since_startup_tow_offset = 0.,
                                .last_sync_msss = 0};

  /* Check that the function returns a number less than 0 if no time of week is
     known */
  ck_assert_double_lt(ubx_convert_msss_to_tow(1234, &state), 0.);

  /* Check that the conversion will work under nominal conditions */
  state.tow_offset_set = true;
  state.last_sync_msss = 1234;
  state.time_since_startup_tow_offset = 24 * 3600;
  u32 msss = 2000;
  double expected_tow = 24 * 3600 + 0.001 * msss;
  double tol = 1e-9;
  ck_assert_double_eq_tol(
      ubx_convert_msss_to_tow(msss, &state), expected_tow, tol);

  /* Check that the conversion will work if messages are only slightly out of
     order, i.e. msss is a bit smaller than the last sync mssss */
  state.tow_offset_set = true;
  state.last_sync_msss = 1234;
  state.time_since_startup_tow_offset = 24 * 3600;
  msss = state.last_sync_msss - 1;
  expected_tow = 24 * 3600 + 0.001 * msss;
  ck_assert_double_eq_tol(
      ubx_convert_msss_to_tow(msss, &state), expected_tow, tol);

  /* Same, but crossing msss overflow */
  state.tow_offset_set = true;
  state.last_sync_msss = 1;
  state.time_since_startup_tow_offset = 24 * 3600;
  msss = state.last_sync_msss - 10;
  expected_tow = 24 * 3600 + 0.001 * msss;
  ck_assert_double_eq_tol(
      ubx_convert_msss_to_tow(msss, &state), expected_tow, tol);

  /* Check that the conversion will work if msss has a 32bit overflow between
     last sync and current time */
  state.tow_offset_set = true;
  state.last_sync_msss = 0xFFFFFFFF - 100;
  state.time_since_startup_tow_offset = 1234.0 - 0.001 * state.last_sync_msss;
  msss = state.last_sync_msss + 500;
  expected_tow = 1234.5;
  ck_assert_double_eq_tol(
      ubx_convert_msss_to_tow(msss, &state), expected_tow, tol);
}
END_TEST

Suite *ubx_suite(void) {
  Suite *s = suite_create("UBX");

  TCase *tc_hnr = tcase_create("UBX_HNR");
  tcase_add_test(tc_hnr, test_hnr_pvt);
  tcase_add_test(tc_hnr, test_hnr_pvt_disabled);
  suite_add_tcase(s, tc_hnr);

  TCase *tc_nav = tcase_create("UBX_NAV");
  tcase_add_test(tc_nav, test_nav_att);
  tcase_add_test(tc_nav, test_nav_pvt);
  tcase_add_test(tc_nav, test_nav_pvt_corrupted);
  tcase_add_test(tc_nav, test_nav_pvt_fix_type);
  tcase_add_test(tc_nav, test_nav_pvt_set_sender_id);
  tcase_add_test(tc_nav, test_nav_vel_ecef);
  tcase_add_test(tc_nav, test_nav_status);
  suite_add_tcase(s, tc_nav);

  TCase *tc_esf = tcase_create("UBX_ESF");
  tcase_add_test(tc_esf, test_convert_msss_to_tow);
  tcase_add_test(tc_esf, test_esf_meas);
  tcase_add_test(tc_esf, test_esf_raw);
  suite_add_tcase(s, tc_esf);

  TCase *tc_rxm = tcase_create("UBX_RXM");
  tcase_add_test(tc_rxm, test_rxm_rawx);
  tcase_add_test(tc_rxm, test_rxm_sfrbx);
  suite_add_tcase(s, tc_rxm);

  return s;
}
