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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <check.h>

#include "../src/rtcm3_sbp_internal.h"
#include <config.h>

#include "check_suites.h"

/* rtcm helper defines and functions */

#define MAX_FILE_SIZE 26000

static double expected_L1CA_bias = 0.0;
static double expected_L1P_bias = 0.0;
static double expected_L2CA_bias = 0.0;
static double expected_L2P_bias = 0.0;

void sbp_callback_gps(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  static uint32_t msg_count = 0;
  if (msg_count == 3 || msg_count == 20 || msg_count == 42) {
    ck_assert_uint_eq(msg_id, SBP_MSG_BASE_POS_ECEF);
  } else if (msg_count == 4) {
    ck_assert_uint_eq(msg_id, SBP_MSG_GLO_BIASES);
  } else {
    ck_assert_uint_eq(msg_id, SBP_MSG_OBS);
  }
  msg_count++;
}

void sbp_callback_1012_first(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *msg = (msg_obs_t *) buffer;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    ck_assert_uint_gt(num_sbp_msgs, 2);
  }
}

void sbp_callback_glo_day_rollover(u16 msg_id, u8 length, u8 *buffer,
                                   u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *msg = (msg_obs_t *)buffer;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    ck_assert(num_sbp_msgs > 2);
  }
}

void check_biases(msg_glo_biases_t *sbp_glo_msg) {
  if (sbp_glo_msg->mask & 0x01) {
    ck_assert(sbp_glo_msg->l1ca_bias / GLO_BIAS_RESOLUTION == expected_L1CA_bias);
  }
  if (sbp_glo_msg->mask & 0x02) {
    ck_assert(sbp_glo_msg->l1p_bias / GLO_BIAS_RESOLUTION == expected_L1P_bias);
  }
  if (sbp_glo_msg->mask & 0x04) {
    ck_assert(sbp_glo_msg->l2ca_bias / GLO_BIAS_RESOLUTION == expected_L2CA_bias);
  }
  if (sbp_glo_msg->mask & 0x08) {
    ck_assert(sbp_glo_msg->l2p_bias / GLO_BIAS_RESOLUTION == expected_L2P_bias);
  }
}

void sbp_callback_bias(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg);
  }
}

void sbp_callback_msm(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_LOG) {
    msg_log_t *sbp_log = (msg_log_t *)buffer;
    ck_assert_uint_eq(sbp_log->level, RTCM_MSM_LOGGING_LEVEL);
  }
}

void test_RTCM3(const char* filename,
                void (*cb_rtcm_to_sbp)(u16 msg_id, u8 length, u8 *buffer, u16 sender_id),
                gps_time_sec_t current_time) {
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, cb_rtcm_to_sbp, NULL);
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen(filename, "rb");
  u8 buffer[MAX_FILE_SIZE];
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n",filename);
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    buffer_index++;
  }

  return;
}

void set_expected_bias(double L1CA_bias, double L1P_bias, double L2CA_bias, double L2P_bias){
  expected_L1CA_bias = L1CA_bias;
  expected_L1P_bias = L1P_bias;
  expected_L2CA_bias = L2CA_bias;
  expected_L2P_bias = L2P_bias;
}

/* end rtcm helpers */

/* fixture globals and functions */
gps_time_sec_t current_time;

void rtcm3_setup_basic(void)
{
  memset(&current_time, 0, sizeof(current_time));
  current_time.wn = 1945;
  current_time.tow = 211190;
}

/* end fixtures */

START_TEST(test_gps_time)
{
  current_time.wn = 1945;
  current_time.tow = 277500;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/RTCM3.bin", sbp_callback_gps, current_time);
}
END_TEST

START_TEST(test_glo_day_rollover)
{
  current_time.wn = 1959;
  current_time.tow = 510191;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/glo_day_rollover.rtcm", sbp_callback_glo_day_rollover, current_time);
}
END_TEST

START_TEST(test_1012_first)
{
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/1012_first.rtcm", sbp_callback_1012_first, current_time);
}
END_TEST

// Test MSM is properly rejected
START_TEST(test_msm_reject)
{
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/msm.rtcm",sbp_callback_msm,current_time);
}
END_TEST

// Test 1033 message sources
START_TEST(test_bias_trm)
{
  set_expected_bias(TRIMBLE_BIAS_M,TRIMBLE_BIAS_M,TRIMBLE_BIAS_M,TRIMBLE_BIAS_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/trimble.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_jav)
{
  set_expected_bias(JAVAD_BIAS_L1CA_M,0.0,0.0,JAVAD_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/javad.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_nov)
{
  set_expected_bias(NOVATEL_BIAS_M,NOVATEL_BIAS_M,NOVATEL_BIAS_M,NOVATEL_BIAS_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/leica.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_sep)
{
  set_expected_bias(SEPTENTRIO_BIAS_M,SEPTENTRIO_BIAS_M,SEPTENTRIO_BIAS_M,SEPTENTRIO_BIAS_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/sept.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_nav)
{
  set_expected_bias(NAVCOM_BIAS_L1CA_M,0.0,0.0,NAVCOM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/navcom.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_top)
{
  set_expected_bias(TOPCON_BIAS_M,TOPCON_BIAS_M,TOPCON_BIAS_M,TOPCON_BIAS_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/topcon.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_hem)
{
  set_expected_bias(HEMISPHERE_BIAS_L1CA_M,0.0,0.0,HEMISPHERE_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/hemi.rtcm",sbp_callback_bias,current_time);
}
END_TEST

  // Test 1033 messages from GEO++
START_TEST(test_bias_gpp_ash1)
{
  set_expected_bias(GPP_ASH1_BIAS_L1CA_M,0.0,0.0,GPP_ASH1_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_ASH1.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_hem)
{
  set_expected_bias(GPP_HEM_BIAS_L1CA_M,0.0,0.0,GPP_HEM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_HEM.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_jav)
{
  set_expected_bias(GPP_JAV_BIAS_L1CA_M,0.0,0.0,GPP_JAV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_JAV.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_jps)
{
  set_expected_bias(GPP_JPS_BIAS_L1CA_M,0.0,0.0,GPP_JPS_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_JPS.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_lei)
{
  set_expected_bias(GPP_NOV_BIAS_L1CA_M,0.0,0.0,GPP_NOV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_LEI.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_nav)
{
  set_expected_bias(GPP_NAV_BIAS_L1CA_M,0.0,0.0,GPP_NAV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NAV.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_nov)
{
  set_expected_bias(GPP_NOV_BIAS_L1CA_M,0.0,0.0,GPP_NOV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NOV.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_nvr)
{
  set_expected_bias(GPP_NVR_BIAS_L1CA_M,0.0,0.0,GPP_NVR_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NVR.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_sep1)
{
  set_expected_bias(GPP_SEP_BIAS_L1CA_M,0.0,0.0,GPP_SEP_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_SEP1.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_sok)
{
  set_expected_bias(GPP_SOK_BIAS_L1CA_M,0.0,0.0,GPP_SOK_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_SOK.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_tps1)
{
  set_expected_bias(GPP_TPS_BIAS_L1CA_M,0.0,0.0,GPP_TPS_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_TPS1.rtcm",sbp_callback_bias,current_time);
}
END_TEST

START_TEST(test_bias_gpp_trm)
{
  set_expected_bias(GPP_TRM_BIAS_L1CA_M,0.0,0.0,GPP_TRM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_TRM.rtcm",sbp_callback_bias,current_time);
}
END_TEST

Suite* rtcm3_suite(void)
{
  Suite *s = suite_create("RTCMv3");

  TCase *tc_core = tcase_create("Core");
  tcase_add_checked_fixture(tc_core, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_core, test_gps_time);
  tcase_add_test(tc_core, test_glo_day_rollover);
  tcase_add_test(tc_core, test_1012_first);
  tcase_add_test(tc_core, test_msm_reject);
  suite_add_tcase(s, tc_core);

  TCase *tc_biases = tcase_create("Biases");
  tcase_add_checked_fixture(tc_biases, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_biases, test_bias_trm);
  tcase_add_test(tc_biases, test_bias_jav);
  tcase_add_test(tc_biases, test_bias_nov);
  tcase_add_test(tc_biases, test_bias_sep);
  tcase_add_test(tc_biases, test_bias_nav);
  tcase_add_test(tc_biases, test_bias_top);
  tcase_add_test(tc_biases, test_bias_hem);
  suite_add_tcase(s, tc_biases);

  TCase *tc_gpp_biases = tcase_create("Geo++ Biases");
  tcase_add_checked_fixture(tc_gpp_biases, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_ash1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_hem);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_jav);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_jps);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_lei);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nav);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nov);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nvr);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_sep1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_sok);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_tps1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_trm);
  suite_add_tcase(s, tc_gpp_biases);

  return s;
}

