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

#include <assert.h>
#include <check.h>
#include <gnss-converters/sbp_nmea.h>
#include <libsbp/sbp.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/sbp_nmea_internal.h"
#include "check_suites.h"
#include "config.h"
#include "nmea_truth.h"

static sbp_msg_callbacks_node_t gps_time_callback_node;
static sbp_msg_callbacks_node_t utc_time_callback_node;
static sbp_msg_callbacks_node_t pos_llh_cov_callback_node;
static sbp_msg_callbacks_node_t vel_ned_callback_node;
static sbp_msg_callbacks_node_t dops_callback_node;
static sbp_msg_callbacks_node_t age_correction_callback_node;
static sbp_msg_callbacks_node_t baseline_heading_callback_node;
static sbp_msg_callbacks_node_t sv_az_el_callback_node;
static sbp_msg_callbacks_node_t measurement_state_callback_node;
static sbp_msg_callbacks_node_t observation_callback_node;

static bool nmea_gpgga_processed = false;
static bool nmea_gprmc_processed = false;
static bool nmea_gpvtg_processed = false;
static bool nmea_gpgll_processed = false;
static bool nmea_gpzda_processed = false;
/*static bool nmea_gphdt_processed = false;*/
static bool nmea_gsa_processed = false;
static bool nmea_gpgst_processed = false;
static bool nmea_gpgsv_processed = false;

static int msg_count[SBP2NMEA_NMEA_CNT];
static bool start_count[SBP2NMEA_NMEA_CNT];

int32_t read_file(uint8_t *buff, uint32_t n, void *context) {
  FILE *f = (FILE *)context;
  return (int32_t)(fread(buff, 1, n, f));
}

void nmea_callback_gpgga(u8 msg[]) {
  if (strstr((char *)msg, "GGA")) {
    if ((strstr((char *)msg, "193508.20") || start_count[SBP2NMEA_NMEA_GGA]) &&
        msg_count[SBP2NMEA_NMEA_GGA] < 10) {
      nmea_gpgga_processed = true;
      start_count[SBP2NMEA_NMEA_GGA] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpgga_truth[msg_count[SBP2NMEA_NMEA_GGA]++]) == 0);
    }
  }
}

void nmea_callback_gprmc(u8 msg[]) {
  if (strstr((char *)msg, "RMC")) {
    if ((strstr((char *)msg, "193502.00") || start_count[SBP2NMEA_NMEA_RMC]) &&
        msg_count[SBP2NMEA_NMEA_RMC] < 10) {
      nmea_gprmc_processed = true;
      start_count[SBP2NMEA_NMEA_RMC] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gprmc_truth[msg_count[SBP2NMEA_NMEA_RMC]++]) == 0);
    }
  }
}

void nmea_callback_gpvtg(u8 msg[]) {
  if (strstr((char *)msg, "VTG")) {
    if ((strstr((char *)msg, "$GPVTG,,T,,M,0.02,N,0.04,K,A*25") ||
         start_count[SBP2NMEA_NMEA_VTG]) &&
        msg_count[SBP2NMEA_NMEA_VTG] < 10) {
      nmea_gpvtg_processed = true;
      start_count[SBP2NMEA_NMEA_VTG] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpvtg_truth[msg_count[SBP2NMEA_NMEA_VTG]++]) == 0);
    }
  }
}

void nmea_callback_gpgll(u8 msg[]) {
  if (strstr((char *)msg, "GLL")) {
    if ((strstr((char *)msg, "193502.00") || start_count[SBP2NMEA_NMEA_GLL]) &&
        msg_count[SBP2NMEA_NMEA_GLL] < 10) {
      nmea_gpgll_processed = true;
      start_count[SBP2NMEA_NMEA_GLL] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpgll_truth[msg_count[SBP2NMEA_NMEA_GLL]++]) == 0);
    }
  }
}

void nmea_callback_gpzda(u8 msg[]) {
  if (strstr((char *)msg, "ZDA")) {
    if ((strstr((char *)msg, "193502.00") || start_count[SBP2NMEA_NMEA_ZDA]) &&
        msg_count[SBP2NMEA_NMEA_ZDA] < 10) {
      nmea_gpzda_processed = true;
      start_count[SBP2NMEA_NMEA_ZDA] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpzda_truth[msg_count[SBP2NMEA_NMEA_ZDA]++]) == 0);
    }
  }
}

/*void nmea_callback_gphdt(u8 msg[]) {
  if(strstr((char*)msg,"GPHDT")){
    static int msg_count = 0;
    static bool start_count = false;
    nmea_gphdt_processed = true;
    if((strstr((char *)msg,"171104.30") || start_count) && msg_count < 10) {
      start_count = true;
      msg [ strcspn((char *)msg, "\r\n") ] = '\0';
      ck_assert(strcmp((char *)msg,gpgga_truth[msg_count]) == 0);
      msg_count++;
    }
  }
} */

void nmea_callback_gsa(u8 msg[]) {
  if (strstr((char *)msg, "GSA")) {
    if ((strstr((char *)msg,
                "$GNGSA,A,3,414,421,426,429,435,436,,,,,,,1.8,0.9,1.6*21") ||
         start_count[SBP2NMEA_NMEA_GSA]) &&
        msg_count[SBP2NMEA_NMEA_GSA] < 10) {
      nmea_gsa_processed = true;
      start_count[SBP2NMEA_NMEA_GSA] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(
          strcmp((char *)msg, gsa_truth[msg_count[SBP2NMEA_NMEA_GSA]++]) == 0);
    }
  }
}

void nmea_callback_gpgst(u8 msg[]) {
  if (strstr((char *)msg, "GST")) {
    if ((strstr((char *)msg, "193501.80") || start_count[SBP2NMEA_NMEA_GST]) &&
        msg_count[SBP2NMEA_NMEA_GST] < 10) {
      nmea_gpgst_processed = true;
      start_count[SBP2NMEA_NMEA_GST] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(
          strcmp((char *)msg, gst_truth[msg_count[SBP2NMEA_NMEA_GST]++]) == 0);
    }
  }
}

void nmea_callback_gpgsv(u8 msg[]) {
  if (strstr((char *)msg, "GSV")) {
    if (msg_count[SBP2NMEA_NMEA_GSV] < 10) {
      nmea_gpgsv_processed = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(
          strcmp((char *)msg, gsv_truth[msg_count[SBP2NMEA_NMEA_GSV]++]) == 0);
    }
  }
}

void gps_time_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_GPS_TIME);
}

void utc_time_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_UTC_TIME);
}

void pos_llh_cov_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_POS_LLH_COV);
}

void vel_ned_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_VEL_NED);
}

void dops_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_DOPS);
}

void age_correction_callback(u16 sender_id,
                             u8 length,
                             u8 msg[],
                             void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_AGE_CORR);
}

void baseline_heading_callback(u16 sender_id,
                               u8 length,
                               u8 msg[],
                               void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_HDG);
}

void sv_az_el_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_SV_AZ_EL);
}

void measurement_state_callback(u16 sender_id,
                                u8 length,
                                u8 msg[],
                                void *context) {
  (void)sender_id;
  sbp2nmea(context, length, msg, SBP2NMEA_SBP_MEASUREMENT_STATE);
}

void observation_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)sender_id;
  msg_obs_t *sbp_obs = (msg_obs_t *)msg;
  uint8_t num_obs =
      (length - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);
  sbp2nmea_obs(context, sbp_obs, num_obs);
}

void sbp_init(sbp_state_t *sbp_state, void *ctx) {
  sbp_state_init(sbp_state);
  sbp_register_callback(sbp_state,
                        SBP_MSG_GPS_TIME,
                        &gps_time_callback,
                        ctx,
                        &gps_time_callback_node);
  sbp_register_callback(sbp_state,
                        SBP_MSG_UTC_TIME,
                        &utc_time_callback,
                        ctx,
                        &utc_time_callback_node);
  sbp_register_callback(sbp_state,
                        SBP_MSG_POS_LLH_COV,
                        &pos_llh_cov_callback,
                        ctx,
                        &pos_llh_cov_callback_node);
  sbp_register_callback(sbp_state,
                        SBP_MSG_VEL_NED,
                        &vel_ned_callback,
                        ctx,
                        &vel_ned_callback_node);
  sbp_register_callback(
      sbp_state, SBP_MSG_DOPS, &dops_callback, ctx, &dops_callback_node);
  sbp_register_callback(sbp_state,
                        SBP_MSG_AGE_CORRECTIONS,
                        &age_correction_callback,
                        ctx,
                        &age_correction_callback_node);
  sbp_register_callback(sbp_state,
                        SBP_MSG_BASELINE_HEADING,
                        &baseline_heading_callback,
                        ctx,
                        &baseline_heading_callback_node);
  sbp_register_callback(sbp_state,
                        SBP_MSG_SV_AZ_EL,
                        &sv_az_el_callback,
                        ctx,
                        &sv_az_el_callback_node);
  sbp_register_callback(sbp_state,
                        SBP_MSG_MEASUREMENT_STATE,
                        &measurement_state_callback,
                        ctx,
                        &measurement_state_callback_node);
  sbp_register_callback(sbp_state,
                        SBP_MSG_OBS,
                        &observation_callback,
                        ctx,
                        &observation_callback_node);
}

void test_NMEA(const char *filename, void (*cb_sbp_to_nmea)(u8 msg[])) {
  memset(&msg_count, 0, sizeof(msg_count));
  memset(&start_count, 0, sizeof(start_count));

  sbp2nmea_t state = {0};
  sbp2nmea_init(&state, cb_sbp_to_nmea);
  sbp2nmea_base_id_set(&state, 33);
  sbp2nmea_soln_freq_set(&state, 10);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GGA);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_RMC);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_VTG);
  /*sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_HDT);*/
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_GLL);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_ZDA);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GSA);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GST);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_GSV);

  sbp_state_t sbp_state_;
  sbp_init(&sbp_state_, &state);

  FILE *fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }
  sbp_state_set_io_context(&sbp_state_, fp);
  while (!feof(fp)) {
    sbp_process(&sbp_state_, &read_file);
  }
  fclose(fp);
  return;
}

void cb_sbp_to_nmea_ctx_passthrough(char *msg, void *context) {
  void (*cb)(u8[]) = context;
  cb((u8 *)msg);
}

void test_NMEA_ctx(const char *filename, void (*cb_sbp_to_nmea)(u8 msg[])) {
  memset(&msg_count, 0, sizeof(msg_count));
  memset(&start_count, 0, sizeof(start_count));

  sbp2nmea_t state = {0};
  sbp2nmea_ctx_init(&state, cb_sbp_to_nmea_ctx_passthrough, cb_sbp_to_nmea);
  sbp2nmea_base_id_set(&state, 33);
  sbp2nmea_soln_freq_set(&state, 10);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GGA);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_RMC);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_VTG);
  /*sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_HDT);*/
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_GLL);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_ZDA);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GSA);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GST);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_GSV);

  sbp_state_t sbp_state_;
  sbp_init(&sbp_state_, &state);

  FILE *fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }
  sbp_state_set_io_context(&sbp_state_, fp);
  while (!feof(fp)) {
    sbp_process(&sbp_state_, &read_file);
  }
  fclose(fp);
  return;
}

void nmea_setup_basic(void) { return; }

START_TEST(test_nmea_gpgga) {
  nmea_gpgga_processed = false;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgga);
  ck_assert(nmea_gpgga_processed);

  nmea_gpgga_processed = false;
  test_NMEA_ctx(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgga);
  ck_assert(nmea_gpgga_processed);
}
END_TEST

START_TEST(test_nmea_gprmc) {
  nmea_gprmc_processed = false;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gprmc);
  ck_assert(nmea_gprmc_processed);

  nmea_gprmc_processed = false;
  test_NMEA_ctx(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gprmc);
  ck_assert(nmea_gprmc_processed);
}
END_TEST

START_TEST(test_nmea_gpvtg) {
  nmea_gpvtg_processed = false;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpvtg);
  ck_assert(nmea_gpvtg_processed);

  nmea_gpvtg_processed = false;
  test_NMEA_ctx(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpvtg);
  ck_assert(nmea_gpvtg_processed);
}
END_TEST

/*START_TEST(test_nmea_gphdt) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp",
             nmea_callback_gphdt);
  ck_assert(nmea_gphdt_processed);
}
END_TEST */

START_TEST(test_nmea_gpgll) {
  nmea_gpgll_processed = false;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgll);
  ck_assert(nmea_gpgll_processed);

  nmea_gpgll_processed = false;
  test_NMEA_ctx(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgll);
  ck_assert(nmea_gpgll_processed);
}
END_TEST

START_TEST(test_nmea_gpzda) {
  nmea_gpzda_processed = false;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpzda);
  ck_assert(nmea_gpzda_processed);

  nmea_gpzda_processed = false;
  test_NMEA_ctx(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpzda);
  ck_assert(nmea_gpzda_processed);
}
END_TEST

START_TEST(test_nmea_gsa) {
  nmea_gsa_processed = false;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gsa);
  ck_assert(nmea_gsa_processed);

  nmea_gsa_processed = false;
  test_NMEA_ctx(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gsa);
  ck_assert(nmea_gsa_processed);
}
END_TEST

START_TEST(test_nmea_gpgst) {
  nmea_gpgst_processed = false;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgst);
  ck_assert(nmea_gpgst_processed);

  nmea_gpgst_processed = false;
  test_NMEA_ctx(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgst);
  ck_assert(nmea_gpgst_processed);
}
END_TEST

START_TEST(test_nmea_gpgsv) {
  nmea_gpgsv_processed = false;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/azel-sbp.sbp", nmea_callback_gpgsv);
  ck_assert(nmea_gpgsv_processed);

  nmea_gpgsv_processed = false;
  test_NMEA_ctx(RELATIVE_PATH_PREFIX "/data/azel-sbp.sbp", nmea_callback_gpgsv);
  ck_assert(nmea_gpgsv_processed);
}
END_TEST

static bool check_utc_time_string(const msg_utc_time_t *msg_time,
                                  const char utc_time[],
                                  const char utc_timedate_trunc[],
                                  const char utc_timedate[]) {
  char utc_str[23];
  get_utc_time_string(true, false, false, msg_time, utc_str, sizeof(utc_str));
  if (strncmp(utc_str, utc_time, 9)) {
    fprintf(stderr, "expected %s, got %s\n", utc_time, utc_str);
    return false;
  }
  if (utc_timedate_trunc) {
    get_utc_time_string(true, true, true, msg_time, utc_str, sizeof(utc_str));
    if (strncmp(utc_str, utc_timedate_trunc, 16)) {
      fprintf(stderr, "expected %s, got %s\n", utc_timedate_trunc, utc_str);
      return false;
    }
  }
  if (utc_timedate) {
    get_utc_time_string(true, true, false, msg_time, utc_str, sizeof(utc_str));
    if (strncmp(utc_str, utc_timedate, 20)) {
      fprintf(stderr, "expected %s, got %s\n", utc_timedate, utc_str);
      return false;
    }
  }
  return true;
}

START_TEST(test_nmea_time_string) {
  msg_utc_time_t msg_time;
  msg_time.flags = 1;
  msg_time.tow = 0;
  msg_time.year = 2018;
  msg_time.month = 1;
  msg_time.day = 1;
  msg_time.hours = 17;
  msg_time.minutes = 9;
  msg_time.seconds = 31;
  msg_time.ns = 0;

  ck_assert(check_utc_time_string(
      &msg_time, "170931.00", "170931.00,010118", "170931.00,01,01,2018"));

  msg_time.ns = 4000000;
  ck_assert(check_utc_time_string(
      &msg_time, "170931.00", "170931.00,010118", "170931.00,01,01,2018"));

  msg_time.ns = 5000000;
  ck_assert(check_utc_time_string(
      &msg_time, "170931.01", "170931.01,010118", "170931.01,01,01,2018"));

  msg_time.ns = 994999999;
  ck_assert(check_utc_time_string(
      &msg_time, "170931.99", "170931.99,010118", "170931.99,01,01,2018"));

  msg_time.ns = 999999999;
  ck_assert(check_utc_time_string(
      &msg_time, "170932.00", "170932.00,010118", "170932.00,01,01,2018"));

  msg_time.seconds = 59;
  ck_assert(check_utc_time_string(
      &msg_time, "171000.00", "171000.00,010118", "171000.00,01,01,2018"));

  msg_time.minutes = 59;
  ck_assert(check_utc_time_string(
      &msg_time, "180000.00", "180000.00,010118", "180000.00,01,01,2018"));

  msg_time.hours = 23;
  ck_assert(check_utc_time_string(
      &msg_time, "000000.00", "000000.00,020118", "000000.00,02,01,2018"));

  msg_time.day = 31;
  ck_assert(check_utc_time_string(
      &msg_time, "000000.00", "000000.00,010218", "000000.00,01,02,2018"));

  msg_time.month = 12;
  ck_assert(check_utc_time_string(
      &msg_time, "000000.00", "000000.00,010119", "000000.00,01,01,2019"));

  /* leap second */
  msg_time.year = 2016;
  /* TODO: this fails, see the note in nmea.c/get_utc_time_string */
  check_utc_time_string(
      &msg_time, "235960.00", "235960.00,311216", "235960.00,31,12,2016");

  /* leap day */
  msg_time.month = 2;
  msg_time.day = 28;
  ck_assert(check_utc_time_string(
      &msg_time, "000000.00", "000000.00,290216", "000000.00,29,02,2016"));
}
END_TEST

Suite *nmea_suite(void) {
  Suite *s = suite_create("NMEA");

  TCase *tc_nmea = tcase_create("NMEA");
  tcase_add_checked_fixture(tc_nmea, nmea_setup_basic, NULL);
  tcase_add_test(tc_nmea, test_nmea_gpgga);
  tcase_add_test(tc_nmea, test_nmea_gprmc);
  tcase_add_test(tc_nmea, test_nmea_gpvtg);
  /*tcase_add_test(tc_nmea, test_nmea_gphdt);*/
  tcase_add_test(tc_nmea, test_nmea_gpgll);
  tcase_add_test(tc_nmea, test_nmea_gpzda);
  tcase_add_test(tc_nmea, test_nmea_gsa);
  tcase_add_test(tc_nmea, test_nmea_gpgst);
  tcase_add_test(tc_nmea, test_nmea_gpgsv);
  tcase_add_test(tc_nmea, test_nmea_time_string);
  suite_add_tcase(s, tc_nmea);

  return s;
}
