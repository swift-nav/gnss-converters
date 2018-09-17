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
#include <gnss-converters/sbp_nmea.h>
#include <libsbp/sbp.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "check_suites.h"
#include "config.h"
#include "nmea_truth.h"

static struct sbp_nmea_state state;
static sbp_msg_callbacks_node_t gps_time_callback_node;
static sbp_msg_callbacks_node_t utc_time_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
static sbp_msg_callbacks_node_t vel_ned_callback_node;
static sbp_msg_callbacks_node_t dops_callback_node;
static sbp_msg_callbacks_node_t age_correction_callback_node;
static sbp_msg_callbacks_node_t baseline_heading_callback_node;
static sbp_msg_callbacks_node_t observation_callback_node;

static bool nmea_gpgga_processed = false;
static bool nmea_gprmc_processed = false;
static bool nmea_gpvtg_processed = false;
static bool nmea_gpgll_processed = false;
static bool nmea_gpzda_processed = false;
/*static bool nmea_gphdt_processed = false;*/
static bool nmea_gsa_processed = false;

uint32_t read_file(uint8_t *buff, uint32_t n, void *context) {
  FILE *f = (FILE *)context;
  return (uint32_t)(fread(buff, 1, n, f));
}

void nmea_callback_gpgga(u8 msg[]) {
  if (strstr((char *)msg, "GPGGA")) {
    static int msg_count = 0;
    static bool start_count = false;
    nmea_gpgga_processed = true;
    if ((strstr((char *)msg, "171104.30") || start_count) && msg_count < 10) {
      start_count = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg, gpgga_truth[msg_count]) == 0);
      msg_count++;
    }
  }
}

void nmea_callback_gprmc(u8 msg[]) {
  if (strstr((char *)msg, "GPRMC")) {
    static int msg_count = 0;
    static bool start_count = false;
    nmea_gprmc_processed = true;
    if ((strstr((char *)msg, "171105.00") || start_count) && msg_count < 10) {
      start_count = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg, gprmc_truth[msg_count]) == 0);
      msg_count++;
    }
  }
}

void nmea_callback_gpvtg(u8 msg[]) {
  if (strstr((char *)msg, "GPVTG")) {
    static int msg_count = 0;
    static bool start_count = false;
    nmea_gpvtg_processed = true;
    if ((strstr((char *)msg, "$GPVTG,,T,,M,0.02,N,0.03,K,D*27") ||
         start_count) &&
        msg_count < 10) {
      start_count = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg, gpvtg_truth[msg_count]) == 0);
      msg_count++;
    }
  }
}

void nmea_callback_gpgll(u8 msg[]) {
  if (strstr((char *)msg, "GPGLL")) {
    static int msg_count = 0;
    static bool start_count = false;
    nmea_gpgll_processed = true;
    if ((strstr((char *)msg, "171105.00") || start_count) && msg_count < 10) {
      start_count = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg, gpgll_truth[msg_count]) == 0);
      msg_count++;
    }
  }
}

void nmea_callback_gpzda(u8 msg[]) {
  if (strstr((char *)msg, "GPZDA")) {
    static int msg_count = 0;
    static bool start_count = false;
    nmea_gpzda_processed = true;
    if ((strstr((char *)msg, "171105.00") || start_count) && msg_count < 10) {
      start_count = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg, gpzda_truth[msg_count]) == 0);
      msg_count++;
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
  if (strstr((char *)msg, "GNGSA")) {
    static int msg_count = 0;
    static bool start_count = false;
    nmea_gsa_processed = true;
    if ((strstr((char *)msg,
                "$GNGSA,A,3,70,71,76,77,86,87,88,,,,,,1.2,1.0,0.7*28") ||
         start_count) &&
        msg_count < 10) {
      start_count = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg, gsa_truth[msg_count++]) == 0);
    }
  }
}

void gps_time_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)context;
  (void)length;
  (void)sender_id;
  msg_gps_time_t *sbp_gps_time = (msg_gps_time_t *)msg;
  sbp2nmea_gps_time(sbp_gps_time, &state);
}

void utc_time_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)context;
  (void)length;
  (void)sender_id;
  msg_utc_time_t *sbp_utc_time = (msg_utc_time_t *)msg;
  sbp2nmea_utc_time(sbp_utc_time, &state);
}

void pos_llh_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)context;
  (void)length;
  (void)sender_id;
  msg_pos_llh_t *sbp_pos_llh = (msg_pos_llh_t *)msg;
  sbp2nmea_pos_llh(sbp_pos_llh, &state);
}

void vel_ned_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)context;
  (void)length;
  (void)sender_id;
  msg_vel_ned_t *sbp_vel_ned = (msg_vel_ned_t *)msg;
  sbp2nmea_vel_ned(sbp_vel_ned, &state);
}

void dops_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)context;
  (void)length;
  (void)sender_id;
  msg_dops_t *sbp_dops = (msg_dops_t *)msg;
  sbp2nmea_dops(sbp_dops, &state);
}

void age_correction_callback(u16 sender_id,
                             u8 length,
                             u8 msg[],
                             void *context) {
  (void)context;
  (void)length;
  (void)sender_id;
  msg_age_corrections_t *sbp_age_corr = (msg_age_corrections_t *)msg;
  sbp2nmea_age_corrections(sbp_age_corr, &state);
}

void baseline_heading_callback(u16 sender_id,
                               u8 length,
                               u8 msg[],
                               void *context) {
  (void)context;
  (void)length;
  (void)sender_id;
  msg_baseline_heading_t *sbp_baseline_heading = (msg_baseline_heading_t *)msg;
  sbp2nmea_baseline_heading(sbp_baseline_heading, &state);
}

void observation_callback(u16 sender_id, u8 length, u8 msg[], void *context) {
  (void)context;
  (void)sender_id;
  msg_obs_t *sbp_obs = (msg_obs_t *)msg;
  uint8_t num_obs =
      (length - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);
  sbp2nmea_obs(sbp_obs, num_obs, &state);
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
                        SBP_MSG_POS_LLH,
                        &pos_llh_callback,
                        ctx,
                        &pos_llh_callback_node);
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
                        SBP_MSG_OBS,
                        &observation_callback,
                        ctx,
                        &observation_callback_node);
}

void test_NMEA(const char *filename, void (*cb_sbp_to_nmea)(u8 msg[])) {
  sbp2nmea_init(&state, cb_sbp_to_nmea);
  sbp2nmea_set_base_id(33, &state);
  sbp2nmea_set_soln_freq(10, &state);
  sbp2nmea_set_gpgga_rate(1, &state);
  sbp2nmea_set_gprmc_rate(10, &state);
  sbp2nmea_set_gpvtg_rate(10, &state);
  /*sbp2nmea_set_gphdt_rate(1,&state);*/
  sbp2nmea_set_gpgll_rate(10, &state);
  sbp2nmea_set_gpzda_rate(10, &state);
  sbp2nmea_set_gsa_rate(1, &state);

  sbp_state_t sbp_state_;
  sbp_init(&sbp_state_, NULL);

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
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgga);
  ck_assert(nmea_gpgga_processed);
}
END_TEST

START_TEST(test_nmea_gprmc) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gprmc);
  ck_assert(nmea_gprmc_processed);
}
END_TEST

START_TEST(test_nmea_gpvtg) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpvtg);
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
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgll);
  ck_assert(nmea_gpgll_processed);
}
END_TEST

START_TEST(test_nmea_gpzda) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpzda);
  ck_assert(nmea_gpzda_processed);
}
END_TEST

START_TEST(test_nmea_gsa) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gsa);
  ck_assert(nmea_gsa_processed);
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
  suite_add_tcase(s, tc_nmea);

  return s;
}
