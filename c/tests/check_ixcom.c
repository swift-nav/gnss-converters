/*
 * Copyright (C) 2020 Swift Navigation Inc.
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

#include <libsbp/edc.h>
#include <libsbp/imu.h>
#include <libsbp/vehicle.h>

#include <gnss-converters/ixcom_sbp.h>

#include "check_ixcom.h"
#include "check_suites.h"
#include "config.h"

static FILE *fp;

int read_file_check_ixcom(uint8_t *buf, size_t len, void *ctx) {
  (void)ctx;
  return fread(buf, sizeof(uint8_t), len, fp);
}

void test_IXCOM(struct ixcom_sbp_state *state, const char *filename) {
  fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  int ret;
  do {
    ret = ixcom_sbp_process(state, &read_file_check_ixcom);
  } while (ret > 0);
}

static const uint16_t imuraw_crc[] = {183, 5104, 48724};
static void ixcom_sbp_callback_imuraw(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static uint8_t msg_index = 0;

  if (msg_index == 0 || msg_index == 2) {
    ck_assert(msg_id == SBP_MSG_IMU_RAW);
    ck_assert(length == sizeof(msg_imu_raw_t));
  } else if (msg_index == 1) {
    ck_assert(msg_id == SBP_MSG_IMU_AUX);
    ck_assert(length == sizeof(msg_imu_aux_t));
  }

  ck_assert(msg_index < sizeof(imuraw_crc) / sizeof(uint16_t));

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == imuraw_crc[msg_index]);

  msg_index++;
}

static const uint16_t wheeldata_crc[] = {46759, 7510};
static void ixcom_sbp_callback_wheeldata(
    u16 msg_id, u8 length, u8 *buff, u16 sender_id, void *context) {
  (void)context;
  static uint8_t msg_index = 0;

  if (msg_index == 0) {
    ck_assert(msg_id == SBP_MSG_ODOMETRY);
    ck_assert(length == sizeof(msg_odometry_t));
  } else if (msg_index == 1) {
    ck_assert(msg_id == SBP_MSG_WHEELTICK);
    ck_assert(length == sizeof(msg_wheeltick_t));
  }

  ck_assert(msg_index < sizeof(wheeldata_crc) / sizeof(uint16_t));

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_id;
  tmpbuf[1] = (uint8_t)(msg_id >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)length;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(buff, length, crc);
  ck_assert(crc == wheeldata_crc[msg_index]);

  msg_index++;
}

START_TEST(test_imuraw) {
  struct ixcom_sbp_state state;
  ixcom_sbp_init(&state, ixcom_sbp_callback_imuraw, NULL);

  test_IXCOM(&state, RELATIVE_PATH_PREFIX "/data/ixcom_imuraw.bin");
}
END_TEST

START_TEST(test_wheeldata) {
  struct ixcom_sbp_state state;
  ixcom_sbp_init(&state, ixcom_sbp_callback_wheeldata, NULL);

  test_IXCOM(&state, RELATIVE_PATH_PREFIX "/data/ixcom_wheeldata.bin");
}
END_TEST

Suite *ixcom_suite(void) {
  Suite *s = suite_create("IXCOM");

  TCase *tc_imuraw = tcase_create("IXCOM_IMURAW");
  tcase_add_test(tc_imuraw, test_imuraw);
  TCase *tc_wheeldata = tcase_create("IXCOM_WHEELDATA");
  tcase_add_test(tc_wheeldata, test_wheeldata);
  suite_add_tcase(s, tc_imuraw);
  suite_add_tcase(s, tc_wheeldata);

  return s;
}
