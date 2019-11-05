/*
 * Copyright (C) 2019 Swift Navigation Inc.
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

#include <gnss-converters/ubx_sbp.h>

#include "check_suites.h"
#include "check_ubx.h"
#include "config.h"

uint16_t rxm_rawx_crc[] = {14708, 41438};
FILE *fp;
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

uint16_t nav_pvt_crc = 23845;
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
  
int read_file_check_ubx(uint8_t * buf, size_t len, void *ctx) {
    (void)ctx;
    return fread(buf, sizeof(uint8_t), len, fp);
  }

void test_UBX(const char *filename,
              void (*cb_ubx_to_sbp)(
                  u16 msg_id, u8 length, u8 *buf, u16 sender_id, void *ctx),
              void *context) {
  static struct ubx_sbp_state state;

  fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  ubx_sbp_init(&state, cb_ubx_to_sbp, context);

  int ret;
  do {
    ret = ubx_sbp_process(&state, &read_file_check_ubx);
  } while (ret > 0);
}

START_TEST(test_nav_pvt) {
  test_UBX(
      RELATIVE_PATH_PREFIX "/data/nav_pvt.ubx", ubx_sbp_callback_nav_pvt, NULL);
}
END_TEST

START_TEST(test_rxm_rawx) {
  test_UBX(RELATIVE_PATH_PREFIX "/data/rxm_rawx.ubx",
           ubx_sbp_callback_rxm_rawx,
           NULL);
}
END_TEST

Suite *ubx_suite(void) {
  Suite *s = suite_create("UBX");

  TCase *tc_nav_pvt = tcase_create("NAV_PVT");
  tcase_add_test(tc_nav_pvt, test_nav_pvt);
  tcase_add_test(tc_nav_pvt, test_rxm_rawx);
  suite_add_tcase(s, tc_nav_pvt);

  return s;
}
