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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libsbp/ssr.h>

#include <config.h>

#include "check_rtcm3.h"
#include "check_suites.h"

void sbp_callback_gps_code_bias(u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_id == SBP_MSG_SSR_CODE_BIASES && !msg_checked) {
    msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t*)buffer;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 1);
    ck_assert(sbp_code_bias->sid.code == 0);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);
    msg_checked = true;

    ck_assert(sbp_code_bias->biases[0].value == -340);
    ck_assert(sbp_code_bias->biases[0].code == 0);
    ck_assert(sbp_code_bias->biases[1].value == -368);
    printf("%u\n",sbp_code_bias->biases[1].code);
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

//"biases":[{"value":-340,"code":0},{"value":-368,"code":2},{"value":-338,"code":19},{"value":-567,"code":5},{"value":-566,"code":8},{"value":-563,"code":7},{"value":-607,"code":11},{"value":-567,"code":9},{"value":-315,"code":15},{"value":-306,"code":16}],"crc":44905,"sid":{"sat":1,"code":0}}

START_TEST(test_ssr_code_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gps_code_bias,
             current_time);
}
END_TEST

Suite *rtcm3_ssr_suite(void) {
  Suite *s = suite_create("RTCMv3_ssr");

  TCase *tc_ssr = tcase_create("SSR");
  tcase_add_checked_fixture(tc_ssr, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_ssr, test_ssr_code_bias);
  suite_add_tcase(s, tc_ssr);

  return s;
}
