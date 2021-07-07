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

#include <gnss-converters/options.h>
#include <rtcm3/messages.h>
#include <rtcm3/msm_utils.h>
#include <rtcm3_utils.h>

#define FLOAT_EPS 1e-6

START_TEST(test_msm_glo_fcn_bias) {
  sbp_glo_code_bias[0] = 5.0f;
  sbp_glo_code_bias[1] = 7.5f;

  sbp_glo_phase_bias[0] = 2.5f;
  sbp_glo_phase_bias[1] = 1.1f;

  rtcm_msm_header header;
  u8 signal_index = 0;

  for (size_t mask_count = 0; mask_count < MSM_SIGNAL_MASK_SIZE; mask_count++) {
    header.signal_mask[mask_count] = true;
  }

  for (size_t i = 0; i < CONSTELLATION_COUNT; i++) {
    header.msg_num = to_msm_msg_num(i, MSM5);

    for (size_t j = 0; j < CODE_COUNT; j++) {
      double code = 3.0;
      double phase = 2.0;
      msm_glo_fcn_bias(&header, signal_index, 3, &code, &phase);
      bool is_glo =
          (to_constellation(header.msg_num) == RTCM_CONSTELLATION_GLO);
      if (is_glo &&
          (signal_index == 3 || signal_index == 2)) { /* GLONASS L1OF or L1P */
        ck_assert(fabs(code - 8.0) < FLOAT_EPS);
        ck_assert(fabs(phase - 9.5) < FLOAT_EPS);
      } else if (is_glo && (signal_index == 8 ||
                            signal_index == 9)) { /* GLONASS L2OF or L2P */
        ck_assert(fabs(code - 5.5) < FLOAT_EPS);
        ck_assert(fabs(phase - 3.1) < FLOAT_EPS);
      } else {
        ck_assert(fabs(code - 0.0) < FLOAT_EPS);
        ck_assert(fabs(phase - 0.0) < FLOAT_EPS);
      }
    }
  }
}
END_TEST

Suite *options_suite(void) {
  Suite *s = suite_create("options");

  TCase *tc_options = tcase_create("options");
  tcase_add_test(tc_options, test_msm_glo_fcn_bias);
  suite_add_tcase(s, tc_options);

  return s;
}
