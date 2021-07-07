#include <assert.h>
#include <check.h>
#include <gnss-converters/sbp_nmea.h>
#include <libsbp/sbp.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/pvt_result.h>

#include "../src/sbp_nmea_internal.h"
#include "config.h"

#define MAX_NMEA_LEN 100

static void do_gpths_test_cb(char msg[], void *ctx) {
  strncpy(ctx, msg, MAX_NMEA_LEN);
}

#define do_gpths_test(a, b, c, d) \
  real_do_gpths_test(__FILE__, __LINE__, a, b, c, d)

static void real_do_gpths_test(const char *file,
                               int line,
                               uint8_t pos_mode,
                               uint8_t orient_valid,
                               s32 yaw,
                               const char *expected_ths) {
  char output[MAX_NMEA_LEN];
  memset(output, 0, sizeof(output));

  sbp2nmea_t state;
  sbp2nmea_init(&state, SBP2NMEA_MODE_BEST, do_gpths_test_cb, output);
  sbp2nmea_soln_freq_set(&state, 10);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_THS);

  sbp_msg_t sbp_msg;

  // Group meta first. Normally the fused wagon would be much bigger than this
  // but for GPTHS we only need POS_LLH_COV and ORIENT_EULER so can just skip
  // everything else. sbp2nmea looks for SOLN_META to mark the end of the fused
  // wagon so we have to include that one as well, plus GROUP_META to start the
  // sequence
  memset(&sbp_msg, 0, sizeof(sbp_msg));
  sbp_msg_group_meta_t *msg_group_meta = &sbp_msg.group_meta;
  msg_group_meta->n_group_msgs = 4;
  msg_group_meta->group_msgs[0] = SBP_MSG_UTC_TIME;
  msg_group_meta->group_msgs[1] = SBP_MSG_POS_LLH_COV;
  msg_group_meta->group_msgs[2] = SBP_MSG_ORIENT_EULER;
  msg_group_meta->group_msgs[3] = SBP_MSG_SOLN_META;
  msg_group_meta->n_group_msgs = 4;
  sbp2nmea(&state, &sbp_msg, SBP2NMEA_SBP_GROUP_META);

  // Need a UTC time message in all cases, only the tow needs to be correct
  memset(&sbp_msg, 0, sizeof(sbp_msg));
  sbp_msg_utc_time_t *msg_utc_time = &sbp_msg.utc_time;
  msg_utc_time->tow = 135264000;
  msg_utc_time->flags = 1;
  sbp2nmea(&state, &sbp_msg, SBP2NMEA_SBP_UTC_TIME);

  // Position. Don't really need any of the fields in this message other than
  // position mode and tow.
  memset(&sbp_msg, 0, sizeof(sbp_msg));
  sbp_msg_pos_llh_cov_t *msg_pos_llh_cov = &sbp_msg.pos_llh_cov;
  msg_pos_llh_cov->tow = 135264000;
  msg_pos_llh_cov->flags = pos_mode;
  sbp2nmea(&state, &sbp_msg, SBP2NMEA_SBP_POS_LLH_COV);

  // Now orient euler. The tow needs to match, also INS mode is in this message.
  // Only the yaw is required to generate GPTHS
  memset(&sbp_msg, 0, sizeof(sbp_msg));
  sbp_msg_orient_euler_t *msg_orient_euler = &sbp_msg.orient_euler;
  msg_orient_euler->tow = 135264000;
  msg_orient_euler->flags = orient_valid;
  msg_orient_euler->yaw = yaw;
  sbp2nmea(&state, &sbp_msg, SBP2NMEA_SBP_ORIENT_EULER);

  // Finally soln meta to finish it off. There isn't any interesting information
  // in this message, it just needs to exist
  memset(&sbp_msg, 0, sizeof(sbp_msg));
  sbp_msg_soln_meta_t *msg_soln_meta = &sbp_msg.soln_meta;
  msg_soln_meta->tow = 135264000;
  sbp2nmea(&state, &sbp_msg, SBP2NMEA_SBP_SOLN_META);

  ck_assert_msg(strncmp(output, expected_ths, strlen(expected_ths)) == 0,
                "%s:%d: Expected \"%s\" but got \"%s\"",
                file,
                line,
                expected_ths,
                output);
}

#define INVALID_GPTHS "$GPTHS,,V*0E"

START_TEST(test_nmea_gpths_no_orient) {
  // When orientation is invalid (unknown) GPTHS should be empty with a mode of
  // 'V'. This should hold true for all position modes. Note that not all of
  // these combinations of position mode and orient valid are actually realistic
  // in practice, but we test them anyway

  do_gpths_test(POSITION_MODE_NONE,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_INVALID,
                0,
                INVALID_GPTHS);
  do_gpths_test(POSITION_MODE_SPP,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_INVALID,
                0,
                INVALID_GPTHS);
  do_gpths_test(POSITION_MODE_DGNSS,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_INVALID,
                0,
                INVALID_GPTHS);
  do_gpths_test(POSITION_MODE_FLOAT,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_INVALID,
                0,
                INVALID_GPTHS);
  do_gpths_test(POSITION_MODE_FIXED,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_INVALID,
                0,
                INVALID_GPTHS);
  do_gpths_test(POSITION_MODE_DEAD_RECKONING,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_INVALID,
                0,
                INVALID_GPTHS);
  do_gpths_test(POSITION_MODE_SBAS,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_INVALID,
                0,
                INVALID_GPTHS);
}
END_TEST

START_TEST(test_nmea_gpths_autonomous) {
  // Autonomous means any position solution other than estimation. It is
  // indicated with an 'A' in the 2nd field.

  do_gpths_test(POSITION_MODE_SPP,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,
                0,
                "$GPTHS,0.00,A");
  do_gpths_test(POSITION_MODE_DGNSS,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,
                0,
                "$GPTHS,0.00,A");
  do_gpths_test(POSITION_MODE_FLOAT,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,
                0,
                "$GPTHS,0.00,A");
  do_gpths_test(POSITION_MODE_FIXED,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,
                0,
                "$GPTHS,0.00,A");
  do_gpths_test(POSITION_MODE_SBAS,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,
                0,
                "$GPTHS,0.00,A");
  return;
}
END_TEST

START_TEST(test_nmea_gpths_dr) {
  // When in dead reckoning there should be an 'E' in the 2nd field

  do_gpths_test(POSITION_MODE_DEAD_RECKONING,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,
                0,
                "$GPTHS,0.00,E");
}
END_TEST

START_TEST(test_nmea_gpths_no_fix) {
  // Slightly ill defined, not sure if this combination is possible. If there is
  // no position solution we should output an invalid sentence even if the
  // orientation message contains valid data

  do_gpths_test(POSITION_MODE_NONE,
                SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,
                0,
                INVALID_GPTHS);
}
END_TEST

START_TEST(test_nmea_gpths_range) {
  // In SBP yaw is indicated in microdegrees with a range of -180->+180
  // (confirm). In GPTHS it's indicated in degrees in the range 0->360. Check
  // the entire range of inputs for valid outputs.

  // This whole section is so much easier to read without clang-format getting
  // in the way since all the parameters line up and it's far easier to see what
  // is changing between each test cases
  //
  // clang-format off
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -180000001, "$GPTHS,180.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -180000001, "$GPTHS,180.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -180000000, "$GPTHS,180.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -180000000, "$GPTHS,180.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -179999999, "$GPTHS,180.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -179999999, "$GPTHS,180.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -179995000, "$GPTHS,180.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -179995000, "$GPTHS,180.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -179994999, "$GPTHS,180.01,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID, -179994999, "$GPTHS,180.01,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,     -10000, "$GPTHS,359.99,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,     -10000, "$GPTHS,359.99,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,      -5001, "$GPTHS,359.99,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,      -5001, "$GPTHS,359.99,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,      -5000, "$GPTHS,0.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,      -5000, "$GPTHS,0.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,         -1, "$GPTHS,0.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,         -1, "$GPTHS,0.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,          1, "$GPTHS,0.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,          1, "$GPTHS,0.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,       4999, "$GPTHS,0.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,       4999, "$GPTHS,0.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,       5000, "$GPTHS,0.01,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,       5000, "$GPTHS,0.01,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  179994999, "$GPTHS,179.99,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  179994999, "$GPTHS,179.99,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  179995000, "$GPTHS,180.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  179995000, "$GPTHS,180.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  179999999, "$GPTHS,180.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  179999999, "$GPTHS,180.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  180000000, "$GPTHS,180.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  180000000, "$GPTHS,180.00,E");
  do_gpths_test(POSITION_MODE_FIXED,          SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  180000001, "$GPTHS,180.00,A");
  do_gpths_test(POSITION_MODE_DEAD_RECKONING, SBP_ORIENT_EULER_INS_NAVIGATION_MODE_VALID,  180000001, "$GPTHS,180.00,E");
  // clang-format on
}
END_TEST

Suite *nmea_gpths_suite(void) {
  Suite *s = suite_create("NMEA GPTHS");

  TCase *tc_nmea_gpths = tcase_create("NMEA GPTHS");
  tcase_add_test(tc_nmea_gpths, test_nmea_gpths_no_orient);
  tcase_add_test(tc_nmea_gpths, test_nmea_gpths_autonomous);
  tcase_add_test(tc_nmea_gpths, test_nmea_gpths_dr);
  tcase_add_test(tc_nmea_gpths, test_nmea_gpths_no_fix);
  tcase_add_test(tc_nmea_gpths, test_nmea_gpths_range);
  suite_add_tcase(s, tc_nmea_gpths);

  return s;
}
