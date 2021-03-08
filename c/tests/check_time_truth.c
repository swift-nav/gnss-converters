/**
 * Copyright (C) 2021 Swift Navigation Inc.
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
#include <gnss-converters/time_truth.h>
#include <stdarg.h>
#include <stdio.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

/** Build time */
static const gps_time_t BUILD_GPS_TIME = {.wn = GPS_WEEK_REFERENCE, .tow = 0};

/** Expected state change to SOLVED message */
const char SOLVED_STATE_TRANSITION[] = "Time truth state changed to \"Solved\"";

/** Expected invalid GPS time message */
const char INVALID_TIME_MESSAGE[] = "Invalid time submitted to time truth";

/** Expected GPS time predates build time message */
const char PREDATES_BUILD_TIME_MESSAGE[] =
    "Time submitted to time truth predates build time";

/**
 * Fixture Data
 */

static bool log_invoked;
static int log_level;
static char log_message[100];

static void reset_log_flags() {
  log_invoked = false;
  log_level = -1;
  memset(log_message, 0, sizeof(log_message));
}

static void swiftnav_log(int level,
                         LIBSWIFTNAV_FORMAT_STRING const char *msg,
                         ...) {
  (void)level;
  (void)msg;

  log_invoked = true;
  log_level = level;

  va_list ap;
  va_start(ap, msg);
  vsnprintf(log_message,
            sizeof(log_message) / sizeof(log_message[0]),
            msg,
            ap);  // NOLINT
  va_end(ap);
}

static void swiftnav_detailed_log(int level,
                                  const char *file_path,
                                  const int line_number,
                                  LIBSWIFTNAV_FORMAT_STRING const char *msg,
                                  ...) {
  (void)level;
  (void)file_path;
  (void)line_number;
  (void)msg;
  log_invoked = true;
  log_level = level;
}

/**
 * STTT-1
 */
START_TEST(test_default_init) {
  enum time_truth_state state;
  gps_time_t time;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_truth_get(&time_truth, &state, &time);
  ck_assert(state == TIME_TRUTH_UNKNOWN);
  ck_assert(!gps_time_valid(&time));

  time_truth_get(&time_truth, NULL, NULL);
}
END_TEST

/**
 * STTT-2
 */
START_TEST(test_unknown_state_eph_gps) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = 2153;
  time_in.tow = 198581;
  ck_assert(gps_time_valid(&time_in));
  ck_assert(time_truth_update(&time_truth, TIME_TRUTH_EPH_GPS, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_INFO);
  ck_assert_str_eq(log_message, SOLVED_STATE_TRANSITION);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_SOLVED);
  ck_assert(gps_time_valid(&time_out));
  ck_assert_int_eq(time_out.wn, time_in.wn);
  ck_assert_double_eq(time_out.tow, time_in.tow);
}
END_TEST

/**
 * STTT-3
 */
START_TEST(test_unknown_state_eph_gal) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = 2153;
  time_in.tow = 199367;
  ck_assert(gps_time_valid(&time_in));
  ck_assert(time_truth_update(&time_truth, TIME_TRUTH_EPH_GAL, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_INFO);
  ck_assert_str_eq(log_message, SOLVED_STATE_TRANSITION);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_SOLVED);
  ck_assert(gps_time_valid(&time_out));
  ck_assert_int_eq(time_out.wn, time_in.wn);
  ck_assert_double_eq(time_out.tow, time_in.tow);
}
END_TEST

/**
 * STTT-4
 */
START_TEST(test_unknown_state_eph_bds) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = 2153;
  time_in.tow = 349502;
  ck_assert(gps_time_valid(&time_in));
  ck_assert(time_truth_update(&time_truth, TIME_TRUTH_EPH_BDS, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_INFO);
  ck_assert_str_eq(log_message, SOLVED_STATE_TRANSITION);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_SOLVED);
  ck_assert(gps_time_valid(&time_out));
  ck_assert_int_eq(time_out.wn, time_in.wn);
  ck_assert_double_eq(time_out.tow, time_in.tow);
}
END_TEST

/**
 * STTT-5
 */
START_TEST(test_unknown_state_invalid_eph_gps) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = WN_UNKNOWN;
  time_in.tow = 0;
  ck_assert(!gps_time_valid(&time_in));
  ck_assert(!time_truth_update(&time_truth, TIME_TRUTH_EPH_GPS, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_WARN);
  ck_assert_str_eq(log_message, INVALID_TIME_MESSAGE);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_UNKNOWN);
  ck_assert(!gps_time_valid(&time_out));
}
END_TEST

/**
 * STTT-6
 */
START_TEST(test_unknown_state_out_of_bounds_eph_gps) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = 0;
  time_in.tow = 0;
  ck_assert(gps_time_valid(&time_in));
  ck_assert(gpsdifftime(&time_in, &BUILD_GPS_TIME) < 0);
  ck_assert(!time_truth_update(&time_truth, TIME_TRUTH_EPH_GPS, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_WARN);
  ck_assert_str_eq(log_message, PREDATES_BUILD_TIME_MESSAGE);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_UNKNOWN);
  ck_assert(!gps_time_valid(&time_out));
}
END_TEST

/**
 * STTT-7
 */
START_TEST(test_unknown_state_invalid_eph_gal) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = WN_UNKNOWN;
  time_in.tow = 0;
  ck_assert(!gps_time_valid(&time_in));
  ck_assert(!time_truth_update(&time_truth, TIME_TRUTH_EPH_GAL, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_WARN);
  ck_assert_str_eq(log_message, INVALID_TIME_MESSAGE);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_UNKNOWN);
  ck_assert(!gps_time_valid(&time_out));
}
END_TEST

/**
 * STTT-8
 */
START_TEST(test_unknown_state_out_of_bounds_eph_gal) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = 1023;
  time_in.tow = 604799;
  ck_assert(gps_time_valid(&time_in));
  ck_assert(gpsdifftime(&time_in, &BUILD_GPS_TIME) < 0);
  ck_assert(!time_truth_update(&time_truth, TIME_TRUTH_EPH_GAL, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_WARN);
  ck_assert_str_eq(log_message, PREDATES_BUILD_TIME_MESSAGE);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_UNKNOWN);
  ck_assert(!gps_time_valid(&time_out));
}
END_TEST

/**
 * STTT-9
 */
START_TEST(test_unknown_state_invalid_eph_bds) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = WN_UNKNOWN;
  time_in.tow = 0;
  ck_assert(!gps_time_valid(&time_in));
  ck_assert(!time_truth_update(&time_truth, TIME_TRUTH_EPH_BDS, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_WARN);
  ck_assert_str_eq(log_message, INVALID_TIME_MESSAGE);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_UNKNOWN);
  ck_assert(!gps_time_valid(&time_out));
}
END_TEST

/**
 * STTT-10
 */
START_TEST(test_unknown_state_out_of_bounds_eph_bds) {
  enum time_truth_state state;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_in.wn = 1356;
  time_in.tow = 13;
  ck_assert(gps_time_valid(&time_in));
  ck_assert(gpsdifftime(&time_in, &BUILD_GPS_TIME) < 0);
  ck_assert(!time_truth_update(&time_truth, TIME_TRUTH_EPH_BDS, time_in));
  ck_assert(log_invoked);
  ck_assert(log_level == LOG_WARN);
  ck_assert_str_eq(log_message, PREDATES_BUILD_TIME_MESSAGE);

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_UNKNOWN);
  ck_assert(!gps_time_valid(&time_out));
}
END_TEST

/**
 * STTT-11
 */
START_TEST(test_solved_state_plus_one_second) {
  enum time_truth_state state;
  gps_time_t time_init;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_init.wn = 2153;
  time_init.tow = 350929;
  ck_assert(time_truth_update(&time_truth, TIME_TRUTH_EPH_GAL, time_init));
  reset_log_flags();

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_SOLVED);
  ck_assert(time_out.wn == time_init.wn);
  ck_assert(time_out.tow == time_init.tow);

  enum time_truth_source sources[] = {
      TIME_TRUTH_EPH_GPS, TIME_TRUTH_EPH_GAL, TIME_TRUTH_EPH_BDS};
  for (size_t i = 0; i < sizeof(sources) / sizeof(sources[0]); ++i) {
    time_in.wn = 2153;
    time_in.tow = time_init.tow + i + 1;
    ck_assert(time_truth_update(&time_truth, sources[i], time_in));

    time_truth_get(&time_truth, &state, &time_out);
    ck_assert(state == TIME_TRUTH_SOLVED);
    ck_assert(gps_time_valid(&time_out));
    ck_assert_int_eq(time_out.wn, time_in.wn);
    ck_assert_double_eq(time_out.tow, time_in.tow);
    ck_assert(!log_invoked);
  }
}
END_TEST

/**
 * STTT-12
 */
START_TEST(test_solved_state_minus_one_second) {
  enum time_truth_state state;
  gps_time_t time_init;
  gps_time_t time_in;
  gps_time_t time_out;

  time_truth_t time_truth;
  time_truth_init(&time_truth);

  time_init.wn = 2153;
  time_init.tow = 350929;
  ck_assert(time_truth_update(&time_truth, TIME_TRUTH_EPH_GAL, time_init));
  reset_log_flags();

  time_truth_get(&time_truth, &state, &time_out);
  ck_assert(state == TIME_TRUTH_SOLVED);
  ck_assert(time_out.wn == time_init.wn);
  ck_assert(time_out.tow == time_init.tow);

  enum time_truth_source sources[] = {
      TIME_TRUTH_EPH_GPS, TIME_TRUTH_EPH_GAL, TIME_TRUTH_EPH_BDS};
  for (size_t i = 0; i < sizeof(sources) / sizeof(sources[0]); ++i) {
    time_in.wn = 2153;
    time_in.tow = 350928;
    ck_assert(!time_truth_update(&time_truth, sources[i], time_in));

    time_truth_get(&time_truth, &state, &time_out);
    ck_assert(state == TIME_TRUTH_SOLVED);
    ck_assert(gps_time_valid(&time_out));
    ck_assert_int_eq(time_out.wn, time_init.wn);
    ck_assert_double_eq(time_out.tow, time_init.tow);
    ck_assert(!log_invoked);
  }
}
END_TEST

static void suite_fixture_setup(void) {
  reset_log_flags();
  logging_set_implementation(swiftnav_log, swiftnav_detailed_log);
}

static void suite_fixture_teardown(void) {
  logging_set_implementation(NULL, NULL);
}

Suite *time_truth_suite(void) {
  Suite *suite = suite_create("Time Truth");

  TCase *test_case_unknown = tcase_create("Unknown State");
  tcase_add_checked_fixture(
      test_case_unknown, suite_fixture_setup, suite_fixture_teardown);

  tcase_add_test(test_case_unknown, test_default_init);
  tcase_add_test(test_case_unknown, test_unknown_state_eph_gps);
  tcase_add_test(test_case_unknown, test_unknown_state_eph_gal);
  tcase_add_test(test_case_unknown, test_unknown_state_eph_bds);
  tcase_add_test(test_case_unknown, test_unknown_state_invalid_eph_gps);
  tcase_add_test(test_case_unknown, test_unknown_state_out_of_bounds_eph_gps);
  tcase_add_test(test_case_unknown, test_unknown_state_invalid_eph_gal);
  tcase_add_test(test_case_unknown, test_unknown_state_out_of_bounds_eph_gal);
  tcase_add_test(test_case_unknown, test_unknown_state_invalid_eph_bds);
  tcase_add_test(test_case_unknown, test_unknown_state_out_of_bounds_eph_bds);
  suite_add_tcase(suite, test_case_unknown);

  TCase *test_case_solved = tcase_create("Solved State");
  tcase_add_checked_fixture(
      test_case_solved, suite_fixture_setup, suite_fixture_teardown);
  tcase_add_test(test_case_solved, test_solved_state_plus_one_second);
  tcase_add_test(test_case_solved, test_solved_state_minus_one_second);
  suite_add_tcase(suite, test_case_solved);

  return suite;
}