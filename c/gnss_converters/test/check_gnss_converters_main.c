/*
 * Copyright (C) 2018 Swift Navigation Inc.
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
#include <stdlib.h>

#include "check_gnss_converters.h"

int main(void) {
  int number_failed = 0;

  Suite *s = {0};

  SRunner *sr = srunner_create(s);
  srunner_set_xml(sr, "test_results.xml");

  srunner_add_suite(sr, nmea_suite());
  srunner_add_suite(sr, nmea_gpths_suite());
  srunner_add_suite(sr, options_suite());
  srunner_add_suite(sr, time_truth_suite());

  srunner_set_fork_status(sr, CK_NOFORK);
  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
