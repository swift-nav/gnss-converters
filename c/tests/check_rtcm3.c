#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <check.h>

#include "rtcm3_sbp_test.h"

#include "check_suites.h"

START_TEST(test_rtcm3)
{
  ck_assert_int_eq(rtcm3_sbp_test(), 0);
}
END_TEST

Suite* rtcm3_suite(void)
{
  Suite *s = suite_create("RTCMv3");

  TCase *tc_core = tcase_create("Core");
  tcase_add_test(tc_core, test_rtcm3);
  suite_add_tcase(s, tc_core);

  return s;
}

