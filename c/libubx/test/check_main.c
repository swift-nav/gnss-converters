#include <check.h>
#include <stdlib.h>

#include "check_suites.h"

int main(void) {
  int number_failed;

  Suite *s = {0};

  SRunner *sr = srunner_create(s);
  srunner_set_xml(sr, "test_results.xml");

  srunner_add_suite(sr, ubx_suite());

  srunner_set_fork_status(sr, CK_NOFORK);
  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}