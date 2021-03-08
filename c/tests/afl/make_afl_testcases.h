#ifndef MAKE_AFL_TESTCASES_H
#define MAKE_AFL_TESTCASES_H

#include <stdint.h>

void make_rtcm_testcases(void);
void make_ubx_testcases(void);
void write_file(const char *name, const uint8_t *buf, uint16_t len);

#endif
