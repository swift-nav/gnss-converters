#define _GNU_SOURCE

#include "make_afl_testcases.h"
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void write_file(const char *name, const uint8_t *buf, uint16_t len) {
  int fd = open(name, O_WRONLY | O_CREAT | O_CLOEXEC, 0644);
  assert(fd != -1);
  ssize_t nr = write(fd, buf, len);
  (void)nr;
  assert(nr == len);
  close(fd);
}

int main(int argc, char *argv[]) {
  (void)argc;
  (void)argv;
  make_rtcm_testcases();
  make_ubx_testcases();
  return 0;
}
