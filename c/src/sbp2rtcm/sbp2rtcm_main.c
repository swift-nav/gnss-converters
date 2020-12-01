/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static int32_t readfn(uint8_t *buff, uint32_t n, void *context) {
  (void)context;
  ssize_t read_bytes = read(STDIN_FILENO, buff, n);
  if (read_bytes < 0) {
    fprintf(stderr, "Read failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }
  if (n > 0 && read_bytes == 0) {
    /* EOF */
    exit(EXIT_SUCCESS);
  }
  return (int32_t)read_bytes;
}

/* Write the RTCM frame to STDOUT. */
static int32_t writefn(uint8_t *buffer, uint16_t n, void *context) {
  (void)(context);
  ssize_t numwritten = write(STDOUT_FILENO, buffer, n);
  if (numwritten < n) {
    fprintf(stderr, "Write failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }
  return (int32_t)numwritten;
}

typedef int32_t (*readfn_ptr)(uint8_t *, uint32_t, void *);
typedef int32_t (*writefn_ptr)(uint8_t *, uint16_t, void *);

int sbp2rtcm_main(int, char **, const char *, readfn_ptr, writefn_ptr);

int main(int argc, char **argv) {
  return sbp2rtcm_main(argc, argv, "", readfn, writefn);
}
