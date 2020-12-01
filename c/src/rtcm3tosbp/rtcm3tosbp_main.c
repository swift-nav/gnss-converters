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

static int readfn(uint8_t *buf, size_t len, void *context) {
  (void)context; /* squash warning */
  return (int)read(STDIN_FILENO, buf, len);
}

static int writefn(const uint8_t *buf, size_t count, void *context) {
  (void)context;
  return (int)write(STDOUT_FILENO, buf, count);
}

typedef int (*readfn_ptr)(uint8_t *, size_t, void *);
typedef int (*writefn_ptr)(const uint8_t *, size_t, void *);

int rtcm3tosbp_main(int argc,
                    char **argv,
                    const char *additional_opts_help,
                    readfn_ptr readfn,
                    writefn_ptr writefn,
                    void *context);

int main(int argc, char **argv) {
  return rtcm3tosbp_main(argc, argv, "", readfn, writefn, NULL);
}
