/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>

typedef int (*readfn_ptr)(uint8_t *, uint32_t, void *);
typedef int (*writefn_ptr)(uint8_t *, uint32_t, void *);

extern "C" int nov2sbp_main(int argc,
                            char **argv,
                            const char *additional_opts_help,
                            readfn_ptr,
                            writefn_ptr,
                            void *);

extern "C" void nov2sbp_proj_context_set_search_paths(const char *path);

static int readfn(uint8_t *bytes, uint32_t n_bytes, void *context) {
  (void)context;
  if (feof(stdin) != 0) {
    return -1;
  }
  return static_cast<int>(fread(bytes, sizeof(*bytes), n_bytes, stdin));
}

static int32_t writefn(uint8_t *bytes, uint32_t n_bytes, void *context) {
  (void)context;
  return static_cast<int>(fwrite(bytes, sizeof(*bytes), n_bytes, stdout));
}

int main(int argc, char **argv) {
  return nov2sbp_main(argc, argv, "", readfn, writefn, nullptr);
}
