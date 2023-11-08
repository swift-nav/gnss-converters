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

#include <cassert>
#include <cstdio>

#include "catch/catch.hpp"
#include "message_gloephemeris.h"
#include "parser.h"

static void logfn(const char *msg) { printf("%s\n", msg); }

namespace Novatel {

static int readfn(uint8_t *buf, uint32_t n_bytes, void *context) {
  (void)context;
  static FILE *f = nullptr;
  if (f == nullptr) {
    f = fopen(UNIT_TEST_DATA_PATH "/novatel.bin", "r");
    assert(f);
  }
  if (feof(f) != 0) {
    fclose(f);
    return -1;
  }
  size_t n = fread(buf, 1, n_bytes, f);
  return n;
}

static int num_messages = 0;
static void count_message(const BinaryHeader *header, const void *data) {
  (void)header;
  (void)data;
  num_messages++;
}

TEST_CASE("Will it run?") {
  Message::CallbackArray callbacks{{
      Message::Callback{Message::GPSEPHEM, count_message},
      Message::Callback{Message::BESTPOS, count_message},
      Message::Callback{Message::RANGECMP, count_message},
      Message::Callback{Message::GLOEPHEMERIS, count_message},
  }};
  Parser parser{readfn, logfn, callbacks, nullptr};
  parser.process();

  REQUIRE(num_messages == 5701);
}
}  // namespace Novatel
