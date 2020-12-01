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

#include "catch/catch.hpp"
#include "read_little_endian.h"

static constexpr uint8_t test_bytes[] = {
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x00};

TEST_CASE("Read bits.") {
  // Aligned read of 8.
  REQUIRE(0xFF == Novatel::Util::read_le_bits8(test_bytes, 0, 8));
  REQUIRE(0xFF == Novatel::Util::read_le_bits8(test_bytes, 16, 8));

  // Unaligned read of 8.
  REQUIRE(0xFF == Novatel::Util::read_le_bits8(test_bytes, 33, 8));
  REQUIRE(0xFF == Novatel::Util::read_le_bits8(test_bytes, 36, 8));
  REQUIRE(0xFF == Novatel::Util::read_le_bits8(test_bytes, 39, 8));

  // Aligned read of 16.
  REQUIRE(0x00FF == Novatel::Util::read_le_bits16(test_bytes, 0, 16));
  REQUIRE(0x00FF == Novatel::Util::read_le_bits16(test_bytes, 16, 16));

  // Unaligned read of 16.
  REQUIRE(0x807F == Novatel::Util::read_le_bits16(test_bytes, 1, 16));
  REQUIRE(0xFE01 == Novatel::Util::read_le_bits16(test_bytes, 7, 16));

  // Aligned read of 32.
  REQUIRE(0x00FF00FF == Novatel::Util::read_le_bits32(test_bytes, 0, 32));
  REQUIRE(0x00FFFFFF == Novatel::Util::read_le_bits32(test_bytes, 32, 32));

  // Unaligned read of 32.
  REQUIRE(0x807F807F == Novatel::Util::read_le_bits32(test_bytes, 1, 32));
  REQUIRE(0x01FFFFFE == Novatel::Util::read_le_bits32(test_bytes, 31, 32));

  // Aligned read of 64.
  REQUIRE(0x00FFFFFF00FF00FFLL ==
          Novatel::Util::read_le_bits64(test_bytes, 0, 64));

  // Read 0 bits.
  REQUIRE(0x00 == Novatel::Util::read_le_bits8(test_bytes, 31, 0));

  // Read 1 bit.
  REQUIRE(0x01 == Novatel::Util::read_le_bits8(test_bytes, 32, 1));

  // Read large odd number of bits.
  REQUIRE(0x1FE01FE01FLL == Novatel::Util::read_le_bits64(test_bytes, 3, 37));
}

TEST_CASE("Read a negative number.") {
  REQUIRE(-16711936 == Novatel::Util::read_le_int32(test_bytes + 1));
}
