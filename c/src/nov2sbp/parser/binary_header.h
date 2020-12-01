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

#ifndef NOVATEL_PARSER_BINARY_HEADER_H_
#define NOVATEL_PARSER_BINARY_HEADER_H_

#include <array>
#include <cstdint>
#include <limits>

namespace Novatel {

/**
 * The fields contained in a short Novatel binary header
 * according to Chapter 1.4 of
 * https://docs.novatel.com/oem7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
 */
struct BinaryHeader {
  uint16_t message_len;  // short version is 1 byte, regular version is 2 bytes
  uint16_t message_id;
  uint16_t week;
  int32_t ms;

  void read(const uint8_t *data);
};

typedef struct BinaryHeader BinaryHeaderShort;

/**
 * The fields contained in a regular Novatel binary header
 * according to Chapter 1.3 of
 * https://docs.novatel.com/oem7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
 */
struct BinaryHeaderRegular : public BinaryHeader {
  uint8_t header_len;
  uint8_t message_type;
  uint8_t port_addr;
  uint16_t sequence;
  uint8_t idle_time;
  uint8_t time_status;
  uint32_t rx_status;
  uint16_t reserved;
  uint16_t rx_software_version;

  void read(const uint8_t *data);
};

}  // namespace Novatel

#endif
