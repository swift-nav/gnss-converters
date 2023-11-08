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

#include "binary_header.h"

#include "read_little_endian.h"

namespace Novatel {

void BinaryHeader::read(const uint8_t *data) {
  message_len = *data++;
  message_id = Util::read_le_uint16(data);
  data += sizeof(uint16_t);
  week = Util::read_le_uint16(data);
  data += sizeof(uint16_t);
  ms = Util::read_le_int32(data);
  /* next woud be: data += sizeof(int32_t); */
}

void BinaryHeaderRegular::read(const uint8_t *data) {
  message_id = Util::read_le_uint16(data);
  data += sizeof(uint16_t);
  message_type = *data++;
  port_addr = *data++;
  message_len = Util::read_le_uint16(data);
  data += sizeof(uint16_t);
  sequence = Util::read_le_uint16(data);
  data += sizeof(uint16_t);
  idle_time = *data++;
  time_status = *data++;
  week = Util::read_le_uint16(data);
  data += sizeof(uint16_t);
  ms = Util::read_le_int32(data);
  data += sizeof(int32_t);
  rx_status = Util::read_le_uint32(data);
  data += sizeof(uint32_t);
  reserved = Util::read_le_uint16(data);
  data += sizeof(uint16_t);
  rx_software_version = Util::read_le_uint16(data);
  /* next would be: data += sizeof(uint16_t); */
}

}  // namespace Novatel
