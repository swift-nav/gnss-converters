/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <swiftnav/bits.h>

#include <ixcom/encode.h>

/* Serialize the iXCOM Header
 *
 * \param header iXCOM Header to serialize
 * \param buff Outgoing data buffer
 * \return number of bytes serialized
 */
size_t ixcom_encode_header(XCOMHeader *header, uint8_t buff[]) {
  assert(header);

  size_t byte_offset = 0;
  byte_offset += ixcom_set_bytes(&header->sync, buff + byte_offset, 1);
  byte_offset += ixcom_set_bytes(&header->msg_id, buff + byte_offset, 1);
  byte_offset += ixcom_set_bytes(&header->frame_counter, buff + byte_offset, 1);
  byte_offset +=
      ixcom_set_bytes(&header->trigger_source, buff + byte_offset, 1);
  byte_offset +=
      ixcom_set_bytes((uint8_t *)&header->msg_len, buff + byte_offset, 2);
  byte_offset +=
      ixcom_set_bytes((uint8_t *)&header->gps_week, buff + byte_offset, 2);
  byte_offset +=
      ixcom_set_bytes((uint8_t *)&header->gps_time_sec, buff + byte_offset, 4);
  byte_offset +=
      ixcom_set_bytes((uint8_t *)&header->gps_time_usec, buff + byte_offset, 4);

  return byte_offset;
}

size_t ixcom_encode_footer(XCOMFooter *footer, uint8_t buff[]) {
  assert(footer);

  size_t byte_offset = 0;
  // clang-format off
  byte_offset += ixcom_set_bytes(
      (uint8_t *)&footer->global_status.value, buff + byte_offset, 2);
  byte_offset +=
      ixcom_set_bytes((uint8_t *)&footer->crc16, buff + byte_offset, 2);
  // clang-format on
  return byte_offset;
}

size_t ixcom_encode_imuraw(XCOMmsg_IMURAW *msg_imuraw, uint8_t buff[]) {
  assert(msg_imuraw);

  size_t byte_offset = 0;
  byte_offset += ixcom_encode_header(&msg_imuraw->header, buff + byte_offset);
  for (int i = 0; i < 3; i++) {
    // clang-format off
    byte_offset += ixcom_set_bytes(
        (uint8_t *)(&msg_imuraw->acc[i]), buff + byte_offset, 4);
    // clang-format on
  }

  for (int i = 0; i < 3; i++) {
    // clang-format off
    byte_offset += ixcom_set_bytes(
        (uint8_t *)(&msg_imuraw->omg[i]), buff + byte_offset, 4);
    // clang-format on
  }

  byte_offset += ixcom_encode_footer(&msg_imuraw->footer, buff + byte_offset);

  return byte_offset;
}

size_t ixcom_encode_wheeldata(XCOMmsg_WHEELDATA *msg_wheeldata,
                              uint8_t buff[]) {
  assert(msg_wheeldata);

  size_t byte_offset = 0;
  byte_offset +=
      ixcom_encode_header(&msg_wheeldata->header, buff + byte_offset);
  // clang-format off
  byte_offset += ixcom_set_bytes(
      (uint8_t *)(&msg_wheeldata->speed), buff + byte_offset, 4);
  byte_offset += ixcom_set_bytes(
      (uint8_t *)(&msg_wheeldata->ticks), buff + byte_offset, 4);
  // clang-format on

  byte_offset +=
      ixcom_encode_footer(&msg_wheeldata->footer, buff + byte_offset);

  return byte_offset;
}
