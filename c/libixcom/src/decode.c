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

#include <ixcom/decode.h>

/* iXCOM is a little-endian protocol. This library assumes the host system is
 * also little-endian.
 * TODO(yizhe) make decoders endian-agnostic
 */

/** Pre-computed table for CRC algorithm.
 * Polynomial: x^16 + x^12 + x^5 + 1
 * Start value: 0
 */
static const uint16_t crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

/** Computes the crc16 checksum for a single byte. The checksum for a message is
 * calculated by calling this function sequentially on every byte in the
 * message.
 *
 * \param crc Previous computed crc value. Initialized to 0.
 * \param byte The byte to compute the crc with.
 * \return The new crc
 */
uint16_t crc_byte(const uint16_t crc, const uint16_t byte) {
  return (uint16_t)((crc << 8) ^ crc_table[(crc >> 8) ^ byte]);
}

/** Computes the crc16 checksum across a message. The checksum is calculated
 * across all data, starting from the SYNC byte, up to but not including the 16
 * bit CRC field.
 *
 * \param buff Buffer containing the iXCOM message
 * \param length Length of the iXCOM message
 * \return 16-bit checksum
 */
uint16_t ixcom_checksum(const uint8_t buff[], size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc = crc_byte(crc, buff[i]);
  }
  return crc;
}

ixcom_rc ixcom_decode_header(const uint8_t buff[], XCOMHeader *header) {
  assert(header);

  size_t byte_offset = 0;
  byte_offset += ixcom_set_bytes(buff + byte_offset, &header->sync, 1);

  if (header->sync != XCOM_SYNC_BYTE) {
    return IXCOM_RC_INVALID_MESSAGE;
  }

  byte_offset += ixcom_set_bytes(buff + byte_offset, &header->msg_id, 1);
  byte_offset += ixcom_set_bytes(buff + byte_offset, &header->frame_counter, 1);
  byte_offset +=
      ixcom_set_bytes(buff + byte_offset, &header->trigger_source, 1);
  byte_offset +=
      ixcom_set_bytes(buff + byte_offset, (uint8_t *)(&header->msg_len), 2);
  byte_offset +=
      ixcom_set_bytes(buff + byte_offset, (uint8_t *)(&header->gps_week), 2);
  // clang-format off
  byte_offset += ixcom_set_bytes(
      buff + byte_offset, (uint8_t *)(&header->gps_time_sec), 4);
  // NOLINTNEXTLINE
  byte_offset += ixcom_set_bytes(
      buff + byte_offset, (uint8_t *)(&header->gps_time_usec), 4);
  // clang-format on

  return IXCOM_RC_OK;
}

ixcom_rc ixcom_decode_footer(const uint8_t buff[], XCOMFooter *footer) {
  assert(footer);

  size_t byte_offset = 0;

  // clang-format off
  byte_offset += ixcom_set_bytes(
      buff + byte_offset, (uint8_t *)(&footer->global_status.value), 2);
  // clang-format on
  // NOLINTNEXTLINE
  byte_offset +=
      ixcom_set_bytes(buff + byte_offset, (uint8_t *)(&footer->crc16), 2);

  return IXCOM_RC_OK;
}

ixcom_rc ixcom_decode_imuraw(const uint8_t buff[], XCOMmsg_IMURAW *msg_imuraw) {
  assert(msg_imuraw);

  size_t byte_offset = 0;

  ixcom_rc ret = ixcom_decode_header(buff, &msg_imuraw->header);
  if (ret != IXCOM_RC_OK) {
    return ret;
  }
  byte_offset += sizeof(msg_imuraw->header);

  if (msg_imuraw->header.msg_id != XCOM_MSGID_IMURAW) {
    return IXCOM_RC_INVALID_MESSAGE;
  }

  for (int i = 0; i < 3; i++) {
    // clang-format off
    byte_offset += ixcom_set_bytes(
        buff + byte_offset, (uint8_t *)(&msg_imuraw->acc[i]), 4);
    // clang-format on
  }

  for (int i = 0; i < 3; i++) {
    // clang-format off
    byte_offset += ixcom_set_bytes(
        buff + byte_offset, (uint8_t *)(&msg_imuraw->omg[i]), 4);
    // clang-format on
  }

  ixcom_decode_footer(buff + byte_offset, &msg_imuraw->footer);
  // NOLINTNEXTLINE
  byte_offset += sizeof(msg_imuraw->footer);

  uint16_t checksum = ixcom_checksum(buff, msg_imuraw->header.msg_len - 2u);
  if (checksum != msg_imuraw->footer.crc16) {
    return IXCOM_RC_INVALID_MESSAGE;
  }

  return IXCOM_RC_OK;
}

ixcom_rc ixcom_decode_wheeldata(const uint8_t buff[],
                                XCOMmsg_WHEELDATA *msg_wheeldata) {
  assert(msg_wheeldata);

  size_t byte_offset = 0;

  ixcom_rc ret = ixcom_decode_header(buff, &msg_wheeldata->header);
  if (ret != IXCOM_RC_OK) {
    return ret;
  }
  byte_offset += sizeof(msg_wheeldata->header);

  if (msg_wheeldata->header.msg_id != XCOM_MSGID_WHEELDATA) {
    return IXCOM_RC_INVALID_MESSAGE;
  }

  // clang-format off
  byte_offset += ixcom_set_bytes(
      buff + byte_offset, (uint8_t *)(&msg_wheeldata->speed), 4);
  byte_offset += ixcom_set_bytes(
      buff + byte_offset, (uint8_t *)(&msg_wheeldata->ticks), 4);
  // clang-format on

  ixcom_decode_footer(buff + byte_offset, &msg_wheeldata->footer);
  // NOLINTNEXTLINE
  byte_offset += sizeof(msg_wheeldata->footer);

  uint16_t checksum = ixcom_checksum(buff, msg_wheeldata->header.msg_len - 2u);
  if (checksum != msg_wheeldata->footer.crc16) {
    return IXCOM_RC_INVALID_MESSAGE;
  }

  return IXCOM_RC_OK;
}
