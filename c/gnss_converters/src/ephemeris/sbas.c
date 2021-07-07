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

#include <gnss-converters/ubx_sbp.h>
#include <libsbp/v4/gnss.h>
#include <libsbp/v4/sbas.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/common.h>
#include <swiftnav/edc.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

#include "common.h"

/** SBAS preamble 1 0x53 */
#define SBAS_PREAMBLE1 (0x53u)
/** SBAS preamble 2 0x9a */
#define SBAS_PREAMBLE2 (0x9au)
/** SBAS preamble 3 0xc6 */
#define SBAS_PREAMBLE3 (0xc6u)
/** SBAS L1 message length [bits] */
#define SBAS_MSG_LENGTH (250)
/** SBAS preamble length in bits */
#define SBAS_PREAMBLE_LENGTH (8)
/** SBAS msg id length in bits */
#define SBAS_MSG_ID_LENGTH (6)
/** SBAS CRC length in bits */
#define SBAS_MSG_CRC_LENGTH (24)
/** SBAS message payload length in bits */
#define SBAS_MSG_DATA_LENGTH (SBAS_MSG_LENGTH - SBAS_MSG_CRC_LENGTH)
/** SBAS data payload length in bits */
#define SBAS_MSG_PAYLOAD_LENGTH \
  (SBAS_MSG_DATA_LENGTH - SBAS_PREAMBLE_LENGTH - SBAS_MSG_ID_LENGTH)

static void pack_sbas_data(const u8 *buffer,
                           u8 prn,
                           u32 tow_ms,
                           u8 message_type,
                           sbp_msg_sbas_raw_t *msg) {
  msg->sid.code = CODE_SBAS_L1CA;
  msg->sid.sat = prn;
  msg->tow = tow_ms;
  msg->message_type = message_type;

  /* copy the data payload into the message */
  bitcopy(msg->data,
          0,
          buffer,
          SBAS_PREAMBLE_LENGTH + SBAS_MSG_ID_LENGTH,
          SBAS_MSG_PAYLOAD_LENGTH);
}

/**
 * Decodes SBAS L1CA subframes.
 * @param data context data
 * @param prn transmitter's PRN
 * @param subframe the array of full subframe (32 bit words)
 * @param sz must be 8 (number of 32 bit words per one SBAS message) or 9
 * (workaround for an alleged bug on u-blox F9 series receivers)
 */
void sbas_decode_subframe(struct eph_sat_data *data,
                          uint32_t tow_ms,
                          int prn,
                          const u32 words[],
                          int sz,
                          u16 sender_id,
                          void *context,
                          void (*sbp_cb)(uint16_t sender_id,
                                         sbp_msg_type_t msg_type,
                                         const sbp_msg_t *msg,
                                         void *context)) {
  (void)data;
  assert(words);
  if (prn < SBAS_FIRST_PRN || prn >= (SBAS_FIRST_PRN + NUM_SATS_SBAS) ||
      (8 != sz && 9 != sz)) {
    return;
  }

  const int kNumberOfWords = 8;
  u8 buffer[32];

  /* copy the input word buffer into byte buffer, swapping byte endianness */
  for (int i = 0; i < kNumberOfWords; i++) {
    buffer[i * 4] = (words[i] >> 24U) & 0xFFU;
    buffer[i * 4 + 1] = (words[i] >> 16U) & 0xFFU;
    buffer[i * 4 + 2] = (words[i] >> 8U) & 0xFFU;
    buffer[i * 4 + 3] = words[i] & 0xFFU;
  }

  /* SBAS preamble */
  u8 preamble = buffer[0];
  if (SBAS_PREAMBLE1 != preamble && SBAS_PREAMBLE2 != preamble &&
      SBAS_PREAMBLE3 != preamble) {
    return;
  }

  /* SBAS message type */
  u8 message_type = (buffer[1] >> 2) & 0x3FU;

  /* verify SBAS CRC */
  u32 computed_crc =
      crc24q_bits(0, buffer, SBAS_MSG_DATA_LENGTH, /* invert_flag */ false);

  u32 crc = getbitu(buffer, SBAS_MSG_DATA_LENGTH, SBAS_MSG_CRC_LENGTH);

  if (crc != computed_crc) {
    log_info("Ignoring uBlox SBAS frame with CRC mismatch");
    return;
  }

  sbp_msg_t sbp_msg;
  sbp_msg_sbas_raw_t *msg = &sbp_msg.sbas_raw;
  memset(msg, 0, sizeof(*msg));
  pack_sbas_data(buffer, prn, tow_ms, message_type, msg);

  assert(sbp_cb);
  sbp_cb(sender_id, SbpMsgSbasRaw, &sbp_msg, context);
}
