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
#include <libsbp/gnss.h>
#include <libsbp/sbas.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/common.h>
#include <swiftnav/edc.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

#include "common.h"

/** SBAS preamble 1 0x53 */
#define SBAS_PREAMBLE1 (0b01010011u)
/** SBAS preamble 2 0x9a */
#define SBAS_PREAMBLE2 (0b10011010u)
/** SBAS preamble 3 0xc6 */
#define SBAS_PREAMBLE3 (0b11000110u)
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
                           msg_sbas_raw_t *msg) {
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
 * @param sz must be 8 (number of 32 bit words per one SBAS message)
 */
void sbas_decode_subframe(struct ubx_sbp_state *data,
                          int prn,
                          const u32 words[],
                          int sz) {
  assert(data);
  assert(prn >= SBAS_FIRST_PRN);
  assert(prn < (SBAS_FIRST_PRN + NUM_SATS_SBAS));
  assert(8 == sz || 9 == sz);

  u8 buffer[32];

  /* copy the input word buffer into byte buffer, swapping byte endianness */
  for (int i = 0; i < sz; i++) {
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

  u32 tow_ms = data->last_tow_ms;
  if (tow_ms > WEEK_MS) {
    /* TOW not yet set */
    return;
  }

  /* Fill in the approximate time of transmission of the start of the SBAS
   * message. The best guess for it we have is the current solution time
   * minus 1 s (length of the message) minus 120 ms (~= 36000km / 3e8m/s). */
  tow_ms -= 1120;
  if (tow_ms < 0.0) {
    tow_ms += WEEK_MS;
  }

  msg_sbas_raw_t msg;
  memset(&msg, 0, sizeof(msg));
  pack_sbas_data(buffer, prn, tow_ms, message_type, &msg);

  assert(data->cb_ubx_to_sbp);
  data->cb_ubx_to_sbp(SBP_MSG_SBAS_RAW,
                      (u8)sizeof(msg),
                      (u8 *)&msg,
                      data->sender_id,
                      data->context);
}
