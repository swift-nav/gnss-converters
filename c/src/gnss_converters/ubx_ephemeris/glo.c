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
#include <libsbp/observation.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_capabilities.h>
#include <swiftnav/signal.h>

#include "common.h"

#define GLO_MAX_STRING_NUM 5

static void invalidate_strings(struct glo_sat_data *sat, unsigned mask) {
  assert(sat);
  sat->vmask &= ~mask;
}

/**
 * Packs Glonass ephemeris data.
 * @param e the ephemeris to pack.
 * @param m the packed ephemeris.
 */
static void pack_ephemeris_glo(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_glo_t *k = &e->glo;
  msg_ephemeris_glo_t *msg = &m->glo;
  pack_ephemeris_common_content(e, &msg->common);
  msg->pos[0] = k->pos[0];
  msg->pos[1] = k->pos[1];
  msg->pos[2] = k->pos[2];
  msg->vel[0] = k->vel[0];
  msg->vel[1] = k->vel[1];
  msg->vel[2] = k->vel[2];
  msg->acc[0] = (float)k->acc[0];
  msg->acc[1] = (float)k->acc[1];
  msg->acc[2] = (float)k->acc[2];
  msg->gamma = (float)k->gamma;
  msg->tau = (float)k->tau;
  msg->d_tau = (float)k->d_tau;
  msg->iod = k->iod;
  msg->fcn = k->fcn;
}

/** strings 1,2,3,4,5 are required */
#define ALL_STRINGS_MASK 0x1FU

static u8 reverse_byte(u8 b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

/**
 * Decodes GLO pages 1-5.
 * Reference: GLO OS SIS ICD, Issue 1.3, December 2016
 * @param data context data
 * @param prn satellite identifier of the sender
 * @param fcn frequency slot of the sender [0..13]
 * @param words the array of full page words (32 bit words)
 * @param sz must be 4 (number of 32 bit words per one GLO page)
 */
void glo_decode_string(
    struct ubx_sbp_state *data, int prn, int fcn, const u32 words[], int sz) {
  if (UBX_UNKNOWN_PRN == prn) {
    /* PRN not yet known, no use decoding the rest */
    return;
  }

  assert(data);
  assert(words);
  if (prn < GLO_FIRST_PRN || prn >= (GLO_FIRST_PRN + NUM_SATS_GLO) || 4 != sz) {
    return;
  }

  int idle = (words[0] >> 31U) & 1U;
  if (0 != idle) {
    return;
  }

  u8 string_num = (words[0] >> 27U) & 0xFU;
  if (string_num > GLO_MAX_STRING_NUM) {
    /* only strings 1-5 needed for ephemeris */
    return;
  }

  struct glo_sat_data *sat = &data->eph_data.glo_sat_data[prn - 1];
  if (sat->curr_superframe_id != (words[3] >> 16)) {
    sat->vmask = 0;
    sat->curr_superframe_id = (words[3] >> 16);
  }
  sat->vmask |= 1U << (u8)(string_num - 1);
  assert(string_num <= (int)ARRAY_SIZE(sat->string));
  memcpy(&sat->string[string_num - 1].words,
         words,
         sizeof(sat->string[string_num - 1].words));
  if (ALL_STRINGS_MASK != (sat->vmask & ALL_STRINGS_MASK)) {
    return; /* not all word types 1,2,3,4,5 are available yet */
  }

  /* do not proceed to decoding until leap second is known */
  if (!data->leap_second_known) {
    return;
  }

  /* all pages collected, rearrange the strings byte and bit order to what the
   * decoder expects */

  glo_string_t page[GLO_MAX_STRING_NUM];
  assert(ARRAY_SIZE(page) == ARRAY_SIZE(sat->string));
  memset(&page, 0, sizeof(page));

  for (int i = 0; i < (int)ARRAY_SIZE(page); i++) {
    u8 *p = (u8 *)&page[i].word[0];
    u32 *w = &sat->string[i].words[0];

    /* swap byte endianness */
    int k = 0;
    for (int j = 0; j < GLO_NAV_STR_WORDS; j++) {
      p[k++] = (w[j] >> 24U) & 0xFFU;
      p[k++] = (w[j] >> 16U) & 0xFFU;
      p[k++] = (w[j] >> 8U) & 0xFFU;
      p[k++] = w[j] & 0xFFU;
    }

    /* reverse the buffer */
    for (k = 0; k <= (GLO_NAV_STR_BITS >> 1); k++) {
      u8 swap = getbitu(p, k, 1);
      setbitu(p, k, 1, getbitu(p, GLO_NAV_STR_BITS - 1 - k, 1));
      setbitu(p, GLO_NAV_STR_BITS - 1 - k, 1, swap);
    }

    /* restore the bit endianness */
    for (k = 0; k <= (GLO_NAV_STR_BITS >> 3); k++) {
      p[k] = reverse_byte(p[k]);
    }
  }

  /* Ephemeris decoded from GLO L1CA is same as the one decoded
     from GLO L2CA */
  gnss_signal_t sid = {prn, CODE_GLO_L1OF};

  ephemeris_t e;
  memset(&e, 0, sizeof(e));
  decode_glo_ephemeris(page, sid, &data->utc_params, &e);

  /* SBP codes FCN as [1..14] */
  e.glo.fcn = fcn + 1;

  msg_ephemeris_t msg;
  pack_ephemeris_glo(&e, &msg);

  assert(data->cb_ubx_to_sbp);
  data->cb_ubx_to_sbp(SBP_MSG_EPHEMERIS_GLO,
                      (u8)sizeof(msg.glo),
                      (u8 *)&msg.glo,
                      data->sender_id,
                      data->context);
  invalidate_strings(sat, /*mask=*/ALL_STRINGS_MASK);
}
