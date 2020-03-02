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

static void invalidate_pages(struct gal_sat_data *sat, unsigned mask) {
  assert(sat);
  sat->vmask &= ~mask;
}

/**
 * Packs Galileo ephemeris data.
 * @param e the ephemeris to pack.
 * @param m the packed ephemeris.
 * @param src the GAL ephemeris source (0 - I/NAV, 1 - F/NAV)
 */
static void pack_ephemeris_gal(const ephemeris_t *e,
                               msg_ephemeris_t *m,
                               int src) {
  assert(e);
  assert(m);
  assert((0 == src) || (1 == src));

  memset(m, 0, sizeof(*m));

  const ephemeris_kepler_t *k = &e->kepler;
  msg_ephemeris_gal_t *msg = &m->gal;
  pack_ephemeris_common_content(e, &msg->common);
  msg->bgd_e1e5a = k->tgd.gal_s[0];
  msg->bgd_e1e5b = k->tgd.gal_s[1];
  msg->c_rs = (float)k->crs;
  msg->c_rc = (float)k->crc;
  msg->c_uc = (float)k->cuc;
  msg->c_us = (float)k->cus;
  msg->c_ic = (float)k->cic;
  msg->c_is = (float)k->cis;
  msg->dn = k->dn;
  msg->m0 = k->m0;
  msg->ecc = k->ecc;
  msg->sqrta = k->sqrta;
  msg->omega0 = k->omega0;
  msg->omegadot = k->omegadot;
  msg->w = k->w;
  msg->inc = k->inc;
  msg->inc_dot = k->inc_dot;
  msg->af0 = k->af0;
  msg->af1 = (float)k->af1;
  msg->af2 = (float)k->af2;
  msg->toc.tow = (u32)round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
  msg->source = (u8)src;
}

/** pages 1,2,3,4,5 are required */
#define ALL_PAGES_MASK 0x1FU

/**
 * Decodes GAL pages 1-5.
 * Reference: GAL OS SIS ICD, Issue 1.3, December 2016
 * @param data context data
 * @param prn transmitter's PRN
 * @param words the array of full page words (32 bit words)
 * @param sz must be >=8 (number of 32 bit words per one GAL page)
 */
void gal_decode_page(struct ubx_sbp_state *data,
                     int prn,
                     const u32 words[],
                     int sz) {
  assert(data);
  assert(prn >= GAL_FIRST_PRN);
  assert(prn < (GAL_FIRST_PRN + NUM_SATS_GAL));
  assert(words);
  assert(sz >= 8);

  int ptype = (words[0] >> 30) & 1U;
  if (1 == ptype) {
    return; /* skip alert messages */
  }

  if (((words[0] >> 31) & 1U) != 0) {
    return; /* must start with even */
  }
  if (((words[4] >> 31) & 1U) != 1) {
    return; /* must end with odd */
  }

  /* word type identification */
  int wtype = (words[0] >> 24U) & 0x3FU;
  if ((wtype < 1) || (wtype > 5)) {
    return; /* only word types 1,2,3,4,5 contain ephemeris data, WN and TOW */
  }

  struct gal_sat_data *sat = &data->gal_sat[prn - 1];
  sat->vmask |= 1U << (wtype - 1);
  assert(wtype <= (int)ARRAY_SIZE(sat->pg));
  memcpy(&sat->pg[wtype - 1].words, words, sizeof(sat->pg[wtype - 1].words));
  if (ALL_PAGES_MASK != (sat->vmask & ALL_PAGES_MASK)) {
    return; /* not all word types 1,2,3,4,5 are available yet */
  }

  int iod[4];
  for (int i = 0; i < (int)ARRAY_SIZE(iod); i++) {
    iod[i] = (words[0] >> 14U) & 0x3FFU;
  }

  for (int i = 0; i < (int)ARRAY_SIZE(iod); i++) {
    for (int j = i + 1; j < (int)ARRAY_SIZE(iod); j++) {
      if (iod[i] != iod[j]) {
        return; /* received pages are not from the same batch */
      }
    }
  }

  /* Now let's actually decode the ephemeris... */

  u8 page[5][GAL_INAV_CONTENT_BYTE];
  assert(ARRAY_SIZE(page) == ARRAY_SIZE(sat->pg));

  for (int i = 0; i < (int)ARRAY_SIZE(page); i++) {
    u8 *p = &page[i][0];
    u32 *w = &sat->pg[i].words[0];
    int k = 0;

    /* extract Word type and Data(122-17) from from the event part */
    for (int j = 0; j < 4; j++) {
      p[k++] = (w[j] >> 24U) & 0xFFU;
      p[k++] = (w[j] >> 16U) & 0xFFU;
      p[k++] = (w[j] >> 8U) & 0xFFU;
      p[k++] = w[j] & 0xFFU;
    }
    /* shift Even/odd and Page type out */
    bitshl(p, ARRAY_SIZE(page[i]), /*shift=*/2);

    /* extract Data(16-1) from the odd part */
    u16 tmp = (w[4] >> 14U) & 0xFFFFU;
    p[14] = (tmp >> 8U) & 0xFFU;
    p[15] = tmp & 0xFFU;
  }

  ephemeris_t e;
  memset(&e, 0, sizeof(e));
  decode_gal_ephemeris(page, &e);

  e.sid.code = CODE_GAL_E1B;
  e.valid = 1;

  msg_ephemeris_t msg;
  /* I/NAV message as per UBX-13003221-R17
     u-blox8 / u-blox M8 Receiver Description Manual */
  pack_ephemeris_gal(&e, &msg, /*src=*/0);

  assert(data->cb_ubx_to_sbp);
  data->cb_ubx_to_sbp(SBP_MSG_EPHEMERIS_GAL,
                      (u8)sizeof(msg.gal),
                      (u8 *)&msg.gal,
                      data->sender_id,
                      data->context);
  invalidate_pages(sat, /*mask=*/ALL_PAGES_MASK);
}
