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
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_capabilities.h>
#include <swiftnav/signal.h>

#include "common.h"

static void pack_ephemeris_bds(const ephemeris_t *e, msg_ephemeris_t *m) {
  const ephemeris_kepler_t *k = &e->kepler;
  msg_ephemeris_bds_t *msg = &m->bds;
  pack_ephemeris_common_content(e, &msg->common);
  msg->tgd1 = k->tgd.bds_s[0];
  msg->tgd2 = k->tgd.bds_s[1];
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
}

/**
 * Decodes BDS D1 subframes.
 * Reference: BDS-SIS-ICD-2.1 (2016-11)
 * @param data context data
 * @param prn transmitter's PRN
 * @param subframe the array of full subframe (30 bit words)
 * @param sz must be 10 (number of 30 bit words per one BDS D1 subframe)
 */
void bds_decode_subframe(struct ubx_sbp_state *data,
                         int prn,
                         const u32 words[],
                         int sz) {
  assert(data);
  assert(words);
  if (prn < BDS_FIRST_PRN || prn >= (BDS_FIRST_PRN + NUM_SATS_BDS) ||
      10 != sz) {
    return;
  }

  int geo = (0 != (GNSS_CAPB_BDS_D2NAV & ((u64)1 << (prn - BDS_FIRST_PRN))));
  if (geo) {
    /* BDS GEO sends D2 nav messages, we only decode D1 nav messages here */
    return;
  }

  /* subframe identification (FraID) */
  int fraid = ((words[0] >> 12U) & 0x7U) - 1;
  if ((fraid < 0) || (fraid > 2)) {
    return; /* only SF 1,2,3 contain ephemeris data */
  }

  struct sat_data *sat = &data->eph_data.bds_sat_data[prn - 1];
  sat->vmask |= 1U << fraid;
  memcpy(&sat->sf[fraid].words, words, sz * sizeof(words[0]));
  if (0x7 != (sat->vmask & 0x7U)) {
    return; /* not all SF 1,2,3 are available yet */
  }

  u32 sow[3];
  for (int i = 0; i < 3; i++) {
    sow[i] = ((sat->sf[i].words[0] >> 4U) & 0xFFU) << 12U;
    sow[i] |= (sat->sf[i].words[1] >> 18U) & 0xFFFU;
    if (sow[i] >= WEEK_SECS) {
      invalidate_subframes(sat, /*mask=*/1U << i);
      return;
    }
  }

  if ((sow[0] != (sow[1] - 6)) || (sow[1] != (sow[2] - 6))) {
    return; /* received subframes are not in sequence */
  }

  /* Now let's actually decode the ephemeris... */

  u32 fraid_words[3][10];
  for (int i = 0; i < 3; i++) {
    memcpy(&fraid_words[i][0], &sat->sf[i].words[0], 10 * sizeof(u32));
  }

  ephemeris_t e;
  memset(&e, 0, sizeof(e));
  gnss_signal_t sid = {.sat = prn, .code = CODE_BDS2_B1};
  decode_bds_d1_ephemeris(fraid_words, sid, &e);

  ephemeris_kepler_t *k = &e.kepler;

  add_secs(&e.toe, BDS_SECOND_TO_GPS_SECOND);
  add_secs(&k->toc, BDS_SECOND_TO_GPS_SECOND);
  e.fit_interval = BDS_FIT_INTERVAL_SECONDS;
  e.valid = 1;

  msg_ephemeris_t msg;
  memset(&msg, 0, sizeof(msg));
  pack_ephemeris_bds(&e, &msg);

  assert(data->cb_ubx_to_sbp);
  data->cb_ubx_to_sbp(SBP_MSG_EPHEMERIS_BDS,
                      (u8)sizeof(msg.bds),
                      (u8 *)&msg.bds,
                      data->sender_id,
                      data->context);
  invalidate_subframes(sat, /*mask=*/7);
}
