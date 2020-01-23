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

#include "gnss-converters/ubx_sbp.h"
#include "gps_l1ca.h"
#include "swiftnav/common.h"
#include "ubx/decode.h"

#define UBX_GNSS_ID_GPS 0
#define UBX_GNSS_ID_GAL 2
#define UBX_GNSS_ID_BDS 3
#define UBX_GNSS_ID_GLO 6

void ubx_ephe_init(const struct ubx_sbp_state *state) {
  assert(state);
  gps_l1ca_init(state->cb_ubx_to_sbp, state->context, state->sender_id);
}

void ubx_ephe_handle_rxm_sfrbx(const struct ubx_sbp_state *state,
                               u8 *buf,
                               int sz) {
  assert(state);
  assert(buf);
  assert(sz > 0);

  ubx_rxm_sfrbx sfrbx;
  if (ubx_decode_rxm_sfrbx(buf, &sfrbx) != RC_OK) {
    return;
  }

  if (UBX_GNSS_ID_GPS == sfrbx.gnss_id) {
    u8 prn = sfrbx.sat_id;
    gps_l1ca_decode_subframe(prn, sfrbx.data_words, sfrbx.num_words);
  }
}
