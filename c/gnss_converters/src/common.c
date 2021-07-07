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

#include "common.h"

void pack_ephemeris_common_content(const ephemeris_t *e,
                                   sbp_ephemeris_common_content_t *common) {
  common->toe.tow = (u32)round(e->toe.tow);
  common->toe.wn = e->toe.wn;
  common->valid = e->valid;
  common->health_bits = e->health_bits;
  common->sid.code = e->sid.code;
  common->sid.sat = e->sid.sat;
  common->fit_interval = e->fit_interval;
  common->ura = e->ura;
}
