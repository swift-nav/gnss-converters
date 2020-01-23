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

#ifndef GNSS_CONVERTERS_UBX_EPHE_H
#define GNSS_CONVERTERS_UBX_EPHE_H

void ubx_ephe_init(const struct ubx_sbp_state *state);
void ubx_ephe_handle_rxm_sfrbx(const struct ubx_sbp_state *state,
                               u8 *buf,
                               int sz);

#endif /* #ifndef GNSS_CONVERTERS_UBX_EPHE_H */
