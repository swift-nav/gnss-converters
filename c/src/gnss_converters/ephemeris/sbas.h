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

#ifndef GNSS_CONVERTERS_SBAS_H
#define GNSS_CONVERTERS_SBAS_H

#include <gnss-converters/eph_sat_data.h>
#include <libsbp/sbas.h>

void sbas_decode_subframe(
    struct eph_sat_data *data,
    uint32_t tow_ms,
    int prn,
    const u32 words[],
    int sz,
    u16 sender_id,
    void *context,
    void (*sbp_cb)(u16 msg_id, u8 length, u8 *buf, u16 sender_id, void *ctx));

#endif /* #ifndef GNSS_CONVERTERS_SBAS_H */
