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
#include <libsbp/v4/sbas.h>
#include <libsbp/v4/sbp_msg.h>

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
                                         void *context));

#endif /* #ifndef GNSS_CONVERTERS_SBAS_H */
