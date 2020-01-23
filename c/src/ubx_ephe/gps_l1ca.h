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

#ifndef GNSS_CONVERTERS_GPS_L1CA_H
#define GNSS_CONVERTERS_GPS_L1CA_H

void gps_l1ca_init(void (*cb_ubx_to_sbp)(u16 msg_id,
                                         u8 length,
                                         u8 *buff,
                                         u16 sender_id,
                                         void *context),
                   void *context,
                   u16 sender_id);
void gps_l1ca_decode_subframe(int prn, const u32 subframe[], int sz);

#endif /* #ifndef GNSS_CONVERTERS_GPS_L1CA_H */
