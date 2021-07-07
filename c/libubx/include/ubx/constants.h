/*
 * Copyright (C) 2017,2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_UBX_CONSTANTS_H
#define SWIFTNAV_UBX_CONSTANTS_H

#include <swiftnav/signal.h>

#define UBX_MAX_NUM_OBS 256

#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62

#define UBX_GNSS_ID_GPS 0
#define UBX_GNSS_ID_SBAS 1
#define UBX_GNSS_ID_GAL 2
#define UBX_GNSS_ID_BDS 3
#define UBX_GNSS_ID_IMES 4
#define UBX_GNSS_ID_QZSS 5
#define UBX_GNSS_ID_GLO 6

#endif /* SWIFTNAV_UBX_CONSTANTS_H */
