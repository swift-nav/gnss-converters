/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_IXCOM_DECODE_H
#define SWIFTNAV_IXCOM_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ixcom/messages.h>

uint16_t ixcom_checksum(const uint8_t buff[], size_t length);
ixcom_rc ixcom_decode_header(const uint8_t buff[], XCOMHeader *header);
ixcom_rc ixcom_decode_footer(const uint8_t buff[], XCOMFooter *footer);

ixcom_rc ixcom_decode_imuraw(const uint8_t buff[], XCOMmsg_IMURAW *msg_imuraw);
ixcom_rc ixcom_decode_wheeldata(const uint8_t buff[],
                                XCOMmsg_WHEELDATA *msg_wheeldata);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_IXCOM_DECODE_H */
