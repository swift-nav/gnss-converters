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

#ifndef SWIFTNAV_IXCOM_ENCODE_H
#define SWIFTNAV_IXCOM_ENCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ixcom/messages.h>

size_t ixcom_encode_header(XCOMHeader *header, uint8_t buff[]);
size_t ixcom_encode_footer(XCOMFooter *footer, uint8_t buff[]);

size_t ixcom_encode_imuraw(XCOMmsg_IMURAW *msg_imuraw, uint8_t buff[]);
size_t ixcom_encode_wheeldata(XCOMmsg_WHEELDATA *msg_wheeldata, uint8_t buff[]);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_IXCOM_ENCODE_H */
