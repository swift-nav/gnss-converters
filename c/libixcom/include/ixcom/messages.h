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

#ifndef SWIFTNAV_IXCOM_MESSAGES_H
#define SWIFTNAV_IXCOM_MESSAGES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ixcom/XCOMdat.h>

size_t ixcom_set_bytes(const uint8_t *src, uint8_t *dest, size_t num_bytes);

/* return codes for the decoders */
typedef enum ixcom_rc_e {
  IXCOM_RC_OK = 0,
  IXCOM_RC_MESSAGE_TYPE_MISMATCH = -1,
  IXCOM_RC_INVALID_MESSAGE = -2
} ixcom_rc;

#endif /* SWIFTNAV_IXCOM_MESSAGES_H */
