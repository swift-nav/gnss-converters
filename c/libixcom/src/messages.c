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

#include <string.h>

#include <ixcom/messages.h>

/* iXCOM protocol is little-endian */

bool is_little_endian() {
  uint32_t i = 1;
  return ((*(char *)&i) != 0);
}

size_t ixcom_set_bytes(const uint8_t *src, uint8_t *dest, size_t num_bytes) {
  if (is_little_endian()) {
    memcpy(dest, src, num_bytes);
  } else {
    for (size_t i = 0; i < num_bytes; i++) {
      dest[i] = src[num_bytes - i - 1];
    }
  }

  return num_bytes;
}
