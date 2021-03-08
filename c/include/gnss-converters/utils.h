/*
 * Copyright (C) 2013-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_UTILS_H
#define SWIFTNAV_UTILS_H

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STRNCPY(DST, SRC)                   \
  do {                                      \
    (DST)[sizeof(DST) - 1] = '\0';          \
    strncpy((DST), (SRC), sizeof(DST) - 1); \
    if ((DST)[sizeof(DST) - 1] != '\0') {   \
      (DST)[sizeof(DST) - 1] = '\0';        \
    }                                       \
  } while (false)

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_UTILS_H */
