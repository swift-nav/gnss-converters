/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdlib.h>

#include <rtcm3/logging.h>

static rtcm_log_callback log_callback_ = NULL;
static void *log_context_ = NULL;

void rtcm_init_logging(rtcm_log_callback callback, void *context) {
  log_callback_ = callback;
  log_context_ = context;
}

void rtcm_log(uint8_t level, uint8_t *msg, uint16_t len) {
  if (log_callback_ != NULL) {
    log_callback_(level, msg, len, log_context_);
  }
}
