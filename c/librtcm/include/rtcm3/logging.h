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

#ifndef SWIFTNAV_RTCM3_LOGGING_H
#define SWIFTNAV_RTCM3_LOGGING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef LIBRTCM_LOG_INTERNAL
/*
 * from syslog.h
 */
#define LOG_EMERG 0   /* system is unusable */
#define LOG_ALERT 1   /* action must be taken immediately */
#define LOG_CRIT 2    /* critical conditions */
#define LOG_ERR 3     /* error conditions */
#define LOG_WARNING 4 /* warning conditions */
#define LOG_NOTICE 5  /* normal but significant condition */
#define LOG_INFO 6    /* informational */
#define LOG_DEBUG 7   /* debug-level messages */

#endif /* RCTM_LOG_INTERNAL */

typedef void (*rtcm_log_callback)(uint8_t level,
                                  uint8_t *msg,
                                  uint16_t len,
                                  void *context);

void rtcm_init_logging(rtcm_log_callback callback, void *context);
void rtcm_log(uint8_t level, uint8_t *msg, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_RTCM3_LOGGING_H */
