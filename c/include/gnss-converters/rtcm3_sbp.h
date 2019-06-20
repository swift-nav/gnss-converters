/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H
#define GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H

#include <libsbp/observation.h>
#include <rtcm3/messages.h>
#include <swiftnav/fifo_byte.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif

/* This is the maximum number of SBP observations possible per epoch:
   - Max number of observation messages comes from the 4 bits assigned to the
     sequence count in header.n_obs
   - The number of observations per message comes from the max 255 byte
     message length
*/
#define SBP_FRAMING_MAX_PAYLOAD_SIZE (255u)
#define SBP_HDR_SIZE (sizeof(observation_header_t))
#define SBP_OBS_SIZE (sizeof(packed_obs_content_t))
#define SBP_MAX_OBS_SEQ (15u)
#define MAX_OBS_IN_SBP \
  ((SBP_FRAMING_MAX_PAYLOAD_SIZE - SBP_HDR_SIZE) / SBP_OBS_SIZE)
#define MAX_OBS_PER_EPOCH (SBP_MAX_OBS_SEQ * MAX_OBS_IN_SBP)
#define OBS_BUFFER_SIZE (SBP_HDR_SIZE + MAX_OBS_PER_EPOCH * SBP_OBS_SIZE)

#define INVALID_TIME 0xFFFF
#define MAX_WN (INT16_MAX)

#define RTCM3_MSG_OVERHEAD 6
#define RTCM3_MAX_MSG_LEN 1023
#define RTCM3_FIFO_SIZE 4096
#define RTCM3_BUFFER_SIZE \
  (RTCM3_FIFO_SIZE - RTCM3_MSG_OVERHEAD - RTCM3_MAX_MSG_LEN)

#define IS_POWER_OF_TWO(x) (0 == ((x) & ((x)-1)))

typedef enum {
  UNSUPPORTED_CODE_UNKNOWN = 0u,
  UNSUPPORTED_CODE_GLO_L1P,
  UNSUPPORTED_CODE_GLO_L2P,
  UNSUPPORTED_CODE_MAX
} unsupported_code_t;

/* A struct for storing either an SSR orbit correction message
or an SSR clock correction message. Used for combining the
separate messages into a combined message */
typedef struct {
  union {
    rtcm_msg_clock clock;
    rtcm_msg_orbit orbit;
  };
  /* Only one of these can be true at once (or neither) */
  bool contains_clock; /* True if clock contains data */
  bool contains_orbit; /* True if orbit contains data */
} ssr_orbit_clock_cache;

struct rtcm3_sbp_state {
  gps_time_t time_from_rover_obs;
  s8 leap_seconds;
  bool leap_second_known;
  u16 sender_id;
  gps_time_t last_gps_time;
  gps_time_t last_glo_time;
  gps_time_t last_1230_received;
  gps_time_t last_msm_received;
  void (*cb_rtcm_to_sbp)(
      u16 msg_id, u8 len, u8 *buff, u16 sender_id, void *context);
  void (*cb_base_obs_invalid)(double time_diff, void *context);
  void *context;
  u8 obs_buffer[OBS_BUFFER_SIZE];
  bool sent_msm_warning;
  bool sent_code_warning[UNSUPPORTED_CODE_MAX];
  /* GLO FCN map, indexed by 1-based PRN */
  u8 glo_sv_id_fcn_map[NUM_SATS_GLO + 1];
  /* The cache for storing the first message before combining the separate
  orbit and clock messages into a combined SBP message */
  ssr_orbit_clock_cache orbit_clock_cache[CONSTELLATION_COUNT];
  uint8_t fifo_buf[RTCM3_FIFO_SIZE];
  fifo_t fifo;
};

void rtcm2sbp_decode_frame(const uint8_t *frame,
                           uint32_t frame_length,
                           struct rtcm3_sbp_state *state);

void rtcm2sbp_decode_payload(const uint8_t *payload,
                             uint32_t payload_length,
                             struct rtcm3_sbp_state *state);

void rtcm2sbp_set_gps_time(const gps_time_t *current_time,
                           struct rtcm3_sbp_state *state);

void rtcm2sbp_set_leap_second(s8 leap_seconds, struct rtcm3_sbp_state *state);

void rtcm2sbp_set_glo_fcn(sbp_gnss_signal_t sid,
                          u8 sbp_fcn,
                          struct rtcm3_sbp_state *state);

void rtcm2sbp_init(struct rtcm3_sbp_state *state,
                   void (*cb_rtcm_to_sbp)(u16 msg_id,
                                          u8 length,
                                          u8 *buffer,
                                          u16 sender_id,
                                          void *context),
                   void (*cb_base_obs_invalid)(double time_diff, void *context),
                   void *context);

int rtcm2sbp_process(struct rtcm3_sbp_state *state,
                     int (*read_stream_func)(uint8_t *buf,
                                             size_t len,
                                             void *context));

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H */
