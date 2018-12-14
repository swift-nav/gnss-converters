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

#define SBP_GLO_FCN_OFFSET 8
#define SBP_GLO_FCN_UNKNOWN 0

#define RTCM3_PREAMBLE 0xD3
#define RTCM3_MSG_OVERHEAD 6
#define RTCM3_MAX_MSG_LEN 0x3FF

typedef enum {
  UNSUPPORTED_CODE_UNKNOWN = 0u,
  UNSUPPORTED_CODE_GLO_L1P,
  UNSUPPORTED_CODE_GLO_L2P,
  UNSUPPORTED_CODE_MAX
} unsupported_code_t;

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
};

struct rtcm3_out_state {
  s8 leap_seconds;
  bool leap_second_known;
  bool ant_known;
  void (*cb_sbp_to_rtcm)(u8 *buffer, u16 length, void *context);
  u16 sender_id;
  observation_header_t sbp_header;
  packed_obs_content_t sbp_obs_buffer[MAX_OBS_PER_EPOCH];
  u16 n_sbp_obs;
  void *context;
  /* GLO FCN map, indexed by 1-based PRN */
  u8 glo_sv_id_fcn_map[NUM_SATS_GLO + 1];

  /** RTCM OUT format options. */

  /* Note that while the specification does not forbid sending both the legacy
   * and MSM observations, it does not recommend it. */
  bool send_legacy_obs;
  bool send_msm_obs;
  msm_enum msm_type;

  double ant_height; /* Antenna height above ARP, meters */
  char ant_descriptor[RTCM_MAX_STRING_LEN];
  char rcv_descriptor[RTCM_MAX_STRING_LEN];
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

void sbp2rtcm_init(struct rtcm3_out_state *state,
                   void (*cb_sbp_to_rtcm)(u8 *buffer,
                                          u16 length,
                                          void *context),
                   void *context);

void sbp2rtcm_set_leap_second(s8 leap_seconds, struct rtcm3_out_state *state);

void sbp2rtcm_set_rtcm_out_mode(msm_enum value, struct rtcm3_out_state *state);

void sbp2rtcm_set_glo_fcn(sbp_gnss_signal_t sid,
                          u8 sbp_fcn,
                          struct rtcm3_out_state *state);

bool sbp2rtcm_set_ant_height(double ant_height, struct rtcm3_out_state *state);

void sbp2rtcm_set_rcv_ant_descriptors(const char *ant_descriptor,
                                      const char *rcv_descriptor,
                                      struct rtcm3_out_state *state);

void sbp2rtcm_base_pos_ecef_cb(const u16 sender_id,
                               const u8 len,
                               const u8 msg[],
                               struct rtcm3_out_state *state);

void sbp2rtcm_glo_biases_cb(const u16 sender_id,
                            const u8 len,
                            const u8 msg[],
                            struct rtcm3_out_state *state);

void sbp2rtcm_sbp_obs_cb(const u16 sender_id,
                         const u8 len,
                         const u8 msg[],
                         struct rtcm3_out_state *state);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H */
