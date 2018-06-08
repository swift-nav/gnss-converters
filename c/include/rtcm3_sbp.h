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

#include <libsbp/gnss.h>
#include <libsbp/logging.h>
#include <libsbp/observation.h>

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

/* MAX valid value (ms) for GPS is 604799999 and GLO is 86401999 */
#define INVALID_TIME 0xFFFF

#define SBP_GLO_FCN_OFFSET 8
#define SBP_GLO_FCN_UNKNOWN 0

struct rtcm3_sbp_state {
  gps_time_sec_t time_from_rover_obs;
  bool gps_time_updated;
  s8 leap_seconds;
  bool leap_second_known;
  u16 sender_id;
  gps_time_sec_t last_gps_time;
  gps_time_sec_t last_glo_time;
  gps_time_sec_t last_1230_received;
  gps_time_sec_t last_msm_received;
  void (*cb_rtcm_to_sbp)(u16 msg_id, u8 len, u8 *buff, u16 sender_id);
  void (*cb_base_obs_invalid)(double time_diff);
  u8 obs_buffer[OBS_BUFFER_SIZE];
  bool sent_msm_warning;
  /* GLO FCN map, indexed by 1-based PRN */
  u8 glo_sv_id_fcn_map[GLO_LAST_PRN + 1];
};

void rtcm2sbp_decode_frame(const uint8_t *frame,
                           uint32_t frame_length,
                           struct rtcm3_sbp_state *state);

void rtcm2sbp_set_gps_time(gps_time_sec_t *current_time,
                           struct rtcm3_sbp_state *state);

void rtcm2sbp_set_leap_second(s8 leap_seconds, struct rtcm3_sbp_state *state);

void rtcm2sbp_set_glo_fcn(sbp_gnss_signal_t sid,
                          u8 fcn,
                          struct rtcm3_sbp_state *state);

void rtcm2sbp_init(
    struct rtcm3_sbp_state *state,
    void (*cb_rtcm_to_sbp)(u16 msg_id, u8 length, u8 *buffer, u16 sender_id),
    void (*cb_base_obs_invalid)(double time_diff));

#endif /* GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H */
