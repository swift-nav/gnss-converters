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

#include "gnss-converters/rtcm3_sbp.h"
#include "rtcm3_sbp_internal.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libsbp/gnss.h>
#include <libsbp/logging.h>
#include <rtcm3/bits.h>
#include <rtcm3/decode.h>
#include <rtcm3/encode.h>
#include <rtcm3/eph_decode.h>
#include <rtcm3/logging.h>
#include <rtcm3/ssr_decode.h>
#include <swiftnav/edc.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/sid_set.h>
#include <swiftnav/signal.h>

#include "rtcm3_msm_utils.h"

static void validate_base_obs_sanity(struct rtcm3_sbp_state *state,
                                     const gps_time_t *obs_time,
                                     const gps_time_t *rover_time);
static void rtcm2sbp_set_leap_second_from_wn(u16 wn_ref,
                                             struct rtcm3_sbp_state *state);

void rtcm2sbp_init(struct rtcm3_sbp_state *state,
                   void (*cb_rtcm_to_sbp)(u16 msg_id,
                                          u8 length,
                                          u8 *buffer,
                                          u16 sender_id,
                                          void *context),
                   void (*cb_base_obs_invalid)(double timediff, void *context),
                   void *context) {
  state->time_from_rover_obs.wn = INVALID_TIME;
  state->time_from_rover_obs.tow = 0;

  state->leap_seconds = 0;
  state->leap_second_known = false;

  state->sender_id = 0;
  state->cb_rtcm_to_sbp = cb_rtcm_to_sbp;
  state->cb_base_obs_invalid = cb_base_obs_invalid;
  state->context = context;

  state->last_gps_time.wn = INVALID_TIME;
  state->last_gps_time.tow = 0;
  state->last_glo_time.wn = INVALID_TIME;
  state->last_glo_time.tow = 0;
  state->last_1230_received.wn = INVALID_TIME;
  state->last_1230_received.tow = 0;
  state->last_msm_received.wn = INVALID_TIME;
  state->last_msm_received.tow = 0;

  state->sent_msm_warning = false;
  for (u8 i = 0; i < UNSUPPORTED_CODE_MAX; i++) {
    state->sent_code_warning[i] = false;
  }

  for (u8 i = 0; i < sizeof(state->glo_sv_id_fcn_map); i++) {
    state->glo_sv_id_fcn_map[i] = MSM_GLO_FCN_UNKNOWN;
  }

  memset(state->obs_buffer, 0, OBS_BUFFER_SIZE);

  rtcm_init_logging(&rtcm_log_callback_fn, state);
}

void sbp2rtcm_init(struct rtcm3_out_state *state,
                   void (*cb_sbp_to_rtcm)(u8 *buffer,
                                          u16 length,
                                          void *context),
                   void *context) {
  state->leap_seconds = 0;
  state->leap_second_known = false;

  state->cb_sbp_to_rtcm = cb_sbp_to_rtcm;
  state->context = context;

  state->n_sbp_obs = 0;

  for (u8 i = 0; i < sizeof(state->glo_sv_id_fcn_map); i++) {
    state->glo_sv_id_fcn_map[i] = MSM_GLO_FCN_UNKNOWN;
  }

  state->send_legacy_obs = false;
  state->send_msm_obs = true;
  state->msm_type = MSM5;

  state->ant_known = false;
  state->ant_height = 0.0;
  memset(state->ant_descriptor, 0, sizeof(state->ant_descriptor));
  memset(state->rcv_descriptor, 0, sizeof(state->rcv_descriptor));

  rtcm_init_logging(&rtcm_log_callback_fn, state);
}

/* Difference between two sbp time stamps in seconds */
double sbp_diff_time(const sbp_gps_time_t *end,
                     const sbp_gps_time_t *beginning) {
  s32 week_diff = end->wn - beginning->wn;
  double dt = (double)end->tow / SECS_MS - (double)beginning->tow / SECS_MS;
  dt += week_diff * SEC_IN_WEEK;
  return dt;
}

static u16 rtcm_stn_to_sbp_sender_id(u16 rtcm_id) {
  /* To avoid conflicts with reserved low number sender ID's we or
   * on the highest nibble as RTCM sender ID's are 12 bit */
  return rtcm_id | 0xF080;
}

static u16 sbp_sender_to_rtcm_stn_id(u16 sbp_id) {
  /* Read the lowest 12 bits of the sender ID.
   * Note that the conversion betweeen SBP sender ID and RTCM station ID thus is
   * not reversible */
  return sbp_id & 0x0FFF;
}

void rtcm2sbp_decode_payload(const uint8_t *payload,
                             uint32_t payload_length,
                             struct rtcm3_sbp_state *state) {
  (void)payload_length;

  if (!gps_time_valid(&state->time_from_rover_obs)) {
    return;
  }

  uint16_t byte = 0;
  uint16_t message_type =
      (payload[byte] << 4) | ((payload[byte + 1] >> 4) & 0xf);

  switch (message_type) {
    case 1001:
    case 1003:
      break;
    case 1002: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1002(&payload[byte], &new_rtcm_obs)) {
        /* Need to check if we've got obs in the buffer from the previous epoch
         and send before accepting the new message */
        add_gps_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1004: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1004(&payload[byte], &new_rtcm_obs)) {
        /* Need to check if we've got obs in the buffer from the previous epoch
         and send before accepting the new message */
        add_gps_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1005: {
      rtcm_msg_1005 msg_1005;
      if (RC_OK == rtcm3_decode_1005(&payload[byte], &msg_1005)) {
        msg_base_pos_ecef_t sbp_base_pos;
        rtcm3_1005_to_sbp(&msg_1005, &sbp_base_pos);
        state->cb_rtcm_to_sbp(SBP_MSG_BASE_POS_ECEF,
                              (u8)sizeof(sbp_base_pos),
                              (u8 *)&sbp_base_pos,
                              rtcm_stn_to_sbp_sender_id(msg_1005.stn_id),
                              state->context);
      }
      break;
    }
    case 1006: {
      rtcm_msg_1006 msg_1006;
      if (RC_OK == rtcm3_decode_1006(&payload[byte], &msg_1006)) {
        msg_base_pos_ecef_t sbp_base_pos;
        rtcm3_1006_to_sbp(&msg_1006, &sbp_base_pos);
        state->cb_rtcm_to_sbp(
            SBP_MSG_BASE_POS_ECEF,
            (u8)sizeof(sbp_base_pos),
            (u8 *)&sbp_base_pos,
            rtcm_stn_to_sbp_sender_id(msg_1006.msg_1005.stn_id),
            state->context);
      }
      break;
    }
    case 1007:
    case 1008:
      break;
    case 1010: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1010(&payload[byte], &new_rtcm_obs) &&
          state->leap_second_known) {
        add_glo_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1012: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1012(&payload[byte], &new_rtcm_obs) &&
          state->leap_second_known) {
        add_glo_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1019: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_gps_eph(&payload[byte], &msg_eph)) {
        msg_ephemeris_gps_t sbp_gps_eph;
        rtcm3_gps_eph_to_sbp(&msg_eph, &sbp_gps_eph, state);
        state->cb_rtcm_to_sbp(SBP_MSG_EPHEMERIS_GPS,
                              (u8)sizeof(sbp_gps_eph),
                              (u8 *)&sbp_gps_eph,
                              rtcm_stn_to_sbp_sender_id(0),
                              state->context);
        rtcm2sbp_set_leap_second_from_wn(sbp_gps_eph.common.toe.wn, state);
      }
      break;
    }
    case 1020: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_glo_eph(&payload[byte], &msg_eph)) {
        msg_ephemeris_glo_t sbp_glo_eph;
        rtcm3_glo_eph_to_sbp(&msg_eph, &sbp_glo_eph, state);
        rtcm2sbp_set_glo_fcn(sbp_glo_eph.common.sid, sbp_glo_eph.fcn, state);
        state->cb_rtcm_to_sbp(SBP_MSG_EPHEMERIS_GLO,
                              (u8)sizeof(sbp_glo_eph),
                              (u8 *)&sbp_glo_eph,
                              rtcm_stn_to_sbp_sender_id(0),
                              state->context);
      }
      break;
    }
    case 1045: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_gal_eph_fnav(&payload[byte], &msg_eph)) {
        msg_ephemeris_gal_t sbp_gal_eph;
        rtcm3_gal_eph_to_sbp(&msg_eph, &sbp_gal_eph, state);
        state->cb_rtcm_to_sbp(SBP_MSG_EPHEMERIS_GAL,
                              (u8)sizeof(sbp_gal_eph),
                              (u8 *)&sbp_gal_eph,
                              rtcm_stn_to_sbp_sender_id(0),
                              state->context);
        rtcm2sbp_set_leap_second_from_wn(sbp_gal_eph.common.toe.wn, state);
      }
      break;
    }
    case 1042: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_bds_eph(&payload[byte], &msg_eph)) {
        msg_ephemeris_bds_t sbp_bds_eph;
        rtcm3_bds_eph_to_sbp(&msg_eph, &sbp_bds_eph, state);
        state->cb_rtcm_to_sbp(SBP_MSG_EPHEMERIS_BDS,
                              (u8)sizeof(sbp_bds_eph),
                              (u8 *)&sbp_bds_eph,
                              rtcm_stn_to_sbp_sender_id(0),
                              state->context);
      }
      break;
    }
    case 1046: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_gal_eph(&payload[byte], &msg_eph)) {
        msg_ephemeris_gal_t sbp_gal_eph;
        rtcm3_gal_eph_to_sbp(&msg_eph, &sbp_gal_eph, state);
        state->cb_rtcm_to_sbp(SBP_MSG_EPHEMERIS_GAL,
                              (u8)sizeof(sbp_gal_eph),
                              (u8 *)&sbp_gal_eph,
                              rtcm_stn_to_sbp_sender_id(0),
                              state->context);
        rtcm2sbp_set_leap_second_from_wn(sbp_gal_eph.common.toe.wn, state);
      }
      break;
    }
    case 1029: {
      rtcm_msg_1029 msg_1029;
      if (RC_OK == rtcm3_decode_1029(&payload[byte], &msg_1029)) {
        send_1029(&msg_1029, state);
      }
      break;
    }
    case 1033: {
      rtcm_msg_1033 msg_1033;
      if (RC_OK == rtcm3_decode_1033(&payload[byte], &msg_1033) &&
          no_1230_received(state)) {
        msg_glo_biases_t sbp_glo_cpb;
        rtcm3_1033_to_sbp(&msg_1033, &sbp_glo_cpb);
        state->cb_rtcm_to_sbp(SBP_MSG_GLO_BIASES,
                              (u8)sizeof(sbp_glo_cpb),
                              (u8 *)&sbp_glo_cpb,
                              rtcm_stn_to_sbp_sender_id(msg_1033.stn_id),
                              state->context);
      }
      break;
    }
    case 1230: {
      rtcm_msg_1230 msg_1230;
      if (RC_OK == rtcm3_decode_1230(&payload[byte], &msg_1230)) {
        msg_glo_biases_t sbp_glo_cpb;
        rtcm3_1230_to_sbp(&msg_1230, &sbp_glo_cpb);
        state->cb_rtcm_to_sbp(SBP_MSG_GLO_BIASES,
                              (u8)sizeof(sbp_glo_cpb),
                              (u8 *)&sbp_glo_cpb,
                              rtcm_stn_to_sbp_sender_id(msg_1230.stn_id),
                              state->context);
        state->last_1230_received = state->time_from_rover_obs;
      }
      break;
    }
    case 1059:
    case 1065:
    case 1242:
    case 1248:
    case 1260: {
      rtcm_msg_code_bias msg_code_bias;
      if (RC_OK == rtcm3_decode_code_bias(&payload[byte], &msg_code_bias)) {
        rtcm3_ssr_code_bias_to_sbp(&msg_code_bias, state);
      }
      break;
    }
    case 1060:
    case 1066:
    case 1243:
    case 1249:
    case 1261: {
      rtcm_msg_orbit_clock msg_orbit_clock;
      if (RC_OK == rtcm3_decode_orbit_clock(&payload[byte], &msg_orbit_clock)) {
        rtcm3_ssr_orbit_clock_to_sbp(&msg_orbit_clock, state);
      }
      break;
    }
    case 1265:
    case 1266:
    case 1267:
    case 1268:
    case 1269:
    case 1270: {
      rtcm_msg_phase_bias msg_phase_bias;
      if (RC_OK == rtcm3_decode_phase_bias(&payload[byte], &msg_phase_bias)) {
        rtcm3_ssr_phase_bias_to_sbp(&msg_phase_bias, state);
      }
      break;
    }
    case 1074:
    case 1084:
    case 1094:
    case 1124: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm4(&payload[byte], &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1075:
    case 1085:
    case 1095:
    case 1125: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm5(&payload[byte], &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1076:
    case 1086:
    case 1096:
    case 1126: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm6(&payload[byte], &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1077:
    case 1087:
    case 1097:
    case 1127: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm7(&payload[byte], &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1104:
    case 1105:
    case 1106:
    case 1107:
      /* SBAS messages suppressed for now */
      break;
    case 1114:
    case 1115:
    case 1116:
    case 1117:
      /* QZSS messages suppressed for now */
      break;
    case 1071:
    case 1072:
    case 1073:
    case 1081:
    case 1082:
    case 1083:
    case 1091:
    case 1092:
    case 1093:
    case 1101:
    case 1102:
    case 1103:
    case 1111:
    case 1112:
    case 1113:
    case 1121:
    case 1122:
    case 1123: {
      /* MSM1-3 messages (1xx3) are currently not supported, warn the user once
       * if these messages are seen - only warn once as these messages can be
       * present in streams that contain MSM4-7 or 1004 and 1012 so are valid */
      send_MSM_warning(&payload[byte], state);
      break;
    }
    default:
      break;
  }

  /* check if the message was the final MSM message in the epoch, and if so send
   * out the SBP buffer */
  if (message_type >= MSM_MSG_TYPE_MIN && message_type <= MSM_MSG_TYPE_MAX) {
    /* The Multiple message bit DF393 is the same regardless of MSM msg type */
    if (rtcm_getbitu(&payload[byte], MSM_MULTIPLE_BIT_OFFSET, 1) == 0) {
      send_observations(state);
    }
  }
}

void rtcm2sbp_decode_frame(const uint8_t *frame,
                           uint32_t frame_length,
                           struct rtcm3_sbp_state *state) {
  if (frame_length < 1) {
    return;
  }

  uint16_t byte = 1;
  uint16_t message_size = ((frame[byte] & 0x3) << 8) | frame[byte + 1];

  if (frame_length < message_size) {
    return;
  }

  byte += 2;
  rtcm2sbp_decode_payload(&frame[byte], message_size, state);
}

/* check if there was a MSM message decoded within the MSM timeout period */
static bool is_msm_active(const gps_time_t *current_time,
                          const struct rtcm3_sbp_state *state) {
  return gps_time_valid(&state->last_msm_received) &&
         gpsdifftime(current_time, &state->last_msm_received) < MSM_TIMEOUT_SEC;
}

/* returns number of bytes in the payload */
static u16 encode_rtcm3_payload(const void *rtcm_msg,
                                u16 message_type,
                                u8 *buff) {
  switch (message_type) {
    case 1004: {
      rtcm_obs_message *msg_1004 = (rtcm_obs_message *)rtcm_msg;
      return rtcm3_encode_1004(msg_1004, buff);
    }
    case 1005: {
      rtcm_msg_1005 *msg_1005 = (rtcm_msg_1005 *)rtcm_msg;
      return rtcm3_encode_1005(msg_1005, buff);
    }
    case 1006: {
      rtcm_msg_1006 *msg_1006 = (rtcm_msg_1006 *)rtcm_msg;
      return rtcm3_encode_1006(msg_1006, buff);
    }
    case 1008: {
      rtcm_msg_1008 *msg_1008 = (rtcm_msg_1008 *)rtcm_msg;
      return rtcm3_encode_1008(msg_1008, buff);
    }
    case 1012: {
      rtcm_obs_message *msg_1012 = (rtcm_obs_message *)rtcm_msg;
      return rtcm3_encode_1012(msg_1012, buff);
    }
    case 1033: {
      rtcm_msg_1033 *msg_1033 = (rtcm_msg_1033 *)rtcm_msg;
      return rtcm3_encode_1033(msg_1033, buff);
    }
    case 1230: {
      rtcm_msg_1230 *msg_1230 = (rtcm_msg_1230 *)rtcm_msg;
      return rtcm3_encode_1230(msg_1230, buff);
    }
    case 1074:
    case 1084:
    case 1094:
    case 1124: {
      rtcm_msm_message *msg_msm = (rtcm_msm_message *)rtcm_msg;
      return rtcm3_encode_msm4(msg_msm, buff);
    }
    case 1075:
    case 1085:
    case 1095:
    case 1125: {
      rtcm_msm_message *msg_msm = (rtcm_msm_message *)rtcm_msg;
      return rtcm3_encode_msm5(msg_msm, buff);
    }
    default:
      assert(!"Unsupported message type");
      break;
  }
  return 0;
}

/* Encode RTCM frame into the given buffer, returns frame length */
u16 encode_rtcm3_frame(const void *rtcm_msg, u16 message_type, u8 *frame) {
  u16 byte = 3;
  u16 message_size = encode_rtcm3_payload(rtcm_msg, message_type, &frame[byte]);

  if (0 == message_size) {
    fprintf(stderr, "Error encoding RTCM message %u\n", message_type);
    return 0;
  }

  u16 frame_size = message_size + 3;

  /* write the header */
  u16 bit = 0;
  rtcm_setbitu(frame, bit, 8, RTCM3_PREAMBLE);
  bit += 8;
  /* reserved bits */
  rtcm_setbitu(frame, bit, 6, 0);
  bit += 6;
  assert(message_size <= RTCM3_MAX_MSG_LEN);
  rtcm_setbitu(frame, bit, 10, message_size);
  bit += 10;
  bit += message_size * 8;
  u32 crc = crc24q(frame, frame_size, 0);
  rtcm_setbitu(frame, bit, 24, crc);

  return frame_size + 3;
}

void add_glo_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  gps_time_t obs_time;
  compute_glo_time(new_rtcm_obs->header.tow_ms,
                   &obs_time,
                   &state->time_from_rover_obs,
                   state);

  if (!gps_time_valid(&obs_time)) {
    /* invalid GLO time */
    return;
  }

  if (is_msm_active(&obs_time, state)) {
    /* Stream potentially contains also MSM observations, so discard the legacy
     * observation messages */
    return;
  }

  if (!gps_time_valid(&state->last_glo_time) ||
      gpsdifftime(&obs_time, &state->last_glo_time) > MS_TO_S / 2) {
    state->last_glo_time = obs_time;
    add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
  }
}

void add_gps_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  gps_time_t obs_time;
  compute_gps_time(new_rtcm_obs->header.tow_ms,
                   &obs_time,
                   &state->time_from_rover_obs,
                   state);
  assert(gps_time_valid(&obs_time));

  if (is_msm_active(&obs_time, state)) {
    /* Stream potentially contains also MSM observations, so discard the legacy
     * observation messages */
    return;
  }

  if (!gps_time_valid(&state->last_gps_time) ||
      gpsdifftime(&obs_time, &state->last_gps_time) > MS_TO_S / 2) {
    state->last_gps_time = obs_time;
    add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
  }
}

void add_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                       gps_time_t *obs_time,
                       struct rtcm3_sbp_state *state) {
  /* Transform the newly received obs to sbp */
  u8 new_obs[OBS_BUFFER_SIZE];
  memset(new_obs, 0, OBS_BUFFER_SIZE);
  msg_obs_t *new_sbp_obs = (msg_obs_t *)(new_obs);

  /* Find the buffer of obs to be sent */
  msg_obs_t *sbp_obs_buffer = (msg_obs_t *)state->obs_buffer;

  /* Build an SBP time stamp */
  new_sbp_obs->header.t.wn = obs_time->wn;
  new_sbp_obs->header.t.tow = (u32)rint(obs_time->tow * S_TO_MS);
  new_sbp_obs->header.t.ns_residual = 0;

  rtcm3_to_sbp(new_rtcm_obs, new_sbp_obs, state);

  /* Check if the buffer already has obs of the same time */
  if (sbp_obs_buffer->header.n_obs != 0 &&
      (sbp_obs_buffer->header.t.tow != new_sbp_obs->header.t.tow ||
       state->sender_id !=
           rtcm_stn_to_sbp_sender_id(new_rtcm_obs->header.stn_id))) {
    /* We either have missed a message, or we have a new station. Either way,
     send through the current buffer and clear before adding new obs */
    send_observations(state);
  }

  /* Copy new obs into buffer */
  u8 obs_index_buffer = sbp_obs_buffer->header.n_obs;
  state->sender_id = rtcm_stn_to_sbp_sender_id(new_rtcm_obs->header.stn_id);
  for (u8 obs_count = 0; obs_count < new_sbp_obs->header.n_obs; obs_count++) {
    if (obs_index_buffer >= MAX_OBS_PER_EPOCH) {
      send_buffer_full_error(state);
      break;
    }

    sbp_obs_buffer->obs[obs_index_buffer] = new_sbp_obs->obs[obs_count];
    obs_index_buffer++;
  }
  sbp_obs_buffer->header.n_obs = obs_index_buffer;
  sbp_obs_buffer->header.t = new_sbp_obs->header.t;

  /* If we aren't expecting another message, send the buffer */
  if (0 == new_rtcm_obs->header.sync) {
    send_observations(state);
  }
}

/**
 * Split the observation buffer into SBP messages and send them
 */
void send_observations(struct rtcm3_sbp_state *state) {
  const msg_obs_t *sbp_obs_buffer = (msg_obs_t *)state->obs_buffer;

  if (sbp_obs_buffer->header.n_obs == 0) {
    return;
  }

  /* We want the ceiling of n_obs divided by max obs in a single message to get
   * total number of messages needed */
  const u8 total_messages =
      1 + ((sbp_obs_buffer->header.n_obs - 1) / MAX_OBS_IN_SBP);

  assert(sbp_obs_buffer->header.n_obs <= MAX_OBS_PER_EPOCH);
  assert(total_messages <= SBP_MAX_OBS_SEQ);

  /* Write the SBP observation messages */
  u8 buffer_obs_index = 0;
  for (u8 msg_num = 0; msg_num < total_messages; ++msg_num) {
    u8 obs_data[SBP_FRAMING_MAX_PAYLOAD_SIZE];
    memset(obs_data, 0, SBP_FRAMING_MAX_PAYLOAD_SIZE);
    msg_obs_t *sbp_obs = (msg_obs_t *)obs_data;

    /* Write the header */
    sbp_obs->header.t = sbp_obs_buffer->header.t;
    /* Note: SBP n_obs puts total messages in the first nibble and msg_num in
     * the second. This differs from all the other instances of n_obs in this
     * module where it is used as observation count. */
    sbp_obs->header.n_obs = (total_messages << 4) + msg_num;

    /* Write the observations */
    u8 obs_index = 0;
    while (obs_index < MAX_OBS_IN_SBP &&
           buffer_obs_index < sbp_obs_buffer->header.n_obs) {
      sbp_obs->obs[obs_index++] = sbp_obs_buffer->obs[buffer_obs_index++];
    }

    u16 len = SBP_HDR_SIZE + obs_index * SBP_OBS_SIZE;
    assert(len <= SBP_FRAMING_MAX_PAYLOAD_SIZE);

    state->cb_rtcm_to_sbp(
        SBP_MSG_OBS, len, obs_data, state->sender_id, state->context);
  }
  /* clear the observation buffer, so also header.n_obs is set to zero */
  memset(state->obs_buffer, 0, OBS_BUFFER_SIZE);
}

bool gps_obs_message(u16 msg_num) {
  if (msg_num == 1001 || msg_num == 1002 || msg_num == 1003 ||
      msg_num == 1004) {
    return true;
  }
  return false;
}

bool glo_obs_message(u16 msg_num) {
  if (msg_num == 1009 || msg_num == 1010 || msg_num == 1011 ||
      msg_num == 1012) {
    return true;
  }
  return false;
}
code_t get_gps_sbp_code(u8 freq, u8 rtcm_code) {
  code_t code = CODE_INVALID;
  if (freq == L1_FREQ) {
    if (rtcm_code == 0) {
      code = CODE_GPS_L1CA;
    } else {
      code = CODE_GPS_L1P;
    }
  } else if (freq == L2_FREQ) {
    if (rtcm_code == 0) {
      code = CODE_GPS_L2CM;
    } else {
      code = CODE_GPS_L2P;
    }
  }
  return code;
}

code_t get_glo_sbp_code(u8 freq, u8 rtcm_code, struct rtcm3_sbp_state *state) {
  code_t code = CODE_INVALID;
  if (freq == L1_FREQ) {
    if (rtcm_code == 0) {
      code = CODE_GLO_L1OF;
    } else {
      /* CODE_GLO_L1P currently not supported in sbp */
      /* warn and then replace with a supported code for now */
      send_unsupported_code_warning(UNSUPPORTED_CODE_GLO_L1P, state);
      code = CODE_GLO_L1OF;
    }
  } else if (freq == L2_FREQ) {
    if (rtcm_code == 0) {
      code = CODE_GLO_L2OF;
    } else if (rtcm_code == 1) {
      /* CODE_GLO_L2P currently not supported in sbp */
      /* warn and then replace with a supported code for now */
      send_unsupported_code_warning(UNSUPPORTED_CODE_GLO_L2P, state);
      code = CODE_GLO_L2OF;
    } else {
      /* rtcm_code == 2 or 3 is reserved */
      code = CODE_INVALID;
    }
  }
  return code;
}

void rtcm3_to_sbp(const rtcm_obs_message *rtcm_obs,
                  msg_obs_t *new_sbp_obs,
                  struct rtcm3_sbp_state *state) {
  for (u8 sat = 0; sat < rtcm_obs->header.n_sat; ++sat) {
    for (u8 freq = 0; freq < NUM_FREQS; ++freq) {
      const rtcm_freq_data *rtcm_freq = &rtcm_obs->sats[sat].obs[freq];
      if (rtcm_freq->flags.valid_pr == 1 && rtcm_freq->flags.valid_cp == 1) {
        if (new_sbp_obs->header.n_obs >= MAX_OBS_PER_EPOCH) {
          send_buffer_full_error(state);
          return;
        }

        packed_obs_content_t *sbp_freq =
            &new_sbp_obs->obs[new_sbp_obs->header.n_obs];
        sbp_freq->flags = 0;
        sbp_freq->P = 0.0;
        sbp_freq->L.i = 0;
        sbp_freq->L.f = 0.0;
        sbp_freq->D.i = 0;
        sbp_freq->D.f = 0.0;
        sbp_freq->cn0 = 0.0;
        sbp_freq->lock = 0.0;

        sbp_freq->sid.sat = rtcm_obs->sats[sat].svId;
        if (gps_obs_message(rtcm_obs->header.msg_num)) {
          if (sbp_freq->sid.sat >= 1 && sbp_freq->sid.sat <= 32) {
            /* GPS PRN, see DF009 */
            sbp_freq->sid.code =
                get_gps_sbp_code(freq, rtcm_obs->sats[sat].obs[freq].code);
          } else if (sbp_freq->sid.sat >= 40 && sbp_freq->sid.sat <= 58 &&
                     freq == 0) {
            /* SBAS L1 PRN */
            sbp_freq->sid.code = CODE_SBAS_L1CA;
            sbp_freq->sid.sat += 80;
          } else {
            /* invalid PRN or code */
            continue;
          }
        } else if (glo_obs_message(rtcm_obs->header.msg_num)) {
          if (sbp_freq->sid.sat >= 1 && sbp_freq->sid.sat <= 24) {
            /* GLO PRN, see DF038 */
            code_t glo_sbp_code = get_glo_sbp_code(
                freq, rtcm_obs->sats[sat].obs[freq].code, state);
            if (glo_sbp_code == CODE_INVALID) {
              continue;
            } else {
              sbp_freq->sid.code = glo_sbp_code;
            }
          } else {
            /* invalid PRN or slot number uknown*/
            continue;
          }
        }

        if (rtcm_freq->flags.valid_pr == 1) {
          sbp_freq->P =
              (u32)rint(rtcm_freq->pseudorange * MSG_OBS_P_MULTIPLIER);
          sbp_freq->flags |= MSG_OBS_FLAGS_CODE_VALID;
        }
        if (rtcm_freq->flags.valid_cp == 1) {
          sbp_freq->L.i = (s32)floor(rtcm_freq->carrier_phase);
          u16 frac_part =
              (u16)rint((rtcm_freq->carrier_phase - (double)sbp_freq->L.i) *
                        MSG_OBS_LF_MULTIPLIER);
          if (frac_part == 256) {
            frac_part = 0;
            sbp_freq->L.i += 1;
          }
          sbp_freq->L.f = (u8)frac_part;
          sbp_freq->flags |= MSG_OBS_FLAGS_PHASE_VALID;
          sbp_freq->flags |= MSG_OBS_FLAGS_HALF_CYCLE_KNOWN;
        }

        if (rtcm_freq->flags.valid_cnr == 1) {
          sbp_freq->cn0 = (u8)rint(rtcm_freq->cnr * MSG_OBS_CN0_MULTIPLIER);
        } else {
          sbp_freq->cn0 = 0;
        }

        if (rtcm_freq->flags.valid_lock == 1) {
          sbp_freq->lock = rtcm3_encode_lock_time(rtcm_freq->lock);
        }

        new_sbp_obs->header.n_obs++;
      }
    }
  }
}

void rtcm3_1005_to_sbp(const rtcm_msg_1005 *rtcm_1005,
                       msg_base_pos_ecef_t *sbp_base_pos) {
  sbp_base_pos->x = rtcm_1005->arp_x;
  sbp_base_pos->y = rtcm_1005->arp_y;
  sbp_base_pos->z = rtcm_1005->arp_z;
}

void sbp_to_rtcm3_1005(const msg_base_pos_ecef_t *sbp_base_pos,
                       rtcm_msg_1005 *rtcm_1005,
                       const struct rtcm3_out_state *state) {
  /* Reference Station ID DF003 */
  rtcm_1005->stn_id = sbp_sender_to_rtcm_stn_id(state->sender_id);
  /* Reserved for ITRF Realization Year DF021 */
  rtcm_1005->ITRF = 0;
  /* GPS Indicator DF022 */
  rtcm_1005->GPS_ind = PIKSI_GPS_SERVICE_SUPPORTED;
  /* GLONASS Indicator DF023 */
  rtcm_1005->GLO_ind = PIKSI_GLO_SERVICE_SUPPORTED;
  /* Galileo Indicator DF024 */
  rtcm_1005->GAL_ind = PIKSI_GAL_SERVICE_SUPPORTED;
  /* Reference-Station Indicator DF141 */
  rtcm_1005->ref_stn_ind = PIKSI_REFERENCE_STATION_INDICATOR;
  /* Single Receiver Oscillator Indicator DF142 */
  rtcm_1005->osc_ind = PIKSI_SINGLE_OSCILLATOR_INDICATOR;
  /* Quarter Cycle Indicator DF364 */
  rtcm_1005->quart_cycle_ind = PIKSI_QUARTER_CYCLE_INDICATOR;

  /* Antenna Reference Point ECEF-X DF025 */
  rtcm_1005->arp_x = sbp_base_pos->x;
  /* Antenna Reference Point ECEF-Y DF026 */
  rtcm_1005->arp_y = sbp_base_pos->y;
  /* Antenna Reference Point ECEF-Z DF027 */
  rtcm_1005->arp_z = sbp_base_pos->z;
}

void rtcm3_1006_to_sbp(const rtcm_msg_1006 *rtcm_1006,
                       msg_base_pos_ecef_t *sbp_base_pos) {
  sbp_base_pos->x = rtcm_1006->msg_1005.arp_x;
  sbp_base_pos->y = rtcm_1006->msg_1005.arp_y;
  sbp_base_pos->z = rtcm_1006->msg_1005.arp_z;
}

void sbp_to_rtcm3_1006(const msg_base_pos_ecef_t *sbp_base_pos,
                       rtcm_msg_1006 *rtcm_1006,
                       const struct rtcm3_out_state *state) {
  sbp_to_rtcm3_1005(sbp_base_pos, &rtcm_1006->msg_1005, state);
  rtcm_1006->ant_height = state->ant_height;
}

void generate_rtcm3_1033(rtcm_msg_1033 *rtcm_1033,
                         const struct rtcm3_out_state *state) {
  memset(rtcm_1033, 0, sizeof(*rtcm_1033));

  /* Reference Station ID DF003 uint12 */
  rtcm_1033->stn_id = sbp_sender_to_rtcm_stn_id(state->sender_id);
  /* Antenna Setup ID DF031 uint8 */
  rtcm_1033->ant_setup_id = 0; /* 0 = Use Standard IGS Model */

  rtcm_1033->ant_serial_num_counter = 0; /* Antenna serial left blank */
  rtcm_1033->rcv_fw_version_counter = 0; /* Receiver FW left blank */
  rtcm_1033->rcv_serial_num_counter = 0; /* Receiver serial left blank */

  /* Antenna Descriptor Counter N DF029 uint8 */
  if (strlen(state->ant_descriptor) > 0) {
    rtcm_1033->ant_descriptor_counter = strlen(state->ant_descriptor);
    /* Antenna Descriptor DF030 char(N) */
    MEMCPY_S(rtcm_1033->ant_descriptor,
             sizeof(rtcm_1033->ant_descriptor),
             state->ant_descriptor,
             strlen(state->ant_descriptor));
  }

  if (strlen(state->rcv_descriptor) > 0) {
    /* Receiver Type Descriptor Counter I DF227 uint8 */
    rtcm_1033->rcv_descriptor_counter = strlen(state->rcv_descriptor);
    /* Receiver Type Descriptor DF228 char(I) */
    MEMCPY_S(rtcm_1033->rcv_descriptor,
             sizeof(rtcm_1033->rcv_descriptor),
             state->rcv_descriptor,
             strlen(state->rcv_descriptor));
  }
}

/* 1008 is a subset of 1033, so copy just the relevant fields */
void rtcm3_1033_to_1008(const rtcm_msg_1033 *rtcm_1033,
                        rtcm_msg_1008 *rtcm_1008) {
  memset(rtcm_1008, 0, sizeof(*rtcm_1008));

  /* Reference Station ID DF003 uint12 12 */
  rtcm_1008->msg_1007.stn_id = rtcm_1033->stn_id;
  /* Descriptor Counter N DF029 uint8 8 N <= 31 */
  if (rtcm_1033->ant_descriptor_counter > 0) {
    rtcm_1008->msg_1007.ant_descriptor_counter =
        rtcm_1033->ant_descriptor_counter;
    /* Antenna Descriptor DF030 char8(N) 8*N */
    MEMCPY_S(rtcm_1008->msg_1007.ant_descriptor,
             sizeof(rtcm_1008->msg_1007.ant_descriptor),
             rtcm_1033->ant_descriptor,
             rtcm_1033->ant_descriptor_counter);
  }

  /* Antenna Setup ID DF031 uint8 8 */
  rtcm_1008->msg_1007.ant_setup_id = rtcm_1033->ant_setup_id;

  /* Serial Number Counter M DF032 uint8 8 M <= 31 */
  if (rtcm_1008->ant_serial_num_counter > 0) {
    rtcm_1008->ant_serial_num_counter = rtcm_1033->ant_serial_num_counter;
    /* Antenna Serial Number DF033 char8(M) 8*M */
    MEMCPY_S(rtcm_1008->ant_serial_num,
             sizeof(rtcm_1008->ant_serial_num),
             rtcm_1033->ant_serial_num,
             rtcm_1033->ant_serial_num_counter);
  }
}

void rtcm3_1033_to_sbp(const rtcm_msg_1033 *rtcm_1033,
                       msg_glo_biases_t *sbp_glo_bias) {
  sbp_glo_bias->mask = 0;
  /* Resolution 2cm */
  /* GEO++ RCV NAMES MUST COME FIRST TO AVOID FALSE POSITIVE */
  if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=ASH)") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_ASH1_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_ASH1_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=HEM)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_HEM_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_HEM_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=JAV)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_JAV_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_JAV_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=JPS)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_JPS_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_JPS_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=LEI)") !=
                 NULL ||
             strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=NOV)") !=
                 NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_NOV_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_NOV_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=NAV)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_NAV_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_NAV_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=NVR)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_NVR_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_NVR_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=SEP)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_SEP_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_SEP_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=SOK)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_SOK_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_SOK_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=TPS)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_TPS_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_TPS_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=TRM)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(GPP_TRM_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(GPP_TRM_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "TRIMBLE") != NULL ||
             strstr(rtcm_1033->rcv_descriptor, "ASHTECH") != NULL) {
    sbp_glo_bias->mask = 0xF;
    sbp_glo_bias->l1ca_bias = (s16)rint(TRIMBLE_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = (s16)rint(TRIMBLE_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias = (s16)rint(TRIMBLE_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias = (s16)rint(TRIMBLE_BIAS_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "LEICA") != NULL ||
             strstr(rtcm_1033->rcv_descriptor, "NOV") != NULL ||
             strstr(rtcm_1033->rcv_descriptor, "GEOMAX") != NULL) {
    sbp_glo_bias->mask = 0xF;
    sbp_glo_bias->l1ca_bias = (s16)rint(NOVATEL_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = (s16)rint(NOVATEL_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias = (s16)rint(NOVATEL_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias = (s16)rint(NOVATEL_BIAS_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "SEPT") != NULL) {
    sbp_glo_bias->mask = 0xF;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(SEPTENTRIO_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = (s16)rint(SEPTENTRIO_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias =
        (s16)rint(SEPTENTRIO_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias = (s16)rint(SEPTENTRIO_BIAS_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "TPS") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = (s16)rint(TOPCON_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = (s16)rint(TOPCON_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias = (s16)rint(TOPCON_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias = (s16)rint(TOPCON_BIAS_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "JAVAD") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(JAVAD_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = (s16)rint(JAVAD_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "NAVCOM") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(NAVCOM_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = (s16)rint(NAVCOM_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "HEMI") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        (s16)rint(HEMISPHERE_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias =
        (s16)rint(HEMISPHERE_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  }
}

void rtcm3_1230_to_sbp(const rtcm_msg_1230 *rtcm_1230,
                       msg_glo_biases_t *sbp_glo_bias) {
  sbp_glo_bias->mask = rtcm_1230->fdma_signal_mask;
  /* GLONASS Code-Phase Bias Indicator DF421 */
  /* 0 - Pseudorange and Phaserange are not aligned, send out the correction
   *     message for the receiver to align the measurements
   * 1 - Pseudorange and Phaserange are aligned, send out zero corrections
   */
  if (0 == rtcm_1230->bias_indicator) {
    /* Biases with resolution of 2cm */
    sbp_glo_bias->l1ca_bias =
        (s16)rint(rtcm_1230->L1_CA_cpb_meter * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias =
        (s16)rint(rtcm_1230->L1_P_cpb_meter * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias =
        (s16)rint(rtcm_1230->L2_CA_cpb_meter * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias =
        (s16)rint(rtcm_1230->L2_P_cpb_meter * GLO_BIAS_RESOLUTION);
  } else {
    /* send zero biases */
    sbp_glo_bias->l1ca_bias = 0;
    sbp_glo_bias->l1p_bias = 0;
    sbp_glo_bias->l2ca_bias = 0;
    sbp_glo_bias->l2p_bias = 0;
  }
}

void sbp_to_rtcm3_1230(const msg_glo_biases_t *sbp_glo_bias,
                       rtcm_msg_1230 *rtcm_1230,
                       const struct rtcm3_out_state *state) {
  /* Reference Station ID DF003 */
  rtcm_1230->stn_id = sbp_sender_to_rtcm_stn_id(state->sender_id);
  /* GLONASS Code-Phase bias indicator DF421 */
  rtcm_1230->bias_indicator = PIKSI_GLO_CODE_PHASE_BIAS_INDICATOR;
  /* GLONASS FDMA signals mask DF422 */
  rtcm_1230->fdma_signal_mask = sbp_glo_bias->mask;

  /* GLONASS L1 C/A Code-Phase Bias DF423 */
  rtcm_1230->L1_CA_cpb_meter = sbp_glo_bias->l1ca_bias / GLO_BIAS_RESOLUTION;
  /* GLONASS L1 P Code-Phase Bias DF424 */
  rtcm_1230->L1_P_cpb_meter = sbp_glo_bias->l1p_bias / GLO_BIAS_RESOLUTION;
  /* GLONASS L2 C/A Code-Phase Bias DF425 */
  rtcm_1230->L2_CA_cpb_meter = sbp_glo_bias->l2ca_bias / GLO_BIAS_RESOLUTION;
  /* GLONASS L2 P Code-Phase Bias DF426 */
  rtcm_1230->L2_P_cpb_meter = sbp_glo_bias->l2p_bias / GLO_BIAS_RESOLUTION;
}

/* Convert from SBP FCN (1..14 with unknown marked with 0) to
 * RTCM FCN (0..13 with unknown marked with 255) */
static u8 sbp_fcn_to_rtcm(u8 sbp_fcn) {
  if (SBP_GLO_FCN_UNKNOWN == sbp_fcn) {
    return MSM_GLO_FCN_UNKNOWN;
  }
  s8 rtcm_fcn = sbp_fcn + MSM_GLO_FCN_OFFSET - SBP_GLO_FCN_OFFSET;
  if (rtcm_fcn < 0 || rtcm_fcn > MSM_GLO_MAX_FCN) {
    fprintf(stderr, "Ignoring invalid GLO FCN %d\n", sbp_fcn);
    return MSM_GLO_FCN_UNKNOWN;
  }
  return rtcm_fcn;
}

void rtcm2sbp_set_gps_time(const gps_time_t *current_time,
                           struct rtcm3_sbp_state *state) {
  if (gps_time_valid(current_time)) {
    state->time_from_rover_obs = *current_time;
  }
}

void rtcm2sbp_set_leap_second(s8 leap_seconds, struct rtcm3_sbp_state *state) {
  state->leap_seconds = leap_seconds;
  state->leap_second_known = true;
}

static void rtcm2sbp_set_leap_second_from_wn(u16 wn_ref,
                                             struct rtcm3_sbp_state *state) {
  gps_time_t gpst_sec = state->time_from_rover_obs;
  if (abs(gpst_sec.wn - wn_ref) > 1) {
    /* this only corrects big discrepancies */
    gpst_sec.wn = wn_ref;
  }
  gps_time_t gt = {.wn = gpst_sec.wn, .tow = gpst_sec.tow};
  s8 gps_utc_offset = (s8)rint(get_gps_utc_offset(&gt, NULL));
  rtcm2sbp_set_leap_second(gps_utc_offset, state);
}

void rtcm2sbp_set_glo_fcn(sbp_gnss_signal_t sid,
                          u8 sbp_fcn,
                          struct rtcm3_sbp_state *state) {
  /* convert FCN from SBP representation to RTCM representation */
  if (sid.sat < GLO_FIRST_PRN || sid.sat >= GLO_FIRST_PRN + NUM_SATS_GLO) {
    /* invalid PRN */
    fprintf(stderr, "Ignoring invalid GLO PRN %u\n", sid.sat);
    return;
  }
  state->glo_sv_id_fcn_map[sid.sat] = sbp_fcn_to_rtcm(sbp_fcn);
}

void sbp2rtcm_set_leap_second(s8 leap_seconds, struct rtcm3_out_state *state) {
  state->leap_seconds = leap_seconds;
  state->leap_second_known = true;
}

/* Set RTCM output mode
 * \param value msm_enum value, valid values
 *        MSM_UNKNOWN - no MSM output, send legacy 1004/1012 messages
 *        MSM4, MSM5  - MSM output
 * \param state RTCM output state struct
 */
void sbp2rtcm_set_rtcm_out_mode(msm_enum value, struct rtcm3_out_state *state) {
  if (MSM4 == value || MSM5 == value) {
    /* enable MSM output if valid MSM type given */
    state->send_msm_obs = true;
    state->msm_type = value;
    state->send_legacy_obs = false;
  } else {
    /* otherwise send legacy obs */
    state->send_legacy_obs = true;
    state->send_msm_obs = false;
  }
}

void sbp2rtcm_set_glo_fcn(sbp_gnss_signal_t sid,
                          u8 sbp_fcn,
                          struct rtcm3_out_state *state) {
  /* convert FCN from SBP representation to RTCM representation */
  if (sid.sat < GLO_FIRST_PRN || sid.sat >= GLO_FIRST_PRN + NUM_SATS_GLO) {
    /* invalid PRN */
    fprintf(stderr, "Ignoring invalid GLO PRN %u\n", sid.sat);
    return;
  }
  state->glo_sv_id_fcn_map[sid.sat] = sbp_fcn_to_rtcm(sbp_fcn);
}

bool sbp2rtcm_set_ant_height(const double ant_height,
                             struct rtcm3_out_state *state) {
  if (ant_height < 0.0 || ant_height > RTCM_1006_MAX_ANTENNA_HEIGHT_M) {
    return false;
  }
  state->ant_height = ant_height;
  state->ant_known = true;
  return true;
}

void sbp2rtcm_set_rcv_ant_descriptors(const char *ant_descriptor,
                                      const char *rcv_descriptor,
                                      struct rtcm3_out_state *state) {
  strncpy(state->ant_descriptor, ant_descriptor, sizeof(state->ant_descriptor));
  strncpy(state->rcv_descriptor, rcv_descriptor, sizeof(state->rcv_descriptor));
  state->ant_known = true;
};

void compute_gps_message_time(u32 tow_ms,
                              gps_time_t *obs_time,
                              const gps_time_t *rover_time) {
  assert(gps_time_valid(rover_time));
  obs_time->tow = tow_ms * MS_TO_S;
  obs_time->wn = rover_time->wn;
  double timediff = gpsdifftime(obs_time, rover_time);
  if (timediff < -SEC_IN_WEEK / 2) {
    obs_time->wn = rover_time->wn + 1;
  } else if (timediff > SEC_IN_WEEK / 2) {
    obs_time->wn = rover_time->wn - 1;
  }
  assert(gps_time_valid(obs_time));
}

void beidou_tow_to_gps_tow(u32 *tow_ms) {
  /* BDS system time has a constant offset */
  *tow_ms += BDS_SECOND_TO_GPS_SECOND * SECS_MS;
  if (*tow_ms >= SEC_IN_WEEK * S_TO_MS) {
    *tow_ms -= SEC_IN_WEEK * S_TO_MS;
  }
}

void gps_tow_to_beidou_tow(u32 *tow_ms) {
  /* BDS system time has a constant offset */
  if (*tow_ms < BDS_SECOND_TO_GPS_SECOND * S_TO_MS) {
    *tow_ms += SEC_IN_WEEK * S_TO_MS;
  }
  *tow_ms -= BDS_SECOND_TO_GPS_SECOND * S_TO_MS;
}

void compute_gps_time(u32 tow_ms,
                      gps_time_t *obs_time,
                      const gps_time_t *rover_time,
                      struct rtcm3_sbp_state *state) {
  compute_gps_message_time(tow_ms, obs_time, rover_time);
  validate_base_obs_sanity(state, obs_time, rover_time);
}

/* Compute full GLO time stamp from the time-of-day count, so that the result
 * is close to the supplied reference gps time
 * TODO: correct functionality during/around leap second event to be
 *       tested and implemented */
void compute_glo_time(u32 tod_ms,
                      gps_time_t *obs_time,
                      const gps_time_t *rover_time,
                      struct rtcm3_sbp_state *state) {
  if (!state->leap_second_known) {
    obs_time->wn = INVALID_TIME;
    return;
  }
  assert(tod_ms < (SEC_IN_DAY + 1) * S_TO_MS);
  assert(gps_time_valid(rover_time));

  /* Approximate DOW from the reference GPS time */
  u8 glo_dow = (u8)(rover_time->tow / SEC_IN_DAY);
  s32 glo_tod_ms = tod_ms - UTC_SU_OFFSET * SEC_IN_HOUR * S_TO_MS;

  obs_time->wn = rover_time->wn;
  s32 tow_ms = glo_dow * SEC_IN_DAY * S_TO_MS + glo_tod_ms +
               state->leap_seconds * S_TO_MS;
  while (tow_ms < 0) {
    tow_ms += SEC_IN_WEEK * S_TO_MS;
    obs_time->wn -= 1;
  }
  obs_time->tow = tow_ms * MS_TO_S;

  normalize_gps_time(obs_time);

  /* check for day rollover against reference time */
  double timediff = gpsdifftime(obs_time, rover_time);
  if (fabs(timediff) > SEC_IN_DAY / 2) {
    obs_time->tow += (timediff < 0 ? 1 : -1) * SEC_IN_DAY;
    normalize_gps_time(obs_time);
    timediff = gpsdifftime(obs_time, rover_time);
  }

  if (fabs(timediff - state->leap_seconds) > GLO_SANITY_THRESHOLD_S) {
    /* time too far from rover time, invalidate */
    obs_time->wn = INVALID_TIME;
  }
}

static void validate_base_obs_sanity(struct rtcm3_sbp_state *state,
                                     const gps_time_t *obs_time,
                                     const gps_time_t *rover_time) {
  assert(gps_time_valid(obs_time));
  assert(gps_time_valid(rover_time));
  double timediff = gpsdifftime(rover_time, obs_time);

  /* exclude base measurements with time stamp too far in the future */
  if (timediff >= BASE_FUTURE_THRESHOLD_S &&
      state->cb_base_obs_invalid != NULL) {
    state->cb_base_obs_invalid(timediff, state->context);
  }
}

bool no_1230_received(struct rtcm3_sbp_state *state) {
  if (!gps_time_valid(&state->last_1230_received) ||
      gpsdifftime(&state->time_from_rover_obs, &state->last_1230_received) >
          MSG_1230_TIMEOUT_SEC) {
    return true;
  }
  return false;
}

void send_1029(rtcm_msg_1029 *msg_1029, struct rtcm3_sbp_state *state) {
  uint8_t message[SBP_FRAMING_MAX_PAYLOAD_SIZE] = RTCM_LOG_PREAMBLE;
  uint8_t preamble_size = sizeof(RTCM_LOG_PREAMBLE) - 1;
  uint8_t max_message_size =
      SBP_FRAMING_MAX_PAYLOAD_SIZE - sizeof(msg_log_t) - preamble_size;
  uint8_t message_size =
      sizeof(msg_log_t) + msg_1029->utf8_code_units_n + preamble_size >
              SBP_FRAMING_MAX_PAYLOAD_SIZE
          ? max_message_size
          : msg_1029->utf8_code_units_n + preamble_size;

  MEMCPY_S(&message[preamble_size],
           max_message_size,
           msg_1029->utf8_code_units,
           message_size);
  /* Check if we've had to truncate the string - we can check for the bit
   * pattern that denotes a 4 byte code unit as it is the super set of all bit
   * patterns (2,3 and 4 byte code units) */
  if (message_size == max_message_size) {
    if ((message[message_size] & 0xF0) == 0xF0 ||
        (message[message_size] & 0xE0) == 0xF0 || /*TODO this is always false*/
        (message[message_size] & 0xF0) == 0xC0) {
      /* We've truncated a 2, 3 or 4 byte code unit */
      message_size--;
    } else if ((message[message_size - 1] & 0xF0) == 0xF0 ||
               (message[message_size - 1] & 0xE0) == 0xE0) {
      /* We've truncated a 3 or 4 byte code unit */
      message_size -= 2;
    } else if ((message[message_size - 1] & 0xF0) == 0xF0) {
      /* We've truncated a 4 byte code unit */
      message_size -= 3;
    }
  }

  send_sbp_log_message(
      RTCM_1029_LOGGING_LEVEL, message, message_size, msg_1029->stn_id, state);
}

void send_sbp_log_message(const uint8_t level,
                          const uint8_t *message,
                          uint16_t length,
                          const uint16_t stn_id,
                          const struct rtcm3_sbp_state *state) {
  u8 frame_buffer[SBP_FRAMING_MAX_PAYLOAD_SIZE];
  msg_log_t *sbp_log_msg = (msg_log_t *)frame_buffer;
  sbp_log_msg->level = level;

  /* truncate the message to fit in the payload */
  u16 max_message_length = SBP_FRAMING_MAX_PAYLOAD_SIZE - sizeof(*sbp_log_msg);
  if (length > max_message_length) {
    fprintf(stderr, "Truncating too long log message: %s\n", message);
    length = max_message_length;
  }
  MEMCPY_S(sbp_log_msg->text, max_message_length, message, length);
  state->cb_rtcm_to_sbp(SBP_MSG_LOG,
                        sizeof(*sbp_log_msg) + length,
                        (u8 *)frame_buffer,
                        rtcm_stn_to_sbp_sender_id(stn_id),
                        state->context);
}

void send_MSM_warning(const uint8_t *frame, struct rtcm3_sbp_state *state) {
  if (!state->sent_msm_warning) {
    /* Only send 1 warning */
    state->sent_msm_warning = true;
    /* Get the stn ID as well */
    uint32_t stn_id = 0;
    for (uint32_t i = 12; i < 24; i++) {
      stn_id = (stn_id << 1) + ((frame[i / 8] >> (7 - i % 8)) & 1u);
    }
    uint8_t msg[39] = "MSM1-3 Messages currently not supported";
    send_sbp_log_message(
        RTCM_MSM_LOGGING_LEVEL, msg, sizeof(msg), stn_id, state);
  }
}

void send_buffer_full_error(const struct rtcm3_sbp_state *state) {
  /* TODO: Get the stn ID as well */
  uint8_t log_msg[] = "Too many RTCM observations received!";
  send_sbp_log_message(
      RTCM_BUFFER_FULL_LOGGING_LEVEL, log_msg, sizeof(log_msg), 0, state);
}

void send_buffer_not_empty_warning(const struct rtcm3_sbp_state *state) {
  uint8_t log_msg[] =
      "RTCM MSM sequence not properly finished, sending incomplete message";
  send_sbp_log_message(
      RTCM_MSM_LOGGING_LEVEL, log_msg, sizeof(log_msg), 0, state);
}

const char *unsupported_code_desc[UNSUPPORTED_CODE_MAX] = {
    "Unknown or Unrecognized Code", /* UNSUPPORTED_CODE_UNKNOWN */
    "GLONASS L1P",                  /* UNSUPPORTED_CODE_GLO_L1P */
    "GLONASS L2P"                   /* UNSUPPORTED_CODE_GLO_L2P */
                                    /* UNSUPPORTED_CODE_MAX */
};

void send_unsupported_code_warning(const unsupported_code_t unsupported_code,
                                   struct rtcm3_sbp_state *state) {
  assert(unsupported_code < UNSUPPORTED_CODE_MAX);
  assert(sizeof(unsupported_code_desc) / sizeof(unsupported_code_desc[0]) >=
         UNSUPPORTED_CODE_MAX);
  if (!state->sent_code_warning[unsupported_code]) {
    /* Only send 1 warning */
    state->sent_code_warning[unsupported_code] = true;
    uint8_t msg[CODE_WARNING_BUFFER_SIZE];
    size_t count = snprintf((char *)msg,
                            CODE_WARNING_BUFFER_SIZE,
                            CODE_WARNING_FMT_STRING,
                            unsupported_code_desc[unsupported_code]);
    assert(count < CODE_WARNING_BUFFER_SIZE);
    send_sbp_log_message(RTCM_CODE_LOGGING_LEVEL, msg, count, 0, state);
  }
}

void rtcm_log_callback_fn(uint8_t level,
                          uint8_t *message,
                          uint16_t length,
                          void *context) {
  const struct rtcm3_sbp_state *state = (const struct rtcm3_sbp_state *)context;
  send_sbp_log_message(level, message, length, 0, state);
}

void add_msm_obs_to_buffer(const rtcm_msm_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  rtcm_constellation_t cons = to_constellation(new_rtcm_obs->header.msg_num);

  gps_time_t obs_time;
  if (RTCM_CONSTELLATION_GLO == cons) {
    compute_glo_time(new_rtcm_obs->header.tow_ms,
                     &obs_time,
                     &state->time_from_rover_obs,
                     state);
    if (!gps_time_valid(&obs_time)) {
      /* time invalid because of missing leap second info or ongoing leap second
       * event, skip these measurements */
      return;
    }

  } else {
    u32 tow_ms = new_rtcm_obs->header.tow_ms;

    if (RTCM_CONSTELLATION_BDS == cons) {
      beidou_tow_to_gps_tow(&tow_ms);
    }

    compute_gps_time(tow_ms, &obs_time, &state->time_from_rover_obs, state);
  }

  if (!gps_time_valid(&state->last_gps_time) ||
      gpsdifftime(&obs_time, &state->last_gps_time) >= -MS_TO_S / 2) {
    /* Find the buffer of obs to be sent */
    msg_obs_t *sbp_obs_buffer = (msg_obs_t *)state->obs_buffer;

    if (!is_msm_active(&obs_time, state) && sbp_obs_buffer->header.n_obs > 0) {
      /* This is the first MSM observation, so clear the already decoded legacy
       * messages from the observation buffer to avoid duplicates */
      memset(state->obs_buffer, 0, OBS_BUFFER_SIZE);
    }

    state->last_gps_time = obs_time;
    state->last_glo_time = obs_time;
    state->last_msm_received = obs_time;

    /* Transform the newly received obs to sbp */
    u8 new_obs[OBS_BUFFER_SIZE];
    memset(new_obs, 0, OBS_BUFFER_SIZE);
    msg_obs_t *new_sbp_obs = (msg_obs_t *)(new_obs);

    /* Build an SBP time stamp */
    new_sbp_obs->header.t.wn = obs_time.wn;
    new_sbp_obs->header.t.tow = (u32)rint(obs_time.tow * S_TO_MS);
    new_sbp_obs->header.t.ns_residual = 0;

    rtcm3_msm_to_sbp(new_rtcm_obs, new_sbp_obs, state);

    /* Check if the buffer already has obs of the same time */
    if (sbp_obs_buffer->header.n_obs != 0 &&
        (sbp_obs_buffer->header.t.tow != new_sbp_obs->header.t.tow ||
         state->sender_id !=
             rtcm_stn_to_sbp_sender_id(new_rtcm_obs->header.stn_id))) {
      /* We either have missed a message, or we have a new station. Either way,
       send through the current buffer and clear before adding new obs */
      send_buffer_not_empty_warning(state);
      send_observations(state);
    }

    /* Copy new obs into buffer */
    u8 obs_index_buffer = sbp_obs_buffer->header.n_obs;
    state->sender_id = rtcm_stn_to_sbp_sender_id(new_rtcm_obs->header.stn_id);
    for (u8 obs_count = 0; obs_count < new_sbp_obs->header.n_obs; obs_count++) {
      if (obs_index_buffer >= MAX_OBS_PER_EPOCH) {
        send_buffer_full_error(state);
        break;
      }

      assert(SBP_HDR_SIZE + (obs_index_buffer + 1) * SBP_OBS_SIZE <=
             OBS_BUFFER_SIZE);

      sbp_obs_buffer->obs[obs_index_buffer] = new_sbp_obs->obs[obs_count];
      obs_index_buffer++;
    }
    sbp_obs_buffer->header.n_obs = obs_index_buffer;
    sbp_obs_buffer->header.t = new_sbp_obs->header.t;
  }
}

/* return true if conversion to SID succeeded, and the SID as a pointer */
static bool get_sid_from_msm(const rtcm_msm_header *header,
                             u8 satellite_index,
                             u8 signal_index,
                             sbp_gnss_signal_t *sid,
                             struct rtcm3_sbp_state *state) {
  assert(signal_index <= MSM_SIGNAL_MASK_SIZE);
  code_t code = msm_signal_to_code(header, signal_index);
  u8 sat = msm_sat_to_prn(header, satellite_index);
  if (CODE_INVALID != code && PRN_INVALID != sat) {
    sid->code = code;
    sid->sat = sat;
    return true;
  }
  if (CODE_INVALID == code) {
    /* should have specific code warning but this requires modifying librtcm */
    send_unsupported_code_warning(UNSUPPORTED_CODE_UNKNOWN, state);
  }
  return false;
}

static bool unsupported_signal(sbp_gnss_signal_t *sid) {
  switch (sid->code) {
    case CODE_GLO_L1P:
    case CODE_GLO_L2P:
    case CODE_BDS3_B1CI:
    case CODE_BDS3_B1CQ:
    case CODE_BDS3_B1CX:
    case CODE_BDS3_B5I:
    case CODE_BDS3_B5Q:
    case CODE_BDS3_B5X:
    case CODE_BDS3_B7I:
    case CODE_BDS3_B7Q:
    case CODE_BDS3_B7X:
    case CODE_BDS3_B3I:
    case CODE_BDS3_B3Q:
    case CODE_BDS3_B3X:
    case CODE_AUX_GPS:
    case CODE_AUX_SBAS:
    case CODE_AUX_GAL:
    case CODE_AUX_QZS:
    case CODE_AUX_BDS:
      return true;
    default:
      return false;
  }
}

void rtcm3_msm_to_sbp(const rtcm_msm_message *msg,
                      msg_obs_t *new_sbp_obs,
                      struct rtcm3_sbp_state *state) {
  uint8_t num_sats = msm_get_num_satellites(&msg->header);
  uint8_t num_sigs = msm_get_num_signals(&msg->header);

  u8 cell_index = 0;
  for (u8 sat = 0; sat < num_sats; sat++) {
    for (u8 sig = 0; sig < num_sigs; sig++) {
      if (msg->header.cell_mask[sat * num_sigs + sig]) {
        sbp_gnss_signal_t sid;
        const rtcm_msm_signal_data *data = &msg->signals[cell_index];
        bool sid_valid = get_sid_from_msm(&msg->header, sat, sig, &sid, state);
        bool supported = !unsupported_signal(&sid);
        if (sid_valid && supported && data->flags.valid_pr &&
            data->flags.valid_cp) {
          if (new_sbp_obs->header.n_obs >= MAX_OBS_PER_EPOCH) {
            send_buffer_full_error(state);
            return;
          }

          /* get GLO FCN */
          uint8_t glo_fcn = MSM_GLO_FCN_UNKNOWN;
          bool glo_fcn_valid = msm_get_glo_fcn(&msg->header,
                                               sat,
                                               msg->sats[sat].glo_fcn,
                                               state->glo_sv_id_fcn_map,
                                               &glo_fcn);

          double freq;
          bool freq_valid = msm_signal_frequency(
              &msg->header, sig, glo_fcn, glo_fcn_valid, &freq);

          packed_obs_content_t *sbp_freq =
              &new_sbp_obs->obs[new_sbp_obs->header.n_obs];

          sbp_freq->flags = 0;
          sbp_freq->P = 0.0;
          sbp_freq->L.i = 0;
          sbp_freq->L.f = 0.0;
          sbp_freq->D.i = 0;
          sbp_freq->D.f = 0.0;
          sbp_freq->cn0 = 0.0;
          sbp_freq->lock = 0.0;

          sbp_freq->sid = sid;

          if (data->flags.valid_pr) {
            double pseudorange_m = data->pseudorange_ms * GPS_C / 1000;
            sbp_freq->P = (u32)rint(pseudorange_m * MSG_OBS_P_MULTIPLIER);
            sbp_freq->flags |= MSG_OBS_FLAGS_CODE_VALID;
          }
          if (data->flags.valid_cp && freq_valid) {
            double carrier_phase_cyc = data->carrier_phase_ms * freq / 1000;
            sbp_freq->L.i = (s32)floor(carrier_phase_cyc);
            u16 frac_part =
                (u16)rint((carrier_phase_cyc - (double)sbp_freq->L.i) *
                          MSG_OBS_LF_MULTIPLIER);
            if (256 == frac_part) {
              frac_part = 0;
              sbp_freq->L.i += 1;
            }
            sbp_freq->L.f = (u8)frac_part;
            sbp_freq->flags |= MSG_OBS_FLAGS_PHASE_VALID;
            if (!data->hca_indicator) {
              sbp_freq->flags |= MSG_OBS_FLAGS_HALF_CYCLE_KNOWN;
            }
          }

          if (data->flags.valid_cnr) {
            sbp_freq->cn0 = (u8)rint(data->cnr * MSG_OBS_CN0_MULTIPLIER);
          } else {
            sbp_freq->cn0 = 0;
          }

          if (data->flags.valid_lock) {
            sbp_freq->lock = rtcm3_encode_lock_time(data->lock_time_s);
          }

          if (data->flags.valid_dop && freq_valid) {
            /* flip Doppler sign to Piksi sign convention */
            double doppler_Hz = -data->range_rate_m_s * freq / GPS_C;
            sbp_freq->D.i = (s16)floor(doppler_Hz);
            u16 frac_part = (u16)rint((doppler_Hz - (double)sbp_freq->D.i) *
                                      MSG_OBS_DF_MULTIPLIER);
            if (256 == frac_part) {
              frac_part = 0;
              sbp_freq->D.i += 1;
            }
            sbp_freq->D.f = (u8)frac_part;
            sbp_freq->flags |= MSG_OBS_FLAGS_DOPPLER_VALID;
          }

          new_sbp_obs->header.n_obs++;
        }
        cell_index++;
      }
    }
  }
}

void sbp2rtcm_base_pos_ecef_cb(const u16 sender_id,
                               const u8 len,
                               const u8 msg[],
                               struct rtcm3_out_state *state) {
  (void)len;
  rtcm_msg_1006 msg_1006;
  rtcm_msg_1008 msg_1008;
  rtcm_msg_1033 msg_1033;
  u8 frame[RTCM3_MAX_MSG_LEN];

  state->sender_id = sender_id;

  /* generate and send the base position message */
  sbp_to_rtcm3_1006((const msg_base_pos_ecef_t *)msg, &msg_1006, state);
  u16 frame_size = encode_rtcm3_frame(&msg_1006, 1006, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);

  if (state->ant_known) {
    /* generate and send the receiver and antenna description messages */
    generate_rtcm3_1033(&msg_1033, state);
    rtcm3_1033_to_1008(&msg_1033, &msg_1008);

    frame_size = encode_rtcm3_frame(&msg_1008, 1008, frame);
    state->cb_sbp_to_rtcm(frame, frame_size, state->context);

    frame_size = encode_rtcm3_frame(&msg_1033, 1033, frame);
    state->cb_sbp_to_rtcm(frame, frame_size, state->context);
  }
}

void sbp2rtcm_glo_biases_cb(const u16 sender_id,
                            const u8 len,
                            const u8 msg[],
                            struct rtcm3_out_state *state) {
  (void)len;
  state->sender_id = sender_id;

  rtcm_msg_1230 msg_1230;
  sbp_to_rtcm3_1230((const msg_glo_biases_t *)msg, &msg_1230, state);

  u8 frame[RTCM3_MAX_MSG_LEN];
  u16 frame_size = encode_rtcm3_frame(&msg_1230, 1230, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

static void sbp_obs_to_freq_data(const packed_obs_content_t *sbp_freq,
                                 rtcm_freq_data *rtcm_freq,
                                 u8 code_indicator) {
  /* 0=C/A code, 1=P code */
  rtcm_freq->code = code_indicator;

  rtcm_freq->pseudorange = sbp_freq->P / MSG_OBS_P_MULTIPLIER;
  rtcm_freq->flags.valid_pr =
      (0 != (sbp_freq->flags & MSG_OBS_FLAGS_CODE_VALID)) ? 1 : 0;

  rtcm_freq->flags.valid_cp =
      (0 != (sbp_freq->flags & MSG_OBS_FLAGS_PHASE_VALID) &&
       0 != (sbp_freq->flags & MSG_OBS_FLAGS_HALF_CYCLE_KNOWN))
          ? 1
          : 0;
  rtcm_freq->carrier_phase =
      (double)sbp_freq->L.i + (double)sbp_freq->L.f / MSG_OBS_LF_MULTIPLIER;

  rtcm_freq->flags.valid_cnr = (sbp_freq->cn0 > 0) ? 1 : 0;
  rtcm_freq->cnr = sbp_freq->cn0 / MSG_OBS_CN0_MULTIPLIER;

  /* SBP lock indicator is in DF402 4-bit format */
  rtcm_freq->lock = rtcm3_decode_lock_time(sbp_freq->lock);
  rtcm_freq->flags.valid_lock = 1;

  /* no Doppler obs */
  rtcm_freq->flags.valid_dop = 0;
}

/* initialize the MSM structure */
static void msm_init_obs_message(rtcm_msm_message *msg,
                                 const struct rtcm3_out_state *state,
                                 const rtcm_constellation_t cons) {
  memset(msg, 0, sizeof(*msg));

  /* Msg Num DF002 uint16 12*/
  msg->header.msg_num = to_msm_msg_num(cons, state->msm_type);

  /* GPS time of week DF004 uint32 30 */
  msg->header.tow_ms = state->sbp_header.t.tow;
  if (RTCM_CONSTELLATION_GLO == cons) {
    /* GLO epoch time DF034 uint32 27 */
    msg->header.tow_ms = compute_glo_tod_ms(state->sbp_header.t.tow, state);
  } else if (RTCM_CONSTELLATION_BDS == cons) {
    gps_tow_to_beidou_tow(&msg->header.tow_ms);
  }

  /* Station Id DF003 uint16 12*/
  msg->header.stn_id = sbp_sender_to_rtcm_stn_id(state->sender_id);

  /* Issue of Data Station DF409 uint8 3 */
  /* A value of "0" indicates that this field is not utilized. */
  msg->header.iods = 0;

  /* Clock Steering Indicator DF411 uint2 2 */
  msg->header.steering = PIKSI_CLOCK_STEERING_INDICATOR;

  /* External Clock Indicator DF412 uint2 2 */
  msg->header.ext_clock = PIKSI_EXT_CLOCK_INDICATOR;

  /* Divergence free flag DF417 bit(1) 1 */
  msg->header.div_free = PIKSI_DIVERGENCE_FREE_INDICATOR;

  /* GPS Smoothing Interval DF418 bit(3) 3 */
  msg->header.smooth = PIKSI_SMOOTHING_INTERVAL;

  /* MSM Multiple Message bit DF393 bit(1) 1
   * 0 this is the last message
   * 1 more messages to follow */
  /* initialize messages to 1, reset the flag of the last message later */
  msg->header.multiple = 1;
}

/* convert the SBP observation into MSM message structure */
static void sbp_obs_to_msm_signal_data(const packed_obs_content_t *sbp_obs,
                                       rtcm_msm_message *msg,
                                       const struct rtcm3_out_state *state) {
  rtcm_constellation_t cons = to_constellation(msg->header.msg_num);
  uint8_t num_sigs = msm_get_num_signals(&msg->header);

  u8 sat_index = prn_to_msm_sat_index(&msg->header, sbp_obs->sid.sat);
  u8 signal_index = code_to_msm_signal_index(&msg->header, sbp_obs->sid.code);
  u8 cell_id = sat_index * num_sigs + signal_index;

  if (!msg->header.cell_mask[cell_id]) {
    fprintf(stderr,
            "Cell mask not set for sat %u signal %u\n",
            sat_index,
            signal_index);
    return;
  }

  u8 cell_index = count_mask_values(cell_id, msg->header.cell_mask);

  /* convenience pointers */

  rtcm_msm_sat_data *sat_data = &(msg->sats[sat_index]);
  rtcm_msm_signal_data *signal_data = &msg->signals[cell_index];
  flag_bf *msm_flags = &signal_data->flags;
  const u8 sbp_flags = sbp_obs->flags;

  /* fill in the signal data */

  if (0 != (sbp_flags & MSG_OBS_FLAGS_CODE_VALID)) {
    double pseudorange_m = sbp_obs->P / MSG_OBS_P_MULTIPLIER;
    signal_data->pseudorange_ms = pseudorange_m * 1000 / GPS_C;
    sat_data->rough_range_ms = rint(signal_data->pseudorange_ms * 1024) / 1024;
    msm_flags->valid_pr = true;
  } else {
    msm_flags->valid_pr = false;
  }

  if (RTCM_CONSTELLATION_GLO == cons) {
    sat_data->glo_fcn = state->glo_sv_id_fcn_map[sbp_obs->sid.sat];
  } else {
    sat_data->glo_fcn = MSM_GLO_FCN_UNKNOWN;
  }

  double freq;
  bool freq_valid =
      msm_signal_frequency(&msg->header,
                           signal_index,
                           sat_data->glo_fcn,
                           (MSM_GLO_FCN_UNKNOWN != sat_data->glo_fcn),
                           &freq);

  if (freq_valid && (0 != (sbp_flags & MSG_OBS_FLAGS_PHASE_VALID))) {
    double carrier_cyc = sbp_obs->L.i + sbp_obs->L.f / MSG_OBS_LF_MULTIPLIER;
    signal_data->carrier_phase_ms = carrier_cyc * 1000 / freq;
    /* DF420 Half-cycle ambiguity indicator
     * 0 = no ambiguity
     * 1 = ambiguity */
    signal_data->hca_indicator = (0 == (sbp_flags & MSG_OBS_FLAGS_CODE_VALID));
    msm_flags->valid_cp = true;
  } else {
    msm_flags->valid_cp = false;
  }

  if (freq_valid && (0 != (sbp_flags & MSG_OBS_FLAGS_DOPPLER_VALID))) {
    double doppler_Hz = sbp_obs->D.i + sbp_obs->D.f / MSG_OBS_DF_MULTIPLIER;
    signal_data->range_rate_m_s = -doppler_Hz * GPS_C / freq;
    sat_data->rough_range_rate_m_s = rint(signal_data->range_rate_m_s);
    msm_flags->valid_dop = true;
  } else {
    msm_flags->valid_dop = false;
  }

  signal_data->lock_time_s = rtcm3_decode_lock_time(sbp_obs->lock);
  msm_flags->valid_lock = true;

  signal_data->cnr = sbp_obs->cn0 / MSG_OBS_CN0_MULTIPLIER;
  msm_flags->valid_cnr = true;
}

uint32_t compute_glo_tod_ms(uint32_t gps_tow_ms,
                            const struct rtcm3_out_state *state) {
  if (!state->leap_second_known) {
    return -1;
  }

  double gps_tod_s = gps_tow_ms * MS_TO_S;
  gps_tod_s -= floor(gps_tod_s / SEC_IN_DAY) * SEC_IN_DAY;

  double glo_tod_s =
      gps_tod_s + UTC_SU_OFFSET * SEC_IN_HOUR - state->leap_seconds;

  if (glo_tod_s < 0) {
    glo_tod_s += SEC_IN_DAY;
  }
  /* TODO: this fails during the leap second event */
  if (glo_tod_s >= SEC_IN_DAY) {
    glo_tod_s -= SEC_IN_DAY;
  }

  assert(glo_tod_s >= 0 && glo_tod_s < (SEC_IN_DAY + 1));
  return (uint32_t)rint(glo_tod_s * S_TO_MS);
}

static void rtcm_init_obs_message(rtcm_obs_message *msg,
                                  struct rtcm3_out_state *state,
                                  rtcm_constellation_t cons) {
  memset(&*msg, 0, sizeof(*msg));

  if (RTCM_CONSTELLATION_GPS == cons) {
    /* Msg Num DF002 uint16 12*/
    msg->header.msg_num = 1004;
    /* GPS time of week DF004 uint32 30 */
    msg->header.tow_ms = state->sbp_header.t.tow;
  } else {
    /* Msg Num DF002 uint16 12*/
    msg->header.msg_num = 1012;
    /* GLO epoch time DF034 uint32 27 */
    msg->header.tow_ms = compute_glo_tod_ms(state->sbp_header.t.tow, state);
  }
  /* Station Id DF003 uint16 12*/
  msg->header.stn_id = sbp_sender_to_rtcm_stn_id(state->sender_id);
  /* Divergence free flag DF007 bit(1) 1 */
  msg->header.div_free = PIKSI_DIVERGENCE_FREE_INDICATOR;
  /* GPS Smoothing Interval DF008 bit(3) 3 */
  msg->header.smooth = PIKSI_SMOOTHING_INTERVAL;
}

/* Find the matching svId from sat data array
 * Return the found index, or n_sats if not found */
static u8 sat_index_from_obs(rtcm_sat_data sats[], u8 n_sats, u8 svId) {
  for (u8 sat_i = 0; sat_i < n_sats; sat_i++) {
    if (sats[sat_i].svId == svId) {
      return sat_i;
    }
  }
  return n_sats;
}

void sbp_buffer_to_msm(const struct rtcm3_out_state *state) {
  /* message for each constellation */
  rtcm_msm_message obs[RTCM_CONSTELLATION_COUNT];
  for (u8 cons = 0; cons < RTCM_CONSTELLATION_COUNT; cons++) {
    msm_init_obs_message(&obs[cons], state, cons);
  }

  /* loop through observations once to generate satellite and signal masks */
  for (u8 i = 0; i < state->n_sbp_obs; i++) {
    const packed_obs_content_t *sbp_obs = &(state->sbp_obs_buffer[i]);
    rtcm_constellation_t cons =
        (rtcm_constellation_t)code_to_constellation(sbp_obs->sid.code);
    msm_add_to_header(&obs[cons].header, sbp_obs->sid.code, sbp_obs->sid.sat);
  }

  /* using the complete satellite and signal masks, loop through observations
   * again to generate the cell mask */
  for (u8 i = 0; i < state->n_sbp_obs; i++) {
    const packed_obs_content_t *sbp_obs = &(state->sbp_obs_buffer[i]);
    rtcm_constellation_t cons =
        (rtcm_constellation_t)code_to_constellation(sbp_obs->sid.code);
    msm_add_to_cell_mask(
        &obs[cons].header, sbp_obs->sid.code, sbp_obs->sid.sat);
  }

  /* loop through observations once more to generate the actual signal data */
  for (u8 i = 0; i < state->n_sbp_obs; i++) {
    const packed_obs_content_t *sbp_obs = &(state->sbp_obs_buffer[i]);
    rtcm_constellation_t cons =
        (rtcm_constellation_t)code_to_constellation(sbp_obs->sid.code);
    sbp_obs_to_msm_signal_data(sbp_obs, &obs[cons], state);
  }

  /* Fill in the MSM Multiple Message bit DF393 bit(1) 1
   * 0 this is the last message
   * 1 more messages to follow */
  for (s8 cons = RTCM_CONSTELLATION_COUNT - 1; cons >= 0; cons--) {
    if (msm_get_num_satellites(&obs[cons].header) > 0) {
      /* reset the multiple message bit of the last constellation that has
       * measurements, others have already been initialized to one */
      obs[cons].header.multiple = 0;
      break;
    }
  }

  /* send out all the messages that have measurements */
  static u8 frame[RTCM3_MAX_MSG_LEN];
  for (u8 cons = 0; cons < RTCM_CONSTELLATION_COUNT; cons++) {
    if (msm_get_num_satellites(&obs[cons].header) > 0) {
      u16 frame_size =
          encode_rtcm3_frame(&obs[cons], obs[cons].header.msg_num, frame);
      state->cb_sbp_to_rtcm(frame, frame_size, state->context);
    }
  }
}

static void sbp_buffer_to_legacy_rtcm3(struct rtcm3_out_state *state) {
  rtcm_obs_message gps_obs;
  rtcm_init_obs_message(&gps_obs, state, RTCM_CONSTELLATION_GPS);

  rtcm_obs_message glo_obs;
  rtcm_init_obs_message(&glo_obs, state, RTCM_CONSTELLATION_GLO);

  u8 n_gps = 0;
  u8 n_glo = 0;

  /* Loop through the observation buffer once and add the observations into
   * gps_obs and glo_obs structures.
   *
   * The observations are stored by satellite. New satellite is created into the
   * struct for each L1 observation. Note that L2 observations are added only if
   * the satellite exists already, that is it has already an L1 observation.
   * This requires that observations come in sorted with L1 first.
   */

  for (u8 i = 0; i < state->n_sbp_obs; i++) {
    packed_obs_content_t *sbp_obs = &(state->sbp_obs_buffer[i]);
    switch (sbp_obs->sid.code) {
      case CODE_GPS_L1CA:
      case CODE_GPS_L1P: {
        /* add a new sat */
        gps_obs.sats[n_gps].fcn = 0;
        gps_obs.sats[n_gps].svId = sbp_obs->sid.sat;
        sbp_obs_to_freq_data(sbp_obs,
                             &(gps_obs.sats[n_gps].obs[L1_FREQ]),
                             (sbp_obs->sid.code == CODE_GPS_L1P) ? 1 : 0);
        n_gps++;
        break;
      }
      case CODE_GPS_L2CM:
      case CODE_GPS_L2P: {
        /* find the sat with L1 observations already added */
        u8 sat_i = sat_index_from_obs(gps_obs.sats, n_gps, sbp_obs->sid.sat);
        if (sat_i < n_gps) {
          sbp_obs_to_freq_data(sbp_obs,
                               &(gps_obs.sats[sat_i].obs[L2_FREQ]),
                               (sbp_obs->sid.code == CODE_GPS_L2P) ? 1 : 0);
        }
        break;
      }
      case CODE_GLO_L1OF:
      case CODE_GLO_L1P: {
        u8 fcn = state->glo_sv_id_fcn_map[sbp_obs->sid.sat];
        if (MSM_GLO_FCN_UNKNOWN != fcn) {
          /* add a new sat */
          glo_obs.sats[n_glo].fcn = fcn;
          glo_obs.sats[n_glo].svId = sbp_obs->sid.sat;
          sbp_obs_to_freq_data(sbp_obs,
                               &(glo_obs.sats[n_glo].obs[L1_FREQ]),
                               (sbp_obs->sid.code == CODE_GLO_L1P) ? 1 : 0);
          n_glo++;
        }
        break;
      }
      case CODE_GLO_L2OF:
      case CODE_GLO_L2P: {
        /* find the sat with L1 observations already added */
        u8 sat_i = sat_index_from_obs(glo_obs.sats, n_glo, sbp_obs->sid.sat);
        if (sat_i < n_glo) {
          sbp_obs_to_freq_data(sbp_obs,
                               &(glo_obs.sats[sat_i].obs[L2_FREQ]),
                               (sbp_obs->sid.code == CODE_GLO_L2P) ? 1 : 0);
        }
        break;
      }
      default:
        break;
    }
  }

  /* Number of satellites DF006 uint8 5 */
  gps_obs.header.n_sat = n_gps;
  glo_obs.header.n_sat = n_glo;

  /* Syncronous flag DF005 bit(1) 1 */
  gps_obs.header.sync = (n_glo > 0) ? 1 : 0; /* if GLO message will follow */
  glo_obs.header.sync = 0; /* no further messages for this epoch */

  static u8 frame[RTCM3_MAX_MSG_LEN];
  if (n_gps > 0) {
    u16 frame_size =
        encode_rtcm3_frame(&gps_obs, gps_obs.header.msg_num, frame);
    state->cb_sbp_to_rtcm(frame, frame_size, state->context);
  }

  if (n_glo > 0) {
    u16 frame_size =
        encode_rtcm3_frame(&glo_obs, glo_obs.header.msg_num, frame);
    state->cb_sbp_to_rtcm(frame, frame_size, state->context);
  }
}

static void sbp_buffer_to_rtcm3(struct rtcm3_out_state *state) {
  if (state->send_legacy_obs) {
    sbp_buffer_to_legacy_rtcm3(state);
  }
  if (state->send_msm_obs) {
    sbp_buffer_to_msm(state);
  }
  /* clear the sent messages from the buffer */
  state->n_sbp_obs = 0;
}

void sbp2rtcm_sbp_obs_cb(const u16 sender_id,
                         const u8 len,
                         const u8 msg[],
                         struct rtcm3_out_state *state) {
  msg_obs_t *sbp_obs = (msg_obs_t *)msg;

  if (state->sender_id != sender_id) {
    /* sender changed, send existing buffer and reset */
    sbp_buffer_to_rtcm3(state);
    state->sender_id = sender_id;
  }

  u8 seq_counter = (sbp_obs->header.n_obs & 0x0F) + 1;
  u8 seq_size = sbp_obs->header.n_obs >> 4;

  /* if sbp buffer is not empty, check that this observations belongs to the
   * sequence */
  if (state->n_sbp_obs > 0) {
    double dt = sbp_diff_time(&sbp_obs->header.t, &state->sbp_header.t);

    if (dt < 0) {
      fprintf(stderr,
              "Discarding SBP obs with timestamp %.1f seconds in the past\n",
              dt);
      return;
    }
    if (dt > 0) {
      /* observations belong to the next epoch, send out the current buffer
       * before processing this message */
      fprintf(
          stderr,
          "SBP obs sequence ended prematurely, starting new one at dt=%.1f\n",
          dt);

      sbp_buffer_to_rtcm3(state);
    } else {
      u8 current_seq_counter = (state->sbp_header.n_obs & 0x0F) + 1;
      u8 current_seq_size = state->sbp_header.n_obs >> 4;

      if (seq_size != current_seq_size || seq_counter <= current_seq_counter) {
        /* sequence broken, send out current buffer and ignore this message */
        fprintf(stderr,
                "Ignoring SBP obs with invalid obs sequence: expected %d/%d "
                "but got %d/%d\n",
                current_seq_counter + 1,
                current_seq_size,
                seq_counter,
                seq_size);
        sbp_buffer_to_rtcm3(state);
        return;
      }
      if (seq_counter != current_seq_counter + 1) {
        /* missed a packet, emit warning but still process this message */
        fprintf(stderr,
                "Missed an SBP obs packet, expected seq %d/%d but got %d/%d\n",
                current_seq_counter + 1,
                current_seq_size,
                seq_counter,
                seq_size);
      }
    }
  }

  /* number of observations in the incoming message */
  u8 n_meas =
      (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

  for (u8 i = 0; i < n_meas; i++) {
    if ((0 == (sbp_obs->obs[i].flags & MSG_OBS_FLAGS_CODE_VALID)) ||
        (0 != (sbp_obs->obs[i].flags & MSG_OBS_FLAGS_RAIM_EXCLUSION))) {
      /* skip observations that do not have valid code measurement or that are
       * flagged by RAIM */
      continue;
    }
    /* add incoming sbp observation into sbp buffer */
    state->sbp_obs_buffer[state->n_sbp_obs] = sbp_obs->obs[i];
    state->n_sbp_obs++;
    if (state->n_sbp_obs == MAX_OBS_PER_EPOCH) {
      fprintf(
          stderr,
          "Reached max observations per epoch %u, ignoring the remaining %u "
          "obs\n",
          (u16)MAX_OBS_PER_EPOCH,
          n_meas - i - 1);
      break;
    }
  }
  state->sbp_header = sbp_obs->header;

  /* if sequence is complete, convert into RTCM */
  if (seq_counter == seq_size) {
    sbp_buffer_to_rtcm3(state);
  }
}
