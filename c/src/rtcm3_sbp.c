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

#include <assert.h>
#include <bits.h>
#include <math.h>
#include <rtcm3_decode.h>
#include <rtcm3_eph_decode.h>
#include <rtcm3_msm_utils.h>
#include <rtcm_logging.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libsbp/logging.h>
#include "rtcm3_sbp_internal.h"

static void validate_base_obs_sanity(struct rtcm3_sbp_state *state,
                                     const gps_time_sec_t *obs_time,
                                     const gps_time_sec_t *rover_time);

void rtcm2sbp_init(
    struct rtcm3_sbp_state *state,
    void (*cb_rtcm_to_sbp)(u16 msg_id, u8 length, u8 *buffer, u16 sender_id),
    void (*cb_base_obs_invalid)(double timediff)) {
  state->time_from_rover_obs.wn = INVALID_TIME;
  state->time_from_rover_obs.tow = 0;

  state->leap_seconds = 0;
  state->leap_second_known = false;

  state->sender_id = 0;
  state->cb_rtcm_to_sbp = cb_rtcm_to_sbp;
  state->cb_base_obs_invalid = cb_base_obs_invalid;

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

  for (u8 i = 0; i < GLO_LAST_PRN + 1; i++) {
    state->glo_sv_id_fcn_map[i] = MSM_GLO_FCN_UNKNOWN;
  }

  memset(state->obs_buffer, 0, OBS_BUFFER_SIZE);

  rtcm_init_logging(&rtcm_log_callback_fn, state);
}

static void normalize_gps_time(gps_time_sec_t *t) {
  /* note that tow is unsigned so no need to check tow<0 */

  while (t->tow >= SEC_IN_WEEK) {
    t->tow -= SEC_IN_WEEK;
    t->wn += 1;
  }
}

static bool gps_time_valid(const gps_time_sec_t *t) {
  return (t->wn != INVALID_TIME) && (t->wn < MAX_WN) && (t->tow < SEC_IN_WEEK);
}

s32 gps_diff_time_sec(const gps_time_sec_t *end,
                      const gps_time_sec_t *beginning) {
  assert(gps_time_valid(beginning));
  assert(gps_time_valid(end));

  s16 week_diff = end->wn - beginning->wn;
  /* assert the total difference in seconds is represantable by s32 (works out
   * to 3549 weeks) */
  assert(week_diff > -MAX_WEEK_DIFF && week_diff < MAX_WEEK_DIFF);

  s32 dt = (s32)end->tow - (s32)beginning->tow;
  dt += week_diff * SEC_IN_WEEK;
  return dt;
}

static u16 rtcm_2_sbp_sender_id(u16 rtcm_id) {
  /* To avoid conflicts with reserved low number sender ID's we or
   * on the highest nibble as RTCM sender ID's are 12 bit */
  return rtcm_id | 0xF000;
}

void rtcm2sbp_decode_frame(const uint8_t *frame,
                           uint32_t frame_length,
                           struct rtcm3_sbp_state *state) {
  if (!gps_time_valid(&state->time_from_rover_obs) || frame_length < 1) {
    return;
  }

  uint16_t byte = 1;
  uint16_t message_size = ((frame[byte] & 0x3) << 8) | frame[byte + 1];

  if (frame_length < message_size) {
    return;
  }

  byte += 2;
  uint16_t message_type = (frame[byte] << 4) | ((frame[byte + 1] >> 4) & 0xf);

  switch (message_type) {
    case 1001:
    case 1003:
      break;
    case 1002: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1002(&frame[byte], &new_rtcm_obs)) {
        /* Need to check if we've got obs in the buffer from the previous epoch
         and send before accepting the new message */
        add_gps_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1004: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1004(&frame[byte], &new_rtcm_obs)) {
        /* Need to check if we've got obs in the buffer from the previous epoch
         and send before accepting the new message */
        add_gps_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1005: {
      rtcm_msg_1005 msg_1005;
      if (RC_OK == rtcm3_decode_1005(&frame[byte], &msg_1005)) {
        msg_base_pos_ecef_t sbp_base_pos;
        rtcm3_1005_to_sbp(&msg_1005, &sbp_base_pos);
        state->cb_rtcm_to_sbp(SBP_MSG_BASE_POS_ECEF,
                              (u8)sizeof(sbp_base_pos),
                              (u8 *)&sbp_base_pos,
                              rtcm_2_sbp_sender_id(msg_1005.stn_id));
      }
      break;
    }
    case 1006: {
      rtcm_msg_1006 msg_1006;
      if (RC_OK == rtcm3_decode_1006(&frame[byte], &msg_1006)) {
        msg_base_pos_ecef_t sbp_base_pos;
        rtcm3_1006_to_sbp(&msg_1006, &sbp_base_pos);
        state->cb_rtcm_to_sbp(SBP_MSG_BASE_POS_ECEF,
                              (u8)sizeof(sbp_base_pos),
                              (u8 *)&sbp_base_pos,
                              rtcm_2_sbp_sender_id(msg_1006.msg_1005.stn_id));
      }
      break;
    }
    case 1007:
    case 1008:
      break;
    case 1010: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1010(&frame[byte], &new_rtcm_obs) &&
          state->leap_second_known) {
        add_glo_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1012: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1012(&frame[byte], &new_rtcm_obs) &&
          state->leap_second_known) {
        add_glo_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1019: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_gps_eph(&frame[byte], &msg_eph)){
        msg_ephemeris_gps_t sbp_gps_eph;
        rtcm3_gps_eph_to_sbp(&msg_eph, &sbp_gps_eph, state);
        state->cb_rtcm_to_sbp(SBP_MSG_EPHEMERIS_GPS,
                              (u8)sizeof(sbp_gps_eph),
                              (u8 *)&sbp_gps_eph,
                              rtcm_2_sbp_sender_id(0));
      }
    }
    case 1020: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_glo_eph(&frame[byte], &msg_eph)){
        msg_ephemeris_glo_t sbp_glo_eph;
        rtcm3_glo_eph_to_sbp(&msg_eph, &sbp_glo_eph, state);
        state->cb_rtcm_to_sbp(SBP_MSG_EPHEMERIS_GLO,
                              (u8)sizeof(sbp_glo_eph),
                              (u8 *)&sbp_glo_eph,
                              rtcm_2_sbp_sender_id(0));
      }
    }
    case 1029: {
      rtcm_msg_1029 msg_1029;
      if (RC_OK == rtcm3_decode_1029(&frame[byte], &msg_1029)) {
        send_1029(&msg_1029, state);
      }
      break;
    }
    case 1033: {
      rtcm_msg_1033 msg_1033;
      if (RC_OK == rtcm3_decode_1033(&frame[byte], &msg_1033) &&
          no_1230_received(state)) {
        msg_glo_biases_t sbp_glo_cpb;
        rtcm3_1033_to_sbp(&msg_1033, &sbp_glo_cpb);
        state->cb_rtcm_to_sbp(SBP_MSG_GLO_BIASES,
                              (u8)sizeof(sbp_glo_cpb),
                              (u8 *)&sbp_glo_cpb,
                              rtcm_2_sbp_sender_id(msg_1033.stn_id));
      }
      break;
    }
    case 1230: {
      rtcm_msg_1230 msg_1230;
      if (RC_OK == rtcm3_decode_1230(&frame[byte], &msg_1230)) {
        msg_glo_biases_t sbp_glo_cpb;
        rtcm3_1230_to_sbp(&msg_1230, &sbp_glo_cpb);
        state->cb_rtcm_to_sbp(SBP_MSG_GLO_BIASES,
                              (u8)sizeof(sbp_glo_cpb),
                              (u8 *)&sbp_glo_cpb,
                              rtcm_2_sbp_sender_id(msg_1230.stn_id));
        state->last_1230_received = state->time_from_rover_obs;
      }
      break;
    }
    case 1074:
    case 1084:
    case 1094:
    case 1124: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm4(
                       &frame[byte], state->glo_sv_id_fcn_map, &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1075:
    case 1085:
    case 1095:
    case 1125: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm5(&frame[byte], &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1076:
    case 1086:
    case 1096:
    case 1126: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm6(
                       &frame[byte], state->glo_sv_id_fcn_map, &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1077:
    case 1087:
    case 1097:
    case 1127: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm7(&frame[byte], &new_rtcm_msm)) {
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
      send_MSM_warning(&frame[byte], state);
      break;
    }
    default:
      break;
  }

  /* check if the message was the final MSM message in the epoch, and if so send
   * out the SBP buffer */
  if (message_type >= MSM_MSG_TYPE_MIN && message_type <= MSM_MSG_TYPE_MAX) {
    /* The Multiple message bit DF393 is the same regardless of MSM msg type */
    if (getbitu(&frame[byte], MSM_MULTIPLE_BIT_OFFSET, 1) == 0) {
      send_observations(state);
    }
  }
}

void add_glo_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  gps_time_sec_t obs_time;
  compute_glo_time(new_rtcm_obs->header.tow_ms,
                   &obs_time,
                   &state->time_from_rover_obs,
                   state);

  if (!gps_time_valid(&obs_time)) {
    /* invalid GLO time */
    return;
  }

  if (gps_time_valid(&state->last_msm_received) &&
      gps_diff_time_sec(&obs_time, &state->last_msm_received) <
          MSM_TIMEOUT_SEC) {
    /* Stream potentially contains also MSM observations, so discard the legacy
     * observation messages */
    return;
  }

  if (!gps_time_valid(&state->last_glo_time) ||
      gps_diff_time_sec(&obs_time, &state->last_glo_time) > 0) {
    state->last_glo_time = obs_time;
    add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
  }
}

void add_gps_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  gps_time_sec_t obs_time;
  compute_gps_time(new_rtcm_obs->header.tow_ms,
                   &obs_time,
                   &state->time_from_rover_obs,
                   state);
  assert(gps_time_valid(&obs_time));

  if (gps_time_valid(&state->last_msm_received) &&
      gps_diff_time_sec(&obs_time, &state->last_msm_received) <
          MSM_TIMEOUT_SEC) {
    /* Stream potentially contains also MSM observations, so discard the legacy
     * observation messages */
    return;
  }

  if (!gps_time_valid(&state->last_gps_time) ||
      gps_diff_time_sec(&obs_time, &state->last_gps_time) > 0) {
    state->last_gps_time = obs_time;
    add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
  }
}

void add_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                       gps_time_sec_t *obs_time,
                       struct rtcm3_sbp_state *state) {
  /* Transform the newly received obs to sbp */
  u8 new_obs[OBS_BUFFER_SIZE];
  memset(new_obs, 0, OBS_BUFFER_SIZE);
  msg_obs_t *new_sbp_obs = (msg_obs_t *)(new_obs);

  /* Find the buffer of obs to be sent */
  msg_obs_t *sbp_obs_buffer = (msg_obs_t *)state->obs_buffer;

  /* Build an SBP time stamp */
  new_sbp_obs->header.t.wn = obs_time->wn;
  new_sbp_obs->header.t.tow = obs_time->tow * S_TO_MS;
  new_sbp_obs->header.t.ns_residual = 0;

  rtcm3_to_sbp(new_rtcm_obs, new_sbp_obs, state);

  /* Check if the buffer already has obs of the same time */
  if (sbp_obs_buffer->header.n_obs != 0 &&
      (sbp_obs_buffer->header.t.tow != new_sbp_obs->header.t.tow ||
       state->sender_id != rtcm_2_sbp_sender_id(new_rtcm_obs->header.stn_id))) {
    /* We either have missed a message, or we have a new station. Either way,
     send through the current buffer and clear before adding new obs */
    send_observations(state);
  }

  /* Copy new obs into buffer */
  u8 obs_index_buffer = sbp_obs_buffer->header.n_obs;
  state->sender_id = rtcm_2_sbp_sender_id(new_rtcm_obs->header.stn_id);
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

    state->cb_rtcm_to_sbp(SBP_MSG_OBS, len, obs_data, state->sender_id);
  }
  /* clear the observation buffer, so also header.n_obs is set to zero */
  memset(state->obs_buffer, 0, OBS_BUFFER_SIZE);
}

/** Convert navigation_measurement_t.lock_time into SBP lock time.
 *
 * Note: It is encoded according to DF402 from the RTCM 10403.2 Amendment 2
 * specification.  Valid values range from 0 to 15 and the most significant
 * nibble is reserved for future use.
 *
 * \param nm_lock_time Navigation measurement lock time [s]
 * \return SBP lock time
 */
u8 encode_lock_time(double nm_lock_time) {
  assert(nm_lock_time >= 0.0);

  /* Convert to milliseconds */
  u32 ms_lock_time;
  if (nm_lock_time < UINT32_MAX) {
    ms_lock_time = (u32)(nm_lock_time * SECS_MS);
  } else {
    ms_lock_time = UINT32_MAX;
  }

  if (ms_lock_time < 32) {
    return 0;
  } else {
    for (u8 i = 0; i < 16; i++) {
      if (ms_lock_time > (1u << (i + 5))) {
        continue;
      } else {
        return i;
      }
    }
    return 15;
  }
}

/** Convert SBP lock time into navigation_measurement_t.lock_time.
 *
 * Note: It is encoded according to DF402 from the RTCM 10403.2 Amendment 2
 * specification.  Valid values range from 0 to 15 and the most significant
 * nibble is reserved for future use.
 *
 * \param sbp_lock_time SBP lock time
 * \return Minimum possible lock time [s]
 */
double decode_lock_time(u8 sbp_lock_time) {
  /* MSB nibble is reserved */
  sbp_lock_time &= 0x0F;

  u32 ms_lock_time;
  if (sbp_lock_time == 0) {
    ms_lock_time = 0;
  } else {
    ms_lock_time = 1u << (sbp_lock_time + 4);
  }

  /* Convert to seconds */
  return (double)ms_lock_time / SECS_MS;
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
              (u32)roundl(rtcm_freq->pseudorange * MSG_OBS_P_MULTIPLIER);
          sbp_freq->flags |= MSG_OBS_FLAGS_CODE_VALID;
        }
        if (rtcm_freq->flags.valid_cp == 1) {
          sbp_freq->L.i = (s32)floor(rtcm_freq->carrier_phase);
          u16 frac_part =
              (u16)roundl((rtcm_freq->carrier_phase - (double)sbp_freq->L.i) *
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
          sbp_freq->cn0 = (u8)roundl(rtcm_freq->cnr * MSG_OBS_CN0_MULTIPLIER);
        } else {
          sbp_freq->cn0 = 0;
        }

        if (rtcm_freq->flags.valid_lock == 1) {
          sbp_freq->lock = encode_lock_time(rtcm_freq->lock);
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
                       rtcm_msg_1005 *rtcm_1005) {
  rtcm_1005->arp_x = sbp_base_pos->x;
  rtcm_1005->arp_y = sbp_base_pos->y;
  rtcm_1005->arp_z = sbp_base_pos->z;
}

void rtcm3_1006_to_sbp(const rtcm_msg_1006 *rtcm_1006,
                       msg_base_pos_ecef_t *sbp_base_pos) {
  sbp_base_pos->x = rtcm_1006->msg_1005.arp_x;
  sbp_base_pos->y = rtcm_1006->msg_1005.arp_y;
  sbp_base_pos->z = rtcm_1006->msg_1005.arp_z;
}

void sbp_to_rtcm3_1006(const msg_base_pos_ecef_t *sbp_base_pos,
                       rtcm_msg_1006 *rtcm_1006) {
  rtcm_1006->msg_1005.arp_x = sbp_base_pos->x;
  rtcm_1006->msg_1005.arp_y = sbp_base_pos->y;
  rtcm_1006->msg_1005.arp_z = sbp_base_pos->z;
  rtcm_1006->ant_height = 0.0;
}

void rtcm3_1033_to_sbp(const rtcm_msg_1033 *rtcm_1033,
                       msg_glo_biases_t *sbp_glo_bias) {
  sbp_glo_bias->mask = 0;
  /* Resolution 2cm */
  /* GEO++ RCV NAMES MUST COME FIRST TO AVOID FALSE POSITIVE */
  if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=ASH)") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_ASH1_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_ASH1_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=HEM)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_HEM_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_HEM_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=JAV)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_JAV_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_JAV_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=JPS)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_JPS_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_JPS_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=LEI)") !=
                 NULL ||
             strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=NOV)") !=
                 NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_NOV_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_NOV_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=NAV)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_NAV_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_NAV_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=NVR)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_NVR_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_NVR_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=SEP)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_SEP_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_SEP_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=SOK)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_SOK_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_SOK_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=TPS)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_TPS_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_TPS_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "Geo++ GNSMART (GLO=TRM)") !=
             NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(GPP_TRM_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(GPP_TRM_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "TRIMBLE") != NULL ||
             strstr(rtcm_1033->rcv_descriptor, "ASHTECH") != NULL) {
    sbp_glo_bias->mask = 0xF;
    sbp_glo_bias->l1ca_bias = round(TRIMBLE_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = round(TRIMBLE_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias = round(TRIMBLE_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias = round(TRIMBLE_BIAS_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "LEICA") != NULL ||
             strstr(rtcm_1033->rcv_descriptor, "NOV") != NULL ||
             strstr(rtcm_1033->rcv_descriptor, "GEOMAX") != NULL) {
    sbp_glo_bias->mask = 0xF;
    sbp_glo_bias->l1ca_bias = round(NOVATEL_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = round(NOVATEL_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias = round(NOVATEL_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias = round(NOVATEL_BIAS_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "SEPT") != NULL) {
    sbp_glo_bias->mask = 0xF;
    sbp_glo_bias->l1ca_bias = round(SEPTENTRIO_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = round(SEPTENTRIO_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias = round(SEPTENTRIO_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias = round(SEPTENTRIO_BIAS_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "TPS") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(TOPCON_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = round(TOPCON_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2ca_bias = round(TOPCON_BIAS_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l2p_bias = round(TOPCON_BIAS_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "JAVAD") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(JAVAD_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(JAVAD_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "NAVCOM") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias = round(NAVCOM_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(NAVCOM_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  } else if (strstr(rtcm_1033->rcv_descriptor, "HEMI") != NULL) {
    sbp_glo_bias->mask = 0x9;
    sbp_glo_bias->l1ca_bias =
        round(HEMISPHERE_BIAS_L1CA_M * GLO_BIAS_RESOLUTION);
    sbp_glo_bias->l1p_bias = 0.0;
    sbp_glo_bias->l2ca_bias = 0.0;
    sbp_glo_bias->l2p_bias = round(HEMISPHERE_BIAS_L2P_M * GLO_BIAS_RESOLUTION);
  }
}

void rtcm3_1230_to_sbp(const rtcm_msg_1230 *rtcm_1230,
                       msg_glo_biases_t *sbp_glo_bias) {
  sbp_glo_bias->mask = rtcm_1230->fdma_signal_mask;
  /* Resolution 2cm */
  s8 sign_indicator = rtcm_1230->bias_indicator == 0 ? 1 : -1;
  sbp_glo_bias->l1ca_bias =
      round(sign_indicator * rtcm_1230->L1_CA_cpb_meter * GLO_BIAS_RESOLUTION);
  sbp_glo_bias->l1p_bias =
      round(sign_indicator * rtcm_1230->L1_P_cpb_meter * GLO_BIAS_RESOLUTION);
  sbp_glo_bias->l2ca_bias =
      round(sign_indicator * rtcm_1230->L2_CA_cpb_meter * GLO_BIAS_RESOLUTION);
  sbp_glo_bias->l2p_bias =
      round(sign_indicator * rtcm_1230->L2_P_cpb_meter * GLO_BIAS_RESOLUTION);
}

void sbp_to_rtcm3_1230(const msg_glo_biases_t *sbp_glo_bias,
                       rtcm_msg_1230 *rtcm_1230) {
  rtcm_1230->fdma_signal_mask = sbp_glo_bias->mask;
  rtcm_1230->bias_indicator = 0;
  rtcm_1230->L1_CA_cpb_meter = sbp_glo_bias->l1ca_bias / GLO_BIAS_RESOLUTION;
  rtcm_1230->L1_P_cpb_meter = sbp_glo_bias->l1p_bias / GLO_BIAS_RESOLUTION;
  rtcm_1230->L2_CA_cpb_meter = sbp_glo_bias->l2ca_bias / GLO_BIAS_RESOLUTION;
  rtcm_1230->L2_P_cpb_meter = sbp_glo_bias->l2p_bias / GLO_BIAS_RESOLUTION;
}

void rtcm2sbp_set_gps_time(const gps_time_sec_t *current_time,
                           struct rtcm3_sbp_state *state) {
  if (gps_time_valid(current_time)) {
    state->time_from_rover_obs = *current_time;
  }
}

void rtcm2sbp_set_leap_second(s8 leap_seconds, struct rtcm3_sbp_state *state) {
  state->leap_seconds = leap_seconds;
  state->leap_second_known = true;
}

void rtcm2sbp_set_glo_fcn(sbp_gnss_signal_t sid,
                          u8 sbp_fcn,
                          struct rtcm3_sbp_state *state) {
  /* convert FCN from SBP representation to RTCM representation */
  if (sid.sat < GLO_FIRST_PRN || sid.sat > GLO_LAST_PRN) {
    /* invalid PRN */
    return;
  }
  if (SBP_GLO_FCN_UNKNOWN == sbp_fcn) {
    state->glo_sv_id_fcn_map[sid.sat] = MSM_GLO_FCN_UNKNOWN;
  } else {
    /* in SBP, FCN is 1..14, with the unknown value 0 already checked above */
    s8 fcn = sbp_fcn - SBP_GLO_FCN_OFFSET;
    /* in RTCM, FCN is in 0..13 */
    state->glo_sv_id_fcn_map[sid.sat] = fcn + MSM_GLO_FCN_OFFSET;
  }
}

void compute_gps_time(u32 tow_ms,
                      gps_time_sec_t *obs_time,
                      const gps_time_sec_t *rover_time,
                      struct rtcm3_sbp_state *state) {
  assert(gps_time_valid(rover_time));
  obs_time->tow = (u32)(tow_ms * MS_TO_S);
  obs_time->wn = rover_time->wn;
  s32 timediff = gps_diff_time_sec(obs_time, rover_time);
  if (timediff < -SEC_IN_WEEK / 2) {
    obs_time->wn = rover_time->wn + 1;
  } else if (timediff > SEC_IN_WEEK / 2) {
    obs_time->wn = rover_time->wn - 1;
  }
  assert(gps_time_valid(obs_time));
  validate_base_obs_sanity(state, obs_time, rover_time);
}

/* Compute full GLO time stamp from the time-of-day count, so that the result
 * is close to the supplied reference gps time */
void compute_glo_time(u32 tod_ms,
                      gps_time_sec_t *obs_time,
                      const gps_time_sec_t *rover_time,
                      struct rtcm3_sbp_state *state) {
  if (!state->leap_second_known) {
    obs_time->wn = INVALID_TIME;
    return;
  }
  assert(gps_time_valid(rover_time));

  /* Approximate DOW from the reference GPS time */
  u8 glo_dow = rover_time->tow / SEC_IN_DAY;
  s32 glo_tod_sec = (u32)(tod_ms * MS_TO_S) - UTC_SU_OFFSET * SEC_IN_HOUR;

  if (glo_tod_sec >= SEC_IN_DAY) {
    /* Time of day overflows, can possibly happen during leap second event.
     * Return as invalid. */
    obs_time->wn = INVALID_TIME;
    return;
  }

  obs_time->wn = rover_time->wn;
  obs_time->tow = glo_dow * SEC_IN_DAY + glo_tod_sec + state->leap_seconds;
  normalize_gps_time(obs_time);

  /* check for day rollover against reference time */
  s32 timediff = gps_diff_time_sec(obs_time, rover_time);
  if (abs(timediff) > SEC_IN_DAY / 2) {
    obs_time->tow += (timediff < 0 ? 1 : -1) * SEC_IN_DAY;
    normalize_gps_time(obs_time);
    timediff = gps_diff_time_sec(obs_time, rover_time);
  }

  if (abs(timediff - state->leap_seconds) > GLO_SANITY_THRESHOLD_S) {
    /* time too far from rover time, invalidate */
    obs_time->wn = INVALID_TIME;
  }
}

static void validate_base_obs_sanity(struct rtcm3_sbp_state *state,
                                     const gps_time_sec_t *obs_time,
                                     const gps_time_sec_t *rover_time) {
  assert(gps_time_valid(obs_time));
  assert(gps_time_valid(rover_time));
  s32 timediff = gps_diff_time_sec(rover_time, obs_time);

  /* exclude base measurements with time stamp too far in the future */
  if (timediff >= BASE_FUTURE_THRESHOLD_S &&
      state->cb_base_obs_invalid != NULL) {
    state->cb_base_obs_invalid(timediff);
  }
}

bool no_1230_received(struct rtcm3_sbp_state *state) {
  if (!gps_time_valid(&state->last_1230_received) ||
      gps_diff_time_sec(&state->time_from_rover_obs,
                        &state->last_1230_received) > MSG_1230_TIMEOUT_SEC) {
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

  memcpy(&message[preamble_size], msg_1029->utf8_code_units, message_size);
  /* Check if we've had to truncate the string - we can check for the bit
   * pattern that denotes a 4 byte code unit as it is the super set of all bit
   * patterns (2,3 and 4 byte code units) */
  if (message_size == max_message_size) {
    if ((message[message_size] & 0xF0) == 0xF0 ||
        (message[message_size] & 0xE0) == 0xF0 ||
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
                          const uint16_t length,
                          const uint16_t stn_id,
                          const struct rtcm3_sbp_state *state) {
  u8 frame_buffer[SBP_FRAMING_MAX_PAYLOAD_SIZE];
  msg_log_t *sbp_log_msg = (msg_log_t *)frame_buffer;
  sbp_log_msg->level = level;
  memcpy(sbp_log_msg->text, message, length);
  state->cb_rtcm_to_sbp(SBP_MSG_LOG,
                        sizeof(*sbp_log_msg) + length,
                        (u8 *)frame_buffer,
                        rtcm_2_sbp_sender_id(stn_id));
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
  constellation_t cons = to_constellation(new_rtcm_obs->header.msg_num);

  gps_time_sec_t obs_time;
  if (CONSTELLATION_GLO == cons) {
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

    if (CONSTELLATION_BDS2 == cons) {
      /* BDS system time has a constant offset */
      tow_ms += BDS_SECOND_TO_GPS_SECOND * SECS_MS;
      if (tow_ms >= SEC_IN_WEEK * S_TO_MS) {
        tow_ms -= SEC_IN_WEEK * S_TO_MS;
      }
    }

    compute_gps_time(tow_ms, &obs_time, &state->time_from_rover_obs, state);
  }

  if (!gps_time_valid(&state->last_gps_time) ||
      gps_diff_time_sec(&obs_time, &state->last_gps_time) >= 0) {
    if (!gps_time_valid(&state->last_msm_received) &&
        gps_time_valid(&state->last_gps_time)) {
      /* First MSM observation but last_gps_time is already set: possibly
       * switched to MSM from legacy stream, so clear the buffer to avoid
       * duplicate observations */
      memset(state->obs_buffer, 0, OBS_BUFFER_SIZE);
    }
    state->last_gps_time = obs_time;
    state->last_glo_time = obs_time;
    state->last_msm_received = obs_time;

    /* Transform the newly received obs to sbp */
    u8 new_obs[OBS_BUFFER_SIZE];
    memset(new_obs, 0, OBS_BUFFER_SIZE);
    msg_obs_t *new_sbp_obs = (msg_obs_t *)(new_obs);

    /* Find the buffer of obs to be sent */
    msg_obs_t *sbp_obs_buffer = (msg_obs_t *)state->obs_buffer;

    /* Build an SBP time stamp */
    new_sbp_obs->header.t.wn = obs_time.wn;
    new_sbp_obs->header.t.tow = obs_time.tow * S_TO_MS;
    new_sbp_obs->header.t.ns_residual = 0;

    rtcm3_msm_to_sbp(new_rtcm_obs, new_sbp_obs, state);

    /* Check if the buffer already has obs of the same time */
    if (sbp_obs_buffer->header.n_obs != 0 &&
        (sbp_obs_buffer->header.t.tow != new_sbp_obs->header.t.tow ||
         state->sender_id !=
             rtcm_2_sbp_sender_id(new_rtcm_obs->header.stn_id))) {
      /* We either have missed a message, or we have a new station. Either way,
       send through the current buffer and clear before adding new obs */
      send_buffer_not_empty_warning(state);
      send_observations(state);
    }

    /* Copy new obs into buffer */
    u8 obs_index_buffer = sbp_obs_buffer->header.n_obs;
    state->sender_id = rtcm_2_sbp_sender_id(new_rtcm_obs->header.stn_id);
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
  code_t code = msm_signal_to_code(header, signal_index);
  u8 sat = msm_sat_to_prn(header, satellite_index);
  if (CODE_INVALID != code && PRN_INVALID != sat) {
    sid->code = code;
    sid->sat = sat;
    return true;
  } else {
    if (CODE_INVALID == code) {
      /* should have specific code warning but this requires modifiying librtcm
       */
      send_unsupported_code_warning(UNSUPPORTED_CODE_UNKNOWN, state);
    }
    return false;
  }
}

bool unsupported_signal(sbp_gnss_signal_t *sid) {
  switch (sid->code) {
    case CODE_GPS_L5I:
    case CODE_GPS_L5X:
    case CODE_GPS_L5Q:
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
    case CODE_GAL_E8:
    case CODE_QZS_L1CA:
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
      return true;
    default:
      return false;
  }
}

void rtcm3_msm_to_sbp(const rtcm_msm_message *msg,
                      msg_obs_t *new_sbp_obs,
                      struct rtcm3_sbp_state *state) {
  uint8_t num_sats =
      count_mask_values(MSM_SATELLITE_MASK_SIZE, msg->header.satellite_mask);
  uint8_t num_sigs =
      count_mask_values(MSM_SIGNAL_MASK_SIZE, msg->header.signal_mask);

  u8 cell_index = 0;
  for (u8 sat = 0; sat < num_sats; sat++) {
    for (u8 sig = 0; sig < num_sigs; sig++) {
      if (msg->header.cell_mask[sat * num_sigs + sig]) {
        sbp_gnss_signal_t sid;
        const rtcm_msm_signal_data *data = &msg->signals[cell_index];
        if (get_sid_from_msm(&msg->header, sat, sig, &sid, state) &&
            data->flags.valid_pr && data->flags.valid_cp &&
            !unsupported_signal(&sid)) {
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

          sbp_freq->sid = sid;

          if (data->flags.valid_pr) {
            sbp_freq->P =
                (u32)roundl(data->pseudorange_m * MSG_OBS_P_MULTIPLIER);
            sbp_freq->flags |= MSG_OBS_FLAGS_CODE_VALID;
          }
          if (data->flags.valid_cp) {
            sbp_freq->L.i = (s32)floor(data->carrier_phase_cyc);
            u16 frac_part =
                (u16)roundl((data->carrier_phase_cyc - (double)sbp_freq->L.i) *
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
            sbp_freq->cn0 = (u8)roundl(data->cnr * MSG_OBS_CN0_MULTIPLIER);
          } else {
            sbp_freq->cn0 = 0;
          }

          if (data->flags.valid_lock) {
            sbp_freq->lock = encode_lock_time(data->lock_time_s);
          }

          if (data->flags.valid_dop) {
            /* flip Doppler sign to Piksi sign convention */
            double doppler_Hz = -data->range_rate_Hz;
            sbp_freq->D.i = (s16)floor(doppler_Hz);
            u16 frac_part = (u16)roundl((doppler_Hz - (double)sbp_freq->D.i) *
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
