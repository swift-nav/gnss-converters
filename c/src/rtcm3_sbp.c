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

#include "rtcm3_sbp_internal.h"
#include <assert.h>
#include <math.h>
#include <rtcm3_decode.h>
#include <string.h>

static void validate_base_obs_sanity(struct rtcm3_sbp_state *state,
                                     gps_time_sec_t *obs_time,
                                     const gps_time_sec_t *rover_time);

void rtcm2sbp_init(struct rtcm3_sbp_state *state,
                   void (*cb_rtcm_to_sbp)(u16 msg_id, u8 length, u8 *buffer,
                                          u16 sender_id),
                   void (*cb_base_obs_invalid)(double timediff)) {
  state->time_from_rover_obs.wn = 0;
  state->time_from_rover_obs.tow = 0;
  state->gps_time_updated = false;

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

  state->sent_msm_warning = false;

  const msg_obs_t *sbp_obs_buffer = (msg_obs_t *)state->obs_buffer;
  memset((void *)sbp_obs_buffer, 0, sizeof(*sbp_obs_buffer));
}

static double gps_diff_time(const gps_time_sec_t *end,
                            const gps_time_sec_t *beginning) {
  int week_diff = end->wn - beginning->wn;
  double dt = (double)end->tow - (double)beginning->tow;
  dt += week_diff * SEC_IN_WEEK;
  return dt;
}

static u16 rtcm_2_sbp_sender_id(u16 rtcm_id) {
  /* To avoid conflicts with reserved low number sender ID's we or
   * on the highest nibble as RTCM sender ID's are 12 bit */
  return rtcm_id | 0xF000;
}

void rtcm2sbp_decode_frame(const uint8_t *frame, uint32_t frame_length,
                           struct rtcm3_sbp_state *state) {
  if (state->gps_time_updated == false || frame_length < 1) {
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
    if (rtcm3_decode_1002(&frame[byte], &new_rtcm_obs) == 0) {
      // Need to check if we've got obs in the buffer from the previous epoch
      // and send before accepting the new message
      add_gps_obs_to_buffer(&new_rtcm_obs, state);
    }
    break;
  }
  case 1004: {
    rtcm_obs_message new_rtcm_obs;
    if (rtcm3_decode_1004(&frame[byte], &new_rtcm_obs) == 0) {
      // Need to check if we've got obs in the buffer from the previous epoch
      // and send before accepting the new message
      add_gps_obs_to_buffer(&new_rtcm_obs, state);
    }
    break;
  }
  case 1005: {
    rtcm_msg_1005 msg_1005;
    if (0 == rtcm3_decode_1005(&frame[byte], &msg_1005)) {
      msg_base_pos_ecef_t sbp_base_pos;
      rtcm3_1005_to_sbp(&msg_1005, &sbp_base_pos);
      state->cb_rtcm_to_sbp(SBP_MSG_BASE_POS_ECEF, (u8)sizeof(sbp_base_pos),
                            (u8 *)&sbp_base_pos,
                            rtcm_2_sbp_sender_id(msg_1005.stn_id));
    }
    break;
  }
  case 1006: {
    rtcm_msg_1006 msg_1006;
    if (0 == rtcm3_decode_1006(&frame[byte], &msg_1006)) {
      msg_base_pos_ecef_t sbp_base_pos;
      rtcm3_1006_to_sbp(&msg_1006, &sbp_base_pos);
      state->cb_rtcm_to_sbp(SBP_MSG_BASE_POS_ECEF, (u8)sizeof(sbp_base_pos),
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
    if (rtcm3_decode_1010(&frame[byte], &new_rtcm_obs) == 0 &&
        state->leap_second_known) {
      add_glo_obs_to_buffer(&new_rtcm_obs, state);
    }
    break;
  }
  case 1012: {
    rtcm_obs_message new_rtcm_obs;
    if (rtcm3_decode_1012(&frame[byte], &new_rtcm_obs) == 0 &&
        state->leap_second_known) {
      add_glo_obs_to_buffer(&new_rtcm_obs, state);
    }
    break;
  }
  case 1029: {
    rtcm_msg_1029 msg_1029;
    if (rtcm3_decode_1029(&frame[byte], &msg_1029) == 0) {
      send_1029(&msg_1029, state);
    }
  }
  case 1033: {
    rtcm_msg_1033 msg_1033;
    if (rtcm3_decode_1033(&frame[byte], &msg_1033) == 0 &&
        no_1230_received(state)) {
      msg_glo_biases_t sbp_glo_cpb;
      rtcm3_1033_to_sbp(&msg_1033, &sbp_glo_cpb);
      state->cb_rtcm_to_sbp(SBP_MSG_GLO_BIASES, (u8)sizeof(sbp_glo_cpb),
                            (u8 *)&sbp_glo_cpb,
                            rtcm_2_sbp_sender_id(msg_1033.stn_id));
    }
    break;
  }
  case 1230: {
    rtcm_msg_1230 msg_1230;
    if (rtcm3_decode_1230(&frame[byte], &msg_1230) == 0) {
      msg_glo_biases_t sbp_glo_cpb;
      rtcm3_1230_to_sbp(&msg_1230, &sbp_glo_cpb);
      state->cb_rtcm_to_sbp(SBP_MSG_GLO_BIASES, (u8)sizeof(sbp_glo_cpb),
                            (u8 *)&sbp_glo_cpb,
                            rtcm_2_sbp_sender_id(msg_1230.stn_id));
      state->last_1230_received = state->time_from_rover_obs;
    }
  }
  case 1071:
  case 1072:
  case 1073:
  case 1074:
  case 1075:
  case 1076:
  case 1077:
  case 1081:
  case 1082:
  case 1083:
  case 1084:
  case 1085:
  case 1086:
  case 1087: {
    /* MSM messages for GPS (1071-1077) and GLO (1081-1087) are currently not
     * supported, warn the user once if these messages are seen - only warn once
     * as these messages can be present in streams that contain 1004 and 1012 so
     * are valid */
    send_MSM_warning(&frame[byte], state);
  }
  default:
    break;
  }
}

void add_glo_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  gps_time_sec_t obs_time;
  compute_glo_time(new_rtcm_obs->header.tow_ms, &obs_time,
                   &state->time_from_rover_obs, state->leap_seconds);

  if (state->last_gps_time.wn == INVALID_TIME ||
      gps_diff_time(&obs_time, &state->last_gps_time) > 0.0) {
    state->last_gps_time.wn = obs_time.wn;
    state->last_gps_time.tow = obs_time.tow;
    add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
  }
}

void add_gps_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  gps_time_sec_t obs_time;
  compute_gps_time(new_rtcm_obs->header.tow_ms, &obs_time,
                   &state->time_from_rover_obs, state);

  if (state->last_glo_time.wn == INVALID_TIME ||
      gps_diff_time(&obs_time, &state->last_gps_time) > 0.0) {
    state->last_glo_time.wn = obs_time.wn;
    state->last_glo_time.tow = obs_time.tow;
    add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
  }
}

void add_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                       gps_time_sec_t *obs_time,
                       struct rtcm3_sbp_state *state) {
  // Transform the newly received obs to sbp
  u8 new_obs[sizeof(observation_header_t) +
             MAX_OBS_PER_EPOCH * sizeof(packed_obs_content_t)];
  msg_obs_t *new_sbp_obs = (msg_obs_t *)(new_obs);
  memset((void *)new_sbp_obs, 0, sizeof(*new_sbp_obs));

  // Find the buffer of obs to be sent
  msg_obs_t *sbp_obs_buffer = (msg_obs_t *)state->obs_buffer;

  new_sbp_obs->header.t.wn = obs_time->wn;
  new_sbp_obs->header.t.tow = obs_time->tow * S_TO_MS;
  new_sbp_obs->header.t.ns_residual = 0;

  rtcm3_to_sbp(new_rtcm_obs, new_sbp_obs);

  // Check if the buffer already has obs of the same time
  if (sbp_obs_buffer->header.n_obs != 0 &&
      (sbp_obs_buffer->header.t.tow != new_sbp_obs->header.t.tow ||
       state->sender_id != rtcm_2_sbp_sender_id(new_rtcm_obs->header.stn_id))) {
    // We either have missed a message, or we have a new station. Either way,
    // send through the current buffer and clear before adding new obs
    send_observations(state);
  }

  // Copy new obs into buffer
  u8 obs_index_buffer = sbp_obs_buffer->header.n_obs;
  state->sender_id = rtcm_2_sbp_sender_id(new_rtcm_obs->header.stn_id);
  for (u8 obs_count = 0; obs_count < new_sbp_obs->header.n_obs; obs_count++) {
    sbp_obs_buffer->obs[obs_index_buffer] = new_sbp_obs->obs[obs_count];
    obs_index_buffer++;
  }
  sbp_obs_buffer->header.n_obs = obs_index_buffer;
  sbp_obs_buffer->header.t = new_sbp_obs->header.t;

  // If we aren't expecting another message, send the buffer
  if (new_rtcm_obs->header.sync == 0) {
    send_observations(state);
  }
}

/**
 * Send the sbp message obs buffer
 */
void send_observations(struct rtcm3_sbp_state *state) {
  const msg_obs_t *sbp_obs_buffer = (msg_obs_t *)state->obs_buffer;

  /* Work out how many sbp messages we need to split this into */
  const uint32_t header_size = sizeof(observation_header_t);
  const uint32_t obs_size = sizeof(packed_obs_content_t);
  const uint32_t max_obs_in_sbp =
      ((SBP_FRAMING_MAX_PAYLOAD_SIZE - header_size) / obs_size);

  /* We want the ceiling of n_obs divided by max obs in a single message to get
   * total number of messages needed */
  const u8 total_messages =
      1 + ((sbp_obs_buffer->header.n_obs - 1) / max_obs_in_sbp);

  u8 obs_count = 0;
  u8 obs_data[SBP_FRAMING_MAX_PAYLOAD_SIZE];
  msg_obs_t *sbp_obs = (msg_obs_t *)obs_data;

  for (u8 msg_num = 0; msg_num < total_messages; ++msg_num) {
    memset(obs_data, 0, SBP_FRAMING_MAX_PAYLOAD_SIZE);

    u8 obs_index = 0;
    while (obs_index < max_obs_in_sbp &&
           obs_count < sbp_obs_buffer->header.n_obs) {
      sbp_obs->obs[obs_index++] = sbp_obs_buffer->obs[obs_count++];
    }

    sbp_obs->header.t = sbp_obs_buffer->header.t;
    sbp_obs->header.n_obs = (total_messages << 4) + msg_num;

    state->cb_rtcm_to_sbp(SBP_MSG_OBS, header_size + obs_index * obs_size,
                          (u8 *)sbp_obs, state->sender_id);
  }
  memset((void *)sbp_obs_buffer, 0, sizeof(*sbp_obs_buffer));
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

code_t get_glo_sbp_code(u8 freq, u8 rtcm_code) {
  (void)rtcm_code;
  code_t code = CODE_INVALID;
  if (freq == L1_FREQ) {
    code = CODE_GLO_L1CA;
    /* CODE_GLO_L1P currently not supported in sbp */
  } else if (freq == L2_FREQ) {
    code = CODE_GLO_L2CA;
    /* CODE_GLO_L2P currently not supported in sbp */
  }
  return code;
}

void rtcm3_to_sbp(const rtcm_obs_message *rtcm_obs, msg_obs_t *new_sbp_obs) {
  for (u8 sat = 0; sat < rtcm_obs->header.n_sat; ++sat) {
    for (u8 freq = 0; freq < NUM_FREQS; ++freq) {
      const rtcm_freq_data *rtcm_freq = &rtcm_obs->sats[sat].obs[freq];
      if (rtcm_freq->flags.valid_pr == 1 && rtcm_freq->flags.valid_cp == 1) {

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
          sbp_freq->sid.code =
              get_gps_sbp_code(freq, rtcm_obs->sats[sat].obs[freq].code);
        } else if (glo_obs_message(rtcm_obs->header.msg_num)) {
          sbp_freq->sid.code =
              get_glo_sbp_code(freq, rtcm_obs->sats[sat].obs[freq].code);
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

void rtcm2sbp_set_gps_time(gps_time_sec_t *current_time,
                           struct rtcm3_sbp_state *state) {
  state->time_from_rover_obs.wn = current_time->wn;
  state->time_from_rover_obs.tow = current_time->tow;
  state->gps_time_updated = true;
}

void rtcm2sbp_set_leap_second(s8 leap_seconds, struct rtcm3_sbp_state *state) {
  state->leap_seconds = leap_seconds;
  state->leap_second_known = true;
}

void compute_gps_time(double tow_ms, gps_time_sec_t *obs_time,
                      const gps_time_sec_t *rover_time,
                      struct rtcm3_sbp_state *state) {
  obs_time->tow = tow_ms * MS_TO_S;
  obs_time->wn = rover_time->wn;
  double timediff = gps_diff_time(obs_time, rover_time);
  if (timediff < -SEC_IN_WEEK / 2) {
    obs_time->wn = rover_time->wn + 1;
  } else if (timediff > SEC_IN_WEEK / 2) {
    obs_time->wn = rover_time->wn - 1;
  }
  validate_base_obs_sanity(state, obs_time, rover_time);
}

void compute_glo_time(double tod_ms, gps_time_sec_t *obs_time,
                      const gps_time_sec_t *rover_time, const s8 leap_second) {
  /* Need to work out DOW from GPS time first */
  int rover_dow = rover_time->tow / SEC_IN_DAY;
  int rover_tod = rover_time->tow - rover_dow * SEC_IN_DAY;

  double glo_tod_sec = tod_ms * MS_TO_S - 3 * SEC_IN_HOUR + leap_second;

  if (glo_tod_sec < 0) {
    glo_tod_sec += SEC_IN_DAY;
  }

  obs_time->wn = rover_time->wn;
  /* Check for day rollover */
  if (glo_tod_sec > rover_tod && glo_tod_sec - rover_tod > SEC_IN_DAY / 2) {
    rover_dow = (rover_dow + 1) % 7;
  } else if (rover_tod > glo_tod_sec &&
             rover_tod - glo_tod_sec > SEC_IN_DAY / 2) {
    rover_dow = (rover_dow - 1) % 7;
  }

  obs_time->tow = rover_dow * SEC_IN_DAY + glo_tod_sec;
}

static const double INSANITY_THRESHOLD = 10.0;

static void validate_base_obs_sanity(struct rtcm3_sbp_state *state,
                                     gps_time_sec_t *obs_time,
                                     const gps_time_sec_t *rover_time) {

  double timediff = gps_diff_time(rover_time, obs_time);

  if (timediff >= INSANITY_THRESHOLD && state->cb_base_obs_invalid != NULL) {
    state->cb_base_obs_invalid(timediff);
  }
}

bool no_1230_received(struct rtcm3_sbp_state *state) {
  if (gps_diff_time(&state->time_from_rover_obs, &state->last_1230_received) >
      MSG_1230_TIMEOUT_SEC) {
    return true;
  }
  return false;
}

void send_1029(rtcm_msg_1029 *msg_1029, struct rtcm3_sbp_state *state) {
  uint8_t message[SBP_FRAMING_MAX_PAYLOAD_SIZE] = RTCM_LOG_PREAMBLE;
  uint8_t preamble_size = sizeof(RTCM_LOG_PREAMBLE) - 1;
  uint8_t max_message_size = SBP_FRAMING_MAX_PAYLOAD_SIZE - sizeof(msg_log_t) - preamble_size;
  uint8_t message_size = sizeof(msg_log_t) + msg_1029->utf8_code_units_n + preamble_size >
                         SBP_FRAMING_MAX_PAYLOAD_SIZE ? max_message_size
                                                      : msg_1029->utf8_code_units_n + preamble_size;

  memcpy(&message[preamble_size],msg_1029->utf8_code_units,message_size);
  /* Check if we've had to truncate the string - we can check for the bit
   * pattern that denotes a 4 byte code unit as it is the super set of all bit
   * patterns (2,3 and 4 byte code units) */
  if (message_size == max_message_size) {
    if ((message[message_size] & 0xF0) == 0xF0 ||
        (message[message_size] & 0xE0) == 0xF0 ||
        (message[message_size] & 0xF0) == 0xC0) {
      // We've truncated a 2, 3 or 4 byte code unit
      message_size--;
    } else if ((message[message_size - 1] & 0xF0) == 0xF0 ||
               (message[message_size - 1] & 0xE0) == 0xE0) {
      // We've truncated a 3 or 4 byte code unit
      message_size -= 2;
    } else if ((message[message_size - 1] & 0xF0) == 0xF0) {
      // We've truncated a 4 byte code unit
      message_size -= 3;
    }
  }

  send_sbp_log_message(RTCM_1029_LOGGING_LEVEL, message, message_size,
                       msg_1029->stn_id, state);
}

void send_sbp_log_message(const uint8_t level, const uint8_t *message,
                          const uint16_t length, const uint16_t stn_id,
                          struct rtcm3_sbp_state *state) {
  u8 frame_buffer[SBP_FRAMING_MAX_PAYLOAD_SIZE];
  msg_log_t *sbp_log_msg = (msg_log_t *)frame_buffer;
  sbp_log_msg->level = level;
  memcpy(sbp_log_msg->text, message, length);
  state->cb_rtcm_to_sbp(SBP_MSG_LOG, sizeof(*sbp_log_msg) + length,
                        (u8 *)frame_buffer, rtcm_2_sbp_sender_id(stn_id));
}

void send_MSM_warning(const uint8_t *frame, struct rtcm3_sbp_state *state) {

  if (!state->sent_msm_warning) {
    // Only send 1 warning
    state->sent_msm_warning = true;
    // Get the stn ID as well
    uint32_t stn_id = 0;
    for (uint32_t i = 12; i < 24; i++) {
      stn_id = (stn_id << 1) + ((frame[i / 8] >> (7 - i % 8)) & 1u);
    }
    uint8_t msg[37] = "MSM Messages currently not supported";
    send_sbp_log_message(RTCM_MSM_LOGGING_LEVEL, msg, sizeof(msg), stn_id,
                         state);
  }
}
