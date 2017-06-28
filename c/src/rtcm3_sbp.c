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

#include <rtcm3_sbp.h>
#include <rtcm3_sbp_interface.h>
#include <rtcm3_decode.h>
#include <rtcm3_messages.h>
#include <sbp.h>
#include <libsbp/gnss.h>
#include <libsbp/observation.h>
#include <libsbp/navigation.h>
#include <math.h>
#include <string.h>


struct rtcm3_sbp_state {
  gps_time_sec_t time_from_rover_obs;
  bool gps_time_updated;
  s8 current_leap_seconds;
  msg_obs_t *sbp_obs_buffer;
  u8 obs_buffer[sizeof(observation_header_t) + MAX_OBS_PER_EPOCH * sizeof(packed_obs_content_t)];
  void (*cb)(u8 msg_id, u8 buff, u8 *len);
};

void init(struct rtcm3_sbp_state *state, void (*cb)(u8 msg_id, u8 buff, u8 *len)) {
  state->time_from_rover_obs.wn = 0;
  state->time_from_rover_obs.tow = 0;
  state->gps_time_updated = false;

  state->sbp_obs_buffer = (msg_obs_t *)(state->obs_buffer);
  state->cb = cb;
}

static double gps_diff_time(const gps_time_sec_t *end, const gps_time_sec_t *beginning)
{
  int week_diff = end->wn - beginning->wn;
  double dt = (double)end->tow - (double)beginning->tow;
  dt += week_diff * 604800;
  return dt;
}

void rtcm3_decode_frame(const uint8_t *frame, uint32_t frame_length, struct rtcm3_sbp_state *state) {

  if (!state->gps_time_updated || frame_length < 1) {
    return;
  }

  uint16_t byte = 1;
  uint16_t message_size = ((frame[byte] & 0x3) << 8) | frame[byte + 1];

  if(frame_length < message_size) {
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
    if (rtcm3_decode_1002(&frame[byte], &new_rtcm_obs) == 0 ) {
      // Need to check if we've got obs in the buffer from the previous epoch and send before accepting the new message
      add_gps_obs_to_buffer(&new_rtcm_obs, state);
    }
    break;
  }
  case 1004: {
    rtcm_obs_message new_rtcm_obs;
    if (rtcm3_decode_1004(&frame[byte], &new_rtcm_obs) == 0 ) {
      // Need to check if we've got obs in the buffer from the previous epoch and send before accepting the new message
      add_gps_obs_to_buffer(&new_rtcm_obs, state);
    }
    break;
  }
  case 1005: {
    rtcm_msg_1005 rtcm_msg_1005;
    if (0 == rtcm3_decode_1005(&frame[byte], &rtcm_msg_1005)) {
      msg_base_pos_ecef_t sbp_base_pos;
      rtcm3_1005_to_sbp(&rtcm_msg_1005, &sbp_base_pos);
      state->cb(SBP_MSG_BASE_POS_ECEF, (u8)sizeof(sbp_base_pos), (u8 *)&sbp_base_pos);
    }
    break;
  }
  case 1006: {
    rtcm_msg_1006 rtcm_msg_1006;
    if (0 == rtcm3_decode_1006(&frame[byte], &rtcm_msg_1006)) {
      msg_base_pos_ecef_t sbp_base_pos;
      rtcm3_1006_to_sbp(&rtcm_msg_1006, &sbp_base_pos);
      state->cb(SBP_MSG_BASE_POS_ECEF, (u8)sizeof(sbp_base_pos), (u8 *)&sbp_base_pos);
    }
    break;
  }
  case 1007:
  case 1008:
    break;
//  case 1010: {
//    rtcm_obs_message new_rtcm_obs;
//    if (rtcm3_decode_1010(&frame[byte], &new_rtcm_obs) == 0 ) {
//      // Need to check if we've got obs in the buffer from the previous epoch and send before accepting the new message
//      convert_glo_to_gps_time(&new_rtcm_obs);
//      add_obs_to_buffer(&rtcm_msg, &new_rtcm_obs);
//      if (rtcm_msg.header.sync == 0) {
//        encode_RTCM_obs(&rtcm_msg);
//      }
//    }
//    break;
//  }
//  case 1012: {
//    rtcm_obs_message new_rtcm_obs;
//    if (rtcm3_decode_1012(&frame[byte], &new_rtcm_obs) == 0 ) {
//      // Need to check if we've got obs in the buffer from the previous epoch and send before accepting the new message
//      convert_glo_to_gps_time(&new_rtcm_obs);
//      add_obs_to_buffer(&rtcm_msg, &new_rtcm_obs);
//      if (rtcm_msg.header.sync == 0) {
//        encode_RTCM_obs(&rtcm_msg);
//      }
//    }
//    break;
//  }
  default:
    break;
  }

}

void add_gps_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs, struct rtcm3_sbp_state *state) {

  gps_time_sec_t obs_time;
  compute_gps_time(new_rtcm_obs->header.tow, &obs_time, &state->time_from_rover_obs);

  add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
}

void add_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs, gps_time_sec_t *obs_time, struct rtcm3_sbp_state *state) {

  // Transform the newly received obs to sbp
  msg_obs_t *new_sbp_obs;
  u8 new_obs[sizeof(observation_header_t) + MAX_OBS_PER_EPOCH * sizeof(packed_obs_content_t)];
  new_sbp_obs = (msg_obs_t * )(new_obs);

  // Find the buffer of obs to be sent
  msg_obs_t *sbp_obs_buffer = state->sbp_obs_buffer;

  new_sbp_obs->header.t.wn = obs_time->wn;
  new_sbp_obs->header.t.tow = obs_time->tow * 1000;
  new_sbp_obs->header.t.ns_residual = 0;

  rtcm3_to_sbp(new_rtcm_obs, new_sbp_obs);

  // Check if the buffer already has obs of the same time
  if(sbp_obs_buffer->header.n_obs != 0 &&
     (sbp_obs_buffer->header.t.tow != new_rtcm_obs->header.tow * 1000)) {
    // We either have missed a message, or we have a new station. Either way,
    // send through the current buffer and clear before adding new obs
    send_observations(state);
  }
  
  // Copy new obs into buffer
  u8 obs_index_buffer = sbp_obs_buffer->header.n_obs;
  for(u8 obs_count = 0; obs_count < new_sbp_obs->header.n_obs; obs_count++) {
    sbp_obs_buffer->obs[obs_index_buffer] = new_sbp_obs->obs[obs_count];
    obs_index_buffer++;
  }

  // If we aren't expecting another message, send the buffer
  if (new_rtcm_obs->header.sync == 0) {
    send_observations(state);
  }
}

/**
 * Send the sbp message obs buffer
 */
void send_observations(struct rtcm3_sbp_state *state) {
  const msg_obs_t *sbp_obs_buffer = state->sbp_obs_buffer;

  u8 obs_count = 0;
  u8 sizes[4];
  u8 obs_data[4 * sizeof(observation_header_t) +
              4 * sbp_obs_buffer->header.n_obs * sizeof(packed_obs_content_t)];
  msg_obs_t *sbp_obs[4];
  u8 msg_num = 0;
  for (msg_num = 0; msg_num < 4; ++msg_num) {
    sbp_obs[msg_num] =
      (msg_obs_t * )(obs_data + (msg_num * sizeof(observation_header_t) +
                                 MAX_OBS_IN_SBP * msg_num *
                                 sizeof(packed_obs_content_t)));
    sbp_obs[msg_num]->header = sbp_obs_buffer->header;
    u8 obs_index;
    for (obs_index = 0; obs_index < MAX_OBS_IN_SBP && obs_count < sbp_obs_buffer->header.n_obs; obs_index++) {
      sbp_obs[msg_num]->obs[obs_index] = sbp_obs_buffer->obs[obs_count++];
    }
    sizes[msg_num] = sizeof(observation_header_t) + obs_index * sizeof(packed_obs_content_t);
  }

  for (u8 msg = 0; msg < msg_num; ++msg) {
    state->cb(SBP_MSG_OBS, sizes[msg], (u8 *)sbp_obs[msg]);
  }
  memset((void*)sbp_obs_buffer,0, sizeof(*sbp_obs_buffer));
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
//  assert(nm_lock_time >= 0.0);

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

void rtcm3_to_sbp(const rtcm_obs_message *rtcm_obs, msg_obs_t *new_sbp_obs) {

  new_sbp_obs->header.n_obs = 0;
  for (u8 sat = 0; sat < rtcm_obs->header.n_sat; ++sat) {
    for (u8 freq = 0; freq < NUM_FREQS; ++freq) {
      const rtcm_freq_data *rtcm_freq = &rtcm_obs->sats[sat].obs[freq];
      if (rtcm_freq->flags.valid_pr == 1 && rtcm_freq->flags.valid_cp == 1) {

        packed_obs_content_t *sbp_freq =
          &new_sbp_obs->obs[new_sbp_obs->header.n_obs];
        sbp_freq->sid.sat = rtcm_obs->sats[sat].svId;
        sbp_freq->sid.code = rtcm_freq->code;
        sbp_freq->flags = 0;
        sbp_freq->P = 0.0;
        sbp_freq->L.i = 0;
        sbp_freq->L.f = 0.0;
        sbp_freq->D.i = 0;
        sbp_freq->D.f = 0.0;
        sbp_freq->cn0 = 0.0;
        sbp_freq->lock = 0.0;
        if (freq == L1_FREQ) {
          if (rtcm_freq->code == 0) {
            sbp_freq->sid.code = CODE_GPS_L1CA;
          } else {
            sbp_freq->sid.code = CODE_GPS_L1P;
          }
        } else if (freq == L2_FREQ) {
          if (rtcm_freq->code == 0) {
            sbp_freq->sid.code = CODE_GPS_L2CM;
          } else {
            sbp_freq->sid.code = CODE_GPS_L2P;
          }
        }

        if (rtcm_freq->flags.valid_pr == 1) {
          sbp_freq->P =
            (u32)roundl(rtcm_freq->pseudorange * MSG_OBS_P_MULTIPLIER);
          sbp_freq->flags |= MSG_OBS_FLAGS_CODE_VALID;
        }
        if (rtcm_freq->flags.valid_cp == 1) {
          sbp_freq->L.i = (s32)floor(rtcm_freq->carrier_phase);
          u16 frac_part = (u16)roundl((rtcm_freq->carrier_phase - (double)sbp_freq->L.i) *
                                      MSG_OBS_LF_MULTIPLIER);
          if( frac_part == 256 ) {
            frac_part -= 256;
            sbp_freq->L.i += 1;
          }
          sbp_freq->L.f = (u8) frac_part;
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

void sbp_to_rtcm3_obs(const msg_obs_t *sbp_obs, const u8 msg_size,
                      rtcm_obs_message *rtcm_obs) {
  rtcm_obs->header.tow = sbp_obs->header.t.tow / 1000.0;

  u8 count = rtcm_obs->header.n_sat;
  s8 sat2index[32];
  for (u8 idx = 0; idx < 32; ++idx) {
    sat2index[idx] = -1;
  }

  for (u8 idx = 0; idx < rtcm_obs->header.n_sat; ++idx) {
    sat2index[idx] = rtcm_obs->sats[idx].svId;
  }

  u8 num_obs =
      (msg_size - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);
  for (u8 obs = 0; obs < num_obs; ++obs) {
    const packed_obs_content_t *sbp_freq = &sbp_obs->obs[obs];
    if (sbp_freq->flags != 0) {
      s8 sat_idx = sat2index[sbp_freq->sid.sat];
      if (sat_idx == -1) {
        sat_idx = sat2index[sbp_freq->sid.sat] = count++;
        rtcm_obs->sats[sat_idx].svId = sbp_freq->sid.sat;
      }
      // freq <-> code???
      rtcm_freq_data *rtcm_freq =
          &rtcm_obs->sats[sat_idx].obs[sbp_freq->sid.code];
      rtcm_freq->flags.data = 0;
      rtcm_freq->code = 0;
      rtcm_freq->pseudorange = 0.0;
      rtcm_freq->carrier_phase = 0.0;
      rtcm_freq->cnr = 0.0;
      rtcm_freq->lock = 0.0;

      if (sbp_freq->flags & MSG_OBS_FLAGS_CODE_VALID) {
        rtcm_freq->pseudorange = sbp_freq->P / MSG_OBS_P_MULTIPLIER;
        rtcm_freq->flags.valid_pr = 1;
      }
      if ((sbp_freq->flags & MSG_OBS_FLAGS_PHASE_VALID) &&
          (sbp_freq->flags & MSG_OBS_FLAGS_HALF_CYCLE_KNOWN)) {
        rtcm_freq->carrier_phase =
            sbp_freq->L.i + (double)sbp_freq->L.f / MSG_OBS_LF_MULTIPLIER;
        rtcm_freq->flags.valid_cp = 1;
      }

      rtcm_freq->cnr = sbp_freq->cn0 / MSG_OBS_CN0_MULTIPLIER;
      rtcm_freq->flags.valid_cnr = 1;

      rtcm_freq->lock = decode_lock_time(sbp_freq->lock);
      rtcm_freq->flags.valid_lock = 1;
    }
  }
  rtcm_obs->header.n_sat = count;
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

void set_gps_time(gps_time_sec_t *current_time, struct rtcm3_sbp_state* state)
{
  state->time_from_rover_obs.wn = current_time->wn;
  state->time_from_rover_obs.tow = current_time->tow;
  state->gps_time_updated = true;
}

void compute_gps_time(double tow, gps_time_sec_t *obs_time, const gps_time_sec_t *rover_time) {

  obs_time->tow = tow;
  obs_time->wn = rover_time->wn;
  double timediff = gps_diff_time(obs_time, rover_time);
  if (timediff < -302400) {
    obs_time->wn = rover_time->wn + 1;
  } else if (timediff > 302400) {
    obs_time->wn = rover_time->wn - 1;
  }
}
