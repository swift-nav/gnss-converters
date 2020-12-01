/*
 * Copyright (C) 2018 Swift Navigation Inc.
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
#include <libsbp/ssr.h>
#include <math.h>
#include <rtcm3/msm_utils.h>
#include <string.h>
#include "rtcm3_sbp_internal.h"

#define SSR_MESSAGE_LENGTH 256

gps_time_sec_t compute_ssr_message_time(
    const enum constellation_e constellation,
    u32 epoch_time_ms,
    const gps_time_t *rover_time,
    struct rtcm3_sbp_state *state) {
  gps_time_sec_t obs_time_sec;
  gps_time_t obs_time;
  if (constellation == CONSTELLATION_GLO) {
    compute_glo_time(epoch_time_ms, &obs_time, rover_time, state);
  } else if (constellation == CONSTELLATION_BDS) {
    beidou_tow_to_gps_tow(&epoch_time_ms);
    compute_gps_message_time(epoch_time_ms, &obs_time, rover_time);
  } else {
    /* GAL / QZSS / SBAS are aligned to GPS time */
    compute_gps_message_time(epoch_time_ms, &obs_time, rover_time);
  }
  obs_time_sec.wn = obs_time.wn;
  obs_time_sec.tow = (u32)rint(obs_time.tow);
  return obs_time_sec;
}

static bool rtcm_ssr_header_to_sbp_orbit_clock(
    const rtcm_msg_ssr_header *header,
    uint8_t *length,
    const rtcm_msg_ssr_orbit_corr *orbit,
    msg_ssr_orbit_clock_t *sbp_orbit_clock,
    struct rtcm3_sbp_state *state) {
  sbp_orbit_clock->time = compute_ssr_message_time(header->constellation,
                                                   header->epoch_time * SECS_MS,
                                                   &state->time_from_rover_obs,
                                                   state);

  if (!gps_time_sec_valid(&sbp_orbit_clock->time)) {
    /* Invalid time */
    return false;
  }
  *length += sizeof(sbp_orbit_clock->time);

  sbp_orbit_clock->sid.code = constellation_to_l1_code(header->constellation);

  sbp_orbit_clock->sid.sat = orbit->sat_id;
  *length += sizeof(sbp_orbit_clock->sid);

  sbp_orbit_clock->update_interval = header->update_interval;
  *length += sizeof(sbp_orbit_clock->update_interval);

  sbp_orbit_clock->iod_ssr = header->iod_ssr;
  *length += sizeof(sbp_orbit_clock->iod_ssr);

  sbp_orbit_clock->iod = orbit->iode;
  *length += sizeof(sbp_orbit_clock->iod);

  return true;
}

static void rtcm_ssr_orbit_to_sbp(const rtcm_msg_ssr_orbit_corr *orbit,
                                  uint8_t *length,
                                  msg_ssr_orbit_clock_t *sbp_orbit_clock) {
  sbp_orbit_clock->radial = orbit->radial;
  *length += sizeof(sbp_orbit_clock->radial);

  sbp_orbit_clock->along = orbit->along_track;
  *length += sizeof(sbp_orbit_clock->along);

  sbp_orbit_clock->cross = orbit->cross_track;
  *length += sizeof(sbp_orbit_clock->cross);

  sbp_orbit_clock->dot_radial = orbit->dot_radial;
  *length += sizeof(sbp_orbit_clock->dot_radial);

  sbp_orbit_clock->dot_along = orbit->dot_along_track;
  *length += sizeof(sbp_orbit_clock->dot_along);

  sbp_orbit_clock->dot_cross = orbit->dot_cross_track;
  *length += sizeof(sbp_orbit_clock->dot_cross);
}

static void rtcm_ssr_clock_to_sbp(const rtcm_msg_ssr_clock_corr *clock,
                                  uint8_t *length,
                                  msg_ssr_orbit_clock_t *sbp_orbit_clock) {
  sbp_orbit_clock->c0 = clock->c0;
  *length += sizeof(sbp_orbit_clock->c0);

  sbp_orbit_clock->c1 = clock->c1;
  *length += sizeof(sbp_orbit_clock->c1);

  sbp_orbit_clock->c2 = clock->c2;
  *length += sizeof(sbp_orbit_clock->c2);
}

void rtcm3_ssr_separate_orbit_clock_to_sbp(rtcm_msg_clock *msg_clock,
                                           rtcm_msg_orbit *msg_orbit,
                                           struct rtcm3_sbp_state *state) {
  assert(msg_clock);
  assert(msg_orbit);
  assert(msg_clock->header.constellation == msg_orbit->header.constellation);
  assert(msg_clock->header.epoch_time == msg_orbit->header.epoch_time);
  assert(msg_clock->header.iod_ssr == msg_orbit->header.iod_ssr);

  uint8_t buffer[SSR_MESSAGE_LENGTH];
  uint8_t length;
  msg_ssr_orbit_clock_t *sbp_orbit_clock = (msg_ssr_orbit_clock_t *)buffer;

  for (int sat_count = 0; (sat_count < msg_clock->header.num_sats) &&
                          (sat_count < msg_orbit->header.num_sats);
       ++sat_count) {
    memset(buffer, 0, sizeof(buffer));
    length = 0;

    // We aren't guaranteed that the satellite info in the orbit and clock
    // messages is in the same order so we must search for pair of corrections
    // for a given satellite
    int clock_index = sat_count;
    int orbit_index = 0;
    while (msg_orbit->orbit[orbit_index].sat_id !=
               msg_clock->clock[clock_index].sat_id &&
           orbit_index < msg_orbit->header.num_sats) {
      orbit_index++;
    }
    if (orbit_index >= msg_orbit->header.num_sats) {
      continue;
    }

    if (!rtcm_ssr_header_to_sbp_orbit_clock(&msg_orbit->header,
                                            &length,
                                            &msg_orbit->orbit[orbit_index],
                                            sbp_orbit_clock,
                                            state)) {
      return;
    }
    rtcm_ssr_orbit_to_sbp(
        &msg_orbit->orbit[orbit_index], &length, sbp_orbit_clock);
    rtcm_ssr_clock_to_sbp(
        &msg_clock->clock[clock_index], &length, sbp_orbit_clock);

    state->cb_rtcm_to_sbp(SBP_MSG_SSR_ORBIT_CLOCK,
                          length,
                          (u8 *)sbp_orbit_clock,
                          0,
                          state->context);
  }
}

void rtcm3_ssr_orbit_clock_to_sbp(rtcm_msg_orbit_clock *msg_orbit_clock,
                                  struct rtcm3_sbp_state *state) {
  uint8_t buffer[SSR_MESSAGE_LENGTH];
  uint8_t length;
  msg_ssr_orbit_clock_t *sbp_orbit_clock = (msg_ssr_orbit_clock_t *)buffer;

  for (int sat_count = 0; sat_count < msg_orbit_clock->header.num_sats;
       sat_count++) {
    memset(buffer, 0, SSR_MESSAGE_LENGTH);
    length = 0;

    if (!rtcm_ssr_header_to_sbp_orbit_clock(&msg_orbit_clock->header,
                                            &length,
                                            &msg_orbit_clock->orbit[sat_count],
                                            sbp_orbit_clock,
                                            state)) {
      return;
    }
    rtcm_ssr_orbit_to_sbp(
        &msg_orbit_clock->orbit[sat_count], &length, sbp_orbit_clock);
    rtcm_ssr_clock_to_sbp(
        &msg_orbit_clock->clock[sat_count], &length, sbp_orbit_clock);

    state->cb_rtcm_to_sbp(SBP_MSG_SSR_ORBIT_CLOCK,
                          length,
                          (u8 *)sbp_orbit_clock,
                          0,
                          state->context);
  }
}

void rtcm3_ssr_code_bias_to_sbp(rtcm_msg_code_bias *msg_code_biases,
                                struct rtcm3_sbp_state *state) {
  uint8_t buffer[SSR_MESSAGE_LENGTH];
  uint8_t length;
  msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t *)buffer;
  for (int sat_count = 0; sat_count < msg_code_biases->header.num_sats;
       sat_count++) {
    memset(buffer, 0, SSR_MESSAGE_LENGTH);
    length = 0;
    sbp_code_bias->time =
        compute_ssr_message_time(msg_code_biases->header.constellation,
                                 msg_code_biases->header.epoch_time * SECS_MS,
                                 &state->time_from_rover_obs,
                                 state);

    if (!gps_time_sec_valid(&sbp_code_bias->time)) {
      /* Invalid time */
      return;
    }
    length += sizeof(sbp_code_bias->time);

    sbp_code_bias->sid.code =
        constellation_to_l1_code(msg_code_biases->header.constellation);

    sbp_code_bias->sid.sat = msg_code_biases->sats[sat_count].sat_id;
    length += sizeof(sbp_code_bias->sid);

    sbp_code_bias->update_interval = msg_code_biases->header.update_interval;
    length += sizeof(sbp_code_bias->update_interval);

    sbp_code_bias->iod_ssr = msg_code_biases->header.iod_ssr;
    length += sizeof(sbp_code_bias->iod_ssr);

    for (int sig_count = 0;
         sig_count < msg_code_biases->sats[sat_count].num_code_biases;
         sig_count++) {
      sbp_code_bias->biases[sig_count].code =
          msg_code_biases->sats[sat_count].signals[sig_count].signal_id;
      length += sizeof(sbp_code_bias->biases[sig_count].code);
      sbp_code_bias->biases[sig_count].value =
          msg_code_biases->sats[sat_count].signals[sig_count].code_bias;
      length += sizeof(sbp_code_bias->biases[sig_count].value);
    }
    state->cb_rtcm_to_sbp(SBP_MSG_SSR_CODE_BIASES,
                          length,
                          (u8 *)sbp_code_bias,
                          0,
                          state->context);
  }
}

void rtcm3_ssr_phase_bias_to_sbp(rtcm_msg_phase_bias *msg_phase_biases,
                                 struct rtcm3_sbp_state *state) {
  uint8_t buffer[SSR_MESSAGE_LENGTH];
  uint8_t length;
  msg_ssr_phase_biases_t *sbp_phase_bias = (msg_ssr_phase_biases_t *)buffer;
  for (int sat_count = 0; sat_count < msg_phase_biases->header.num_sats;
       sat_count++) {
    memset(buffer, 0, SSR_MESSAGE_LENGTH);
    length = 0;
    sbp_phase_bias->time =
        compute_ssr_message_time(msg_phase_biases->header.constellation,
                                 msg_phase_biases->header.epoch_time * SECS_MS,
                                 &state->time_from_rover_obs,
                                 state);

    if (!gps_time_sec_valid(&sbp_phase_bias->time)) {
      /* Invalid time */
      return;
    }
    length += sizeof(sbp_phase_bias->time);

    sbp_phase_bias->sid.code =
        constellation_to_l1_code(msg_phase_biases->header.constellation);

    sbp_phase_bias->sid.sat = msg_phase_biases->sats[sat_count].sat_id;
    length += sizeof(sbp_phase_bias->sid);

    sbp_phase_bias->update_interval = msg_phase_biases->header.update_interval;
    length += sizeof(sbp_phase_bias->update_interval);

    sbp_phase_bias->iod_ssr = msg_phase_biases->header.iod_ssr;
    length += sizeof(sbp_phase_bias->iod_ssr);

    sbp_phase_bias->dispersive_bias =
        msg_phase_biases->header.dispersive_bias_consistency;
    length += sizeof(sbp_phase_bias->dispersive_bias);

    sbp_phase_bias->mw_consistency =
        msg_phase_biases->header.melbourne_wubbena_consistency;
    length += sizeof(sbp_phase_bias->mw_consistency);

    sbp_phase_bias->yaw = msg_phase_biases->sats[sat_count].yaw_angle;
    length += sizeof(sbp_phase_bias->yaw);

    sbp_phase_bias->yaw_rate = msg_phase_biases->sats[sat_count].yaw_rate;
    length += sizeof(sbp_phase_bias->yaw_rate);

    for (int sig_count = 0;
         sig_count < msg_phase_biases->sats[sat_count].num_phase_biases;
         sig_count++) {
      sbp_phase_bias->biases[sig_count].code =
          msg_phase_biases->sats[sat_count].signals[sig_count].signal_id;
      length += sizeof(sbp_phase_bias->biases[sig_count].code);

      sbp_phase_bias->biases[sig_count].integer_indicator =
          msg_phase_biases->sats[sat_count]
              .signals[sig_count]
              .integer_indicator;
      length += sizeof(sbp_phase_bias->biases[sig_count].integer_indicator);

      sbp_phase_bias->biases[sig_count].widelane_integer_indicator =
          msg_phase_biases->sats[sat_count]
              .signals[sig_count]
              .widelane_indicator;
      length +=
          sizeof(sbp_phase_bias->biases[sig_count].widelane_integer_indicator);

      sbp_phase_bias->biases[sig_count].discontinuity_counter =
          msg_phase_biases->sats[sat_count]
              .signals[sig_count]
              .discontinuity_indicator;
      length += sizeof(sbp_phase_bias->biases[sig_count].discontinuity_counter);

      sbp_phase_bias->biases[sig_count].bias =
          msg_phase_biases->sats[sat_count].signals[sig_count].phase_bias;
      length += sizeof(sbp_phase_bias->biases[sig_count].bias);
    }
    state->cb_rtcm_to_sbp(SBP_MSG_SSR_PHASE_BIASES,
                          length,
                          (u8 *)sbp_phase_bias,
                          0,
                          state->context);
  }
}
