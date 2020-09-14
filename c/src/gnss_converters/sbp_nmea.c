/*
 * Copyright (C) 2018-2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <gnss-converters/nmea.h>
#include <gnss-converters/sbp_nmea.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/pvt_result.h>
#include <swiftnav/signal.h>

#include "sbp_nmea_internal.h"

struct nmea_meta_entry {
  uint16_t base_tow_mask;
  uint16_t non_dr_tow_mask;
  void (*send)(const sbp2nmea_t *);
} nmea_meta[SBP2NMEA_NMEA_CNT] = {
    [SBP2NMEA_NMEA_GGA] = {.base_tow_mask = (1 << SBP2NMEA_SBP_POS_LLH_COV) |
                                            (1 << SBP2NMEA_SBP_GPS_TIME) |
                                            (1 << SBP2NMEA_SBP_VEL_NED),
                           .non_dr_tow_mask = (1 << SBP2NMEA_SBP_DOPS) |
                                              (1 << SBP2NMEA_SBP_AGE_CORR),
                           .send = send_gpgga},
    [SBP2NMEA_NMEA_RMC] = {.base_tow_mask = (1 << SBP2NMEA_SBP_POS_LLH_COV) |
                                            (1 << SBP2NMEA_SBP_GPS_TIME) |
                                            (1 << SBP2NMEA_SBP_VEL_NED),
                           .non_dr_tow_mask = (1 << SBP2NMEA_SBP_DOPS) |
                                              (1 << SBP2NMEA_SBP_AGE_CORR),
                           .send = send_gprmc},
    [SBP2NMEA_NMEA_VTG] = {.base_tow_mask = (1 << SBP2NMEA_SBP_POS_LLH_COV) |
                                            (1 << SBP2NMEA_SBP_GPS_TIME) |
                                            (1 << SBP2NMEA_SBP_VEL_NED),
                           .non_dr_tow_mask = (1 << SBP2NMEA_SBP_DOPS) |
                                              (1 << SBP2NMEA_SBP_AGE_CORR),
                           .send = send_gpvtg},
    [SBP2NMEA_NMEA_HDT] = {.base_tow_mask = (1 << SBP2NMEA_SBP_HDG),
                           .non_dr_tow_mask = 0,
                           .send = send_gphdt},
    [SBP2NMEA_NMEA_GLL] = {.base_tow_mask = (1 << SBP2NMEA_SBP_POS_LLH_COV) |
                                            (1 << SBP2NMEA_SBP_GPS_TIME) |
                                            (1 << SBP2NMEA_SBP_VEL_NED),
                           .non_dr_tow_mask = (1 << SBP2NMEA_SBP_DOPS) |
                                              (1 << SBP2NMEA_SBP_AGE_CORR),
                           .send = send_gpgll},
    [SBP2NMEA_NMEA_ZDA] = {.base_tow_mask = 0,
                           .non_dr_tow_mask = 0,
                           .send = send_gpzda},
    [SBP2NMEA_NMEA_GSA] = {.base_tow_mask = (1 << SBP2NMEA_SBP_POS_LLH_COV) |
                                            (1 << SBP2NMEA_SBP_GPS_TIME) |
                                            (1 << SBP2NMEA_SBP_VEL_NED) |
                                            (1 << SBP2NMEA_SBP_DOPS) |
                                            (1 << SBP2NMEA_SBP_AGE_CORR),
                           .non_dr_tow_mask = 0,
                           .send = send_gsa},
    [SBP2NMEA_NMEA_GST] = {.base_tow_mask = (1 << SBP2NMEA_SBP_POS_LLH_COV) |
                                            (1 << SBP2NMEA_SBP_GPS_TIME) |
                                            (1 << SBP2NMEA_SBP_VEL_NED),
                           .non_dr_tow_mask = (1 << SBP2NMEA_SBP_DOPS) |
                                              (1 << SBP2NMEA_SBP_AGE_CORR),
                           .send = send_gpgst},
    [SBP2NMEA_NMEA_GSV] = {.base_tow_mask = 0,
                           .non_dr_tow_mask = 0,
                           .send = send_gsv},
    [SBP2NMEA_NMEA_PUBX] = {.base_tow_mask = (1 << SBP2NMEA_SBP_POS_LLH_COV) |
                                             (1 << SBP2NMEA_SBP_GPS_TIME) |
                                             (1 << SBP2NMEA_SBP_VEL_NED),
                            .non_dr_tow_mask = (1 << SBP2NMEA_SBP_DOPS) |
                                               (1 << SBP2NMEA_SBP_AGE_CORR),
                            .send = send_pubx},
};

struct sbp_meta_entry {
  size_t offset_tow;
} sbp_meta[SBP2NMEA_SBP_CNT] = {
    [SBP2NMEA_SBP_GPS_TIME] = {.offset_tow = offsetof(msg_gps_time_t, tow)},
    [SBP2NMEA_SBP_UTC_TIME] = {.offset_tow = offsetof(msg_utc_time_t, tow)},
    [SBP2NMEA_SBP_POS_LLH_COV] = {.offset_tow =
                                      offsetof(msg_pos_llh_cov_t, tow)},
    [SBP2NMEA_SBP_VEL_NED] = {.offset_tow = offsetof(msg_vel_ned_t, tow)},
    [SBP2NMEA_SBP_DOPS] = {.offset_tow = offsetof(msg_dops_t, tow)},
    [SBP2NMEA_SBP_AGE_CORR] = {.offset_tow =
                                   offsetof(msg_age_corrections_t, tow)},
    [SBP2NMEA_SBP_HDG] = {.offset_tow = offsetof(msg_baseline_heading_t, tow)},
    [SBP2NMEA_SBP_SV_AZ_EL] = {.offset_tow = 0},
    [SBP2NMEA_SBP_MEASUREMENT_STATE] = {.offset_tow = 0},
};

static uint32_t get_tow(const sbp2nmea_t *state, sbp2nmea_sbp_id_t id) {
  uint32_t result;
  memcpy(&result,
         sbp2nmea_msg_get(state, id) + sbp_meta[id].offset_tow,
         sizeof(uint32_t));
  return result;
}

static double sbp_gpsdifftime(const sbp_gps_time_t *sbp_end,
                              const sbp_gps_time_t *sbp_begin) {
  gps_time_t end;
  gps_time_t begin;
  end.wn = sbp_end->wn;
  end.tow = (double)sbp_end->tow / SECS_MS;
  begin.wn = sbp_begin->wn;
  begin.tow = (double)sbp_begin->tow / SECS_MS;
  return gpsdifftime(&end, &begin);
}

static bool nmea_ready(const sbp2nmea_t *state, sbp2nmea_nmea_id_t nmea_id) {
  if (state->nmea_state[nmea_id].last_tow ==
      get_tow(state, SBP2NMEA_SBP_UTC_TIME)) {
    /* this message was already sent on this epoch */
    return false;
  }

  if (SBP2NMEA_NMEA_GSA == nmea_id) {
    msg_gps_time_t *msg = sbp2nmea_msg_get(state, SBP2NMEA_SBP_GPS_TIME);
    if (NULL == msg) {
      return false;
    }

    if (state->obs_seq_count != state->obs_seq_total - 1) {
      /* observation sequence not complete */
      return false;
    }

    const sbp_gps_time_t gps_time = {.wn = msg->wn, .tow = msg->tow};
    if (sbp_gpsdifftime(&gps_time, &state->obs_time) > 0) {
      /* Observations are older than current solution epoch.
       * Note that newer observations are allowed to be able to produce GSA
       * messages also in Time Matched mode. */
      return false;
    }
  }

  /* check that the time stamps of the component messages match that of the UTC
   * time message */
  for (sbp2nmea_sbp_id_t sbp_id = 0; sbp_id < SBP2NMEA_SBP_CNT; ++sbp_id) {
    if (nmea_meta[nmea_id].base_tow_mask & (1 << sbp_id)) {
      if (get_tow(state, sbp_id) != get_tow(state, SBP2NMEA_SBP_UTC_TIME)) {
        return false;
      }
    }
  }

  if (nmea_meta[nmea_id].non_dr_tow_mask != 0) {
    msg_pos_llh_cov_t *msg = sbp2nmea_msg_get(state, SBP2NMEA_SBP_POS_LLH_COV);
    if ((msg->flags & POSITION_MODE_MASK) != POSITION_MODE_DEAD_RECKONING) {
      for (sbp2nmea_sbp_id_t sbp_id = 0; sbp_id < SBP2NMEA_SBP_CNT; ++sbp_id) {
        if (nmea_meta[nmea_id].non_dr_tow_mask & (1 << sbp_id)) {
          if (get_tow(state, sbp_id) != get_tow(state, SBP2NMEA_SBP_UTC_TIME)) {
            return false;
          }
        }
      }
    }
  }

  return true;
}

/* Send all the NMEA messages that have become ready to send */
static void check_nmea_send(sbp2nmea_t *state) {
  const u32 tow = get_tow(state, SBP2NMEA_SBP_UTC_TIME);
  const float freq = state->soln_freq;

  /* Send each NMEA message if all its component SBP messages have been received
   * and current time matches the send rate */
  for (sbp2nmea_nmea_id_t id = 0; id < SBP2NMEA_NMEA_CNT; ++id) {
    if (!nmea_ready(state, id)) {
      continue;
    }

    if (!check_nmea_rate(state->nmea_state[id].rate, tow, freq)) {
      continue;
    }

    nmea_meta[id].send(state);
    state->nmea_state[id].last_tow = tow;
  }
}

static bool sbp2nmea_discard_sbp(const sbp2nmea_sbp_id_t sbp_id,
                                 const u8 len,
                                 const void *sbp_msg) {
  /* discard the odd MEASUREMENT_STATE messages that carry GLO FCNs instead of
   * satellite IDs */
  if (SBP2NMEA_SBP_MEASUREMENT_STATE == sbp_id) {
    const measurement_state_t *states =
        ((msg_measurement_state_t *)sbp_msg)->states;
    const u8 n_state = len / sizeof(measurement_state_t);
    for (u8 i = 0; i < n_state; i++) {
      if (IS_GLO(states[i].mesid) && states[i].mesid.sat > NUM_SATS_GLO) {
        /* GLO mesid is of the form 100+FCN, discard this message */
        return true;
      }
    }
  }
  return false;
}

void sbp2nmea(sbp2nmea_t *state,
              const u8 len,
              const void *sbp_msg,
              sbp2nmea_sbp_id_t sbp_id) {
  if (sbp2nmea_discard_sbp(sbp_id, len, sbp_msg)) {
    return;
  }
  MEMCPY_S(sbp2nmea_msg_get(state, sbp_id),
           sizeof(state->sbp_state[sbp_id].msg),
           sbp_msg,
           len);
  sbp2nmea_msg_length_set(state, len, sbp_id);
  check_nmea_send(state);
}

void sbp2nmea_base_id_set(sbp2nmea_t *state, const uint16_t base_sender_id) {
  state->base_sender_id = base_sender_id;
}

uint16_t sbp2nmea_base_id_get(const sbp2nmea_t *state) {
  return state->base_sender_id;
}

uint8_t sbp2nmea_num_obs_get(const sbp2nmea_t *state) { return state->num_obs; }

const sbp_gnss_signal_t *sbp2nmea_nav_sids_get(const sbp2nmea_t *state) {
  return state->nav_sids;
}

static void unpack_obs_header(const observation_header_t *header,
                              sbp_gps_time_t *obs_time,
                              u8 *total,
                              u8 *count) {
  *obs_time = header->t;
  *total = (header->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (header->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

void sbp2nmea_obs(sbp2nmea_t *state,
                  const msg_obs_t *sbp_obs,
                  uint8_t num_obs) {
  uint8_t count;
  sbp_gps_time_t obs_time;
  unpack_obs_header(&sbp_obs->header, &obs_time, &state->obs_seq_total, &count);

  /* Count zero means it's the first message in a sequence */
  if (count == 0) {
    state->num_obs = 0;
  } else if ((fabs(sbp_gpsdifftime(&obs_time, &state->obs_time)) >
              FLOAT_EQUALITY_EPS) ||
             (state->obs_time.wn != obs_time.wn) ||
             ((state->obs_seq_count + 1) != count)) {
    /* Obs message missed, reset sequence */
    return;
  }
  state->obs_seq_count = count;
  state->obs_time = obs_time;

  for (int i = 0; i < num_obs; i++) {
    if (!(sbp_obs->obs[i].flags & OBSERVATION_VALID)) {
      state->nav_sids[state->num_obs] = sbp_obs->obs[i].sid;
      state->num_obs++;
    }
  }

  check_nmea_send(state);
}

void sbp2nmea_to_str(const sbp2nmea_t *state, char *sentence) {
  state->cb_sbp_to_nmea(sentence, state->ctx);
}

void *sbp2nmea_msg_get(const sbp2nmea_t *state, sbp2nmea_sbp_id_t id) {
  return (void *)&state->sbp_state[id].msg.data;
}

u8 sbp2nmea_msg_length(const sbp2nmea_t *state, sbp2nmea_sbp_id_t id) {
  return state->sbp_state[id].msg.length;
}

void sbp2nmea_msg_length_set(sbp2nmea_t *state, u8 len, sbp2nmea_sbp_id_t id) {
  state->sbp_state[id].msg.length = len;
}

void sbp2nmea_rate_set(sbp2nmea_t *state, int rate, sbp2nmea_nmea_id_t id) {
  state->nmea_state[id].rate = rate;
}

void sbp2nmea_soln_freq_set(sbp2nmea_t *state, float soln_freq) {
  state->soln_freq = soln_freq;
}

void sbp2nmea_init(sbp2nmea_t *state,
                   void (*cb_sbp_to_nmea)(char *msg, void *ctx),
                   void *ctx) {
  memset(state, 0, sizeof(*state));
  state->cb_sbp_to_nmea = cb_sbp_to_nmea;
  state->ctx = ctx;
}
