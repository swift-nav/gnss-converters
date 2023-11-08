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

#include <gnss-converters/nmea.h>
#include <gnss-converters/sbp_nmea.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/pvt_result.h>
#include <swiftnav/signal.h>

#include "sbp_nmea_internal.h"

/* Based on testing calculated Course Over Ground starts deviating noticeably
 * below this limit. */
#define NMEA_COG_STATIC_LIMIT_MPS 0.1f

struct nmea_meta_entry {
  uint16_t gnss_tow_mask;
  bool available_in_fused;
  void (*send)(sbp2nmea_t *);
} nmea_meta[SBP2NMEA_NMEA_CNT] = {
    [SBP2NMEA_NMEA_GGA] = {.gnss_tow_mask =
                               (1 << SBP2NMEA_SBP_POS_LLH_COV_GNSS) |
                               (1 << SBP2NMEA_SBP_GPS_TIME_GNSS) |
                               (1 << SBP2NMEA_SBP_VEL_NED_GNSS) |
                               (1 << SBP2NMEA_SBP_DOPS) |
                               (1 << SBP2NMEA_SBP_AGE_CORR),
                           .available_in_fused = true,
                           .send = send_gpgga},
    [SBP2NMEA_NMEA_RMC] = {.gnss_tow_mask =
                               (1 << SBP2NMEA_SBP_POS_LLH_COV_GNSS) |
                               (1 << SBP2NMEA_SBP_GPS_TIME_GNSS) |
                               (1 << SBP2NMEA_SBP_VEL_NED_GNSS) |
                               (1 << SBP2NMEA_SBP_DOPS) |
                               (1 << SBP2NMEA_SBP_AGE_CORR),
                           .available_in_fused = true,
                           .send = send_gprmc},
    [SBP2NMEA_NMEA_VTG] = {.gnss_tow_mask =
                               (1 << SBP2NMEA_SBP_POS_LLH_COV_GNSS) |
                               (1 << SBP2NMEA_SBP_GPS_TIME_GNSS) |
                               (1 << SBP2NMEA_SBP_VEL_NED_GNSS) |
                               (1 << SBP2NMEA_SBP_DOPS) |
                               (1 << SBP2NMEA_SBP_AGE_CORR),
                           .available_in_fused = true,
                           .send = send_gpvtg},
    [SBP2NMEA_NMEA_HDT] = {.gnss_tow_mask = (1 << SBP2NMEA_SBP_HDG),
                           .available_in_fused = false,
                           .send = send_gphdt},
    [SBP2NMEA_NMEA_GLL] = {.gnss_tow_mask =
                               (1 << SBP2NMEA_SBP_POS_LLH_COV_GNSS) |
                               (1 << SBP2NMEA_SBP_GPS_TIME_GNSS) |
                               (1 << SBP2NMEA_SBP_VEL_NED_GNSS) |
                               (1 << SBP2NMEA_SBP_DOPS) |
                               (1 << SBP2NMEA_SBP_AGE_CORR),
                           .available_in_fused = true,
                           .send = send_gpgll},
    [SBP2NMEA_NMEA_ZDA] = {.gnss_tow_mask = 0,
                           .available_in_fused = true,
                           .send = send_gpzda},
    [SBP2NMEA_NMEA_GSA] = {.gnss_tow_mask =
                               (1 << SBP2NMEA_SBP_POS_LLH_COV_GNSS) |
                               (1 << SBP2NMEA_SBP_GPS_TIME_GNSS) |
                               (1 << SBP2NMEA_SBP_VEL_NED_GNSS) |
                               (1 << SBP2NMEA_SBP_DOPS) |
                               (1 << SBP2NMEA_SBP_AGE_CORR),
                           .available_in_fused = false,
                           .send = send_gsa},
    [SBP2NMEA_NMEA_GST] = {.gnss_tow_mask =
                               (1 << SBP2NMEA_SBP_POS_LLH_COV_GNSS) |
                               (1 << SBP2NMEA_SBP_GPS_TIME_GNSS) |
                               (1 << SBP2NMEA_SBP_VEL_NED_GNSS) |
                               (1 << SBP2NMEA_SBP_DOPS) |
                               (1 << SBP2NMEA_SBP_AGE_CORR),
                           .available_in_fused = true,
                           .send = send_gpgst},
    [SBP2NMEA_NMEA_GSV] = {.gnss_tow_mask = 0,
                           .available_in_fused = false,
                           .send = send_gsv},
    [SBP2NMEA_NMEA_PUBX] = {.gnss_tow_mask =
                                (1 << SBP2NMEA_SBP_POS_LLH_COV_GNSS) |
                                (1 << SBP2NMEA_SBP_GPS_TIME_GNSS) |
                                (1 << SBP2NMEA_SBP_VEL_NED_GNSS) |
                                (1 << SBP2NMEA_SBP_DOPS) |
                                (1 << SBP2NMEA_SBP_AGE_CORR),
                            .available_in_fused = true,
                            .send = send_pubx},
    [SBP2NMEA_NMEA_THS] = {.gnss_tow_mask =
                               (1 << SBP2NMEA_SBP_ORIENT_EULER) |
                               (1 << SBP2NMEA_SBP_POS_LLH_COV_GNSS),
                           .available_in_fused = true,
                           .send = send_gpths},
};

static sbp2nmea_sbp_id_t translate_sbpid(const sbp2nmea_t *state,
                                         sbp2nmea_sbp_id_t id,
                                         bool consider_mode) {
  static const sbp2nmea_sbp_id_t id_map[4][2] = {
      {SBP2NMEA_SBP_GPS_TIME_GNSS, SBP2NMEA_SBP_GPS_TIME},
      {SBP2NMEA_SBP_UTC_TIME_GNSS, SBP2NMEA_SBP_UTC_TIME},
      {SBP2NMEA_SBP_POS_LLH_COV_GNSS, SBP2NMEA_SBP_POS_LLH_COV},
      {SBP2NMEA_SBP_VEL_NED_GNSS, SBP2NMEA_SBP_VEL_NED}};

  if (consider_mode) {
    for (size_t i = 0; i < ARRAY_SIZE(id_map); i++) {
      if (id == id_map[i][state->actual_mode == SBP2NMEA_MODE_GNSS]) {
        return id_map[i][state->actual_mode == SBP2NMEA_MODE_FUSED];
      }
    }
  }
  return id;
}

static uint32_t get_tow(const sbp2nmea_t *state,
                        sbp2nmea_sbp_id_t id,
                        bool consider_mode) {
  const sbp_msg_t *msg = sbp2nmea_msg_get(state, id, consider_mode);
  switch (id) {
    case SBP2NMEA_SBP_GPS_TIME:
      return msg->gps_time.tow;
    case SBP2NMEA_SBP_UTC_TIME:
      return msg->utc_time.tow;
    case SBP2NMEA_SBP_POS_LLH_COV:
      return msg->pos_llh_cov.tow;
    case SBP2NMEA_SBP_VEL_NED:
      return msg->vel_ned.tow;
    case SBP2NMEA_SBP_DOPS:
      return msg->dops.tow;
    case SBP2NMEA_SBP_AGE_CORR:
      return msg->age_corrections.tow;
    case SBP2NMEA_SBP_HDG:
      return msg->baseline_heading.tow;
    case SBP2NMEA_SBP_SOLN_META:
      return msg->soln_meta.tow;
    case SBP2NMEA_SBP_GPS_TIME_GNSS:
      return msg->gps_time_gnss.tow;
    case SBP2NMEA_SBP_UTC_TIME_GNSS:
      return msg->utc_time_gnss.tow;
    case SBP2NMEA_SBP_POS_LLH_COV_GNSS:
      return msg->pos_llh_cov_gnss.tow;
    case SBP2NMEA_SBP_VEL_NED_GNSS:
      return msg->vel_ned_gnss.tow;
    case SBP2NMEA_SBP_ORIENT_EULER:
      return msg->orient_euler.tow;
    case SBP2NMEA_SBP_SV_AZ_EL:
    case SBP2NMEA_SBP_MEASUREMENT_STATE:
    case SBP2NMEA_SBP_GROUP_META:
    case SBP2NMEA_SBP_CNT:
    default:
      return 0;
  }
}

static double sbp_gpsdifftime(const sbp_v4_gps_time_t *sbp_end,
                              const sbp_v4_gps_time_t *sbp_begin) {
  gps_time_t end;
  gps_time_t begin;
  end.wn = sbp_end->wn;
  end.tow = (double)sbp_end->tow / SECS_MS;
  begin.wn = sbp_begin->wn;
  begin.tow = (double)sbp_begin->tow / SECS_MS;
  return gpsdifftime(&end, &begin);
}

static bool nmea_ready(const sbp2nmea_t *state, sbp2nmea_nmea_id_t nmea_id) {
  if (state->requested_mode == SBP2NMEA_MODE_FUSED &&
      !nmea_meta[nmea_id].available_in_fused) {
    return false;
  }

  if (state->actual_mode == SBP2NMEA_MODE_FUSED &&
      nmea_meta[nmea_id].available_in_fused) {
    if (!state->fused_wagon_complete) {
      return false;
    }

    return state->nmea_state[nmea_id].last_tow !=
           get_tow(state, SBP2NMEA_SBP_UTC_TIME, false);
  }

  if (state->nmea_state[nmea_id].last_tow ==
      get_tow(state, SBP2NMEA_SBP_UTC_TIME_GNSS, false)) {
    /* this message was already sent on this epoch */
    return false;
  }

  if (SBP2NMEA_NMEA_GSA == nmea_id) {
    const sbp_msg_gps_time_t *msg =
        &sbp2nmea_msg_get(state, SBP2NMEA_SBP_GPS_TIME_GNSS, false)->gps_time;
    if (NULL == msg) {
      return false;
    }

    if (state->obs_seq_count != state->obs_seq_total - 1) {
      /* observation sequence not complete */
      return false;
    }

    const sbp_v4_gps_time_t gps_time = {.wn = msg->wn, .tow = msg->tow};
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
    if (nmea_meta[nmea_id].gnss_tow_mask & (1 << sbp_id)) {
      if (get_tow(state, sbp_id, false) !=
          get_tow(state, SBP2NMEA_SBP_UTC_TIME_GNSS, false)) {
        return false;
      }
    }
  }

  return true;
}

/* Send all the NMEA messages that have become ready to send */
static void check_nmea_send(sbp2nmea_t *state) {
  const u32 tow_from_mode = get_tow(state, SBP2NMEA_SBP_UTC_TIME, true);
  const u32 tow_from_gnss = get_tow(state, SBP2NMEA_SBP_UTC_TIME_GNSS, false);
  const float freq = state->soln_freq;

  /* Send each NMEA message if all its component SBP messages have been received
   * and current time matches the send rate */
  for (sbp2nmea_nmea_id_t id = 0; id < SBP2NMEA_NMEA_CNT; ++id) {
    u32 tow = nmea_meta[id].available_in_fused ? tow_from_mode : tow_from_gnss;

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
                                 const sbp_msg_t *sbp_msg) {
  /* discard the odd MEASUREMENT_STATE messages that carry GLO FCNs instead of
   * satellite IDs */
  if (SBP2NMEA_SBP_MEASUREMENT_STATE == sbp_id) {
    const sbp_measurement_state_t *states = sbp_msg->measurement_state.states;
    const u8 n_state = sbp_msg->measurement_state.n_states;
    for (u8 i = 0; i < n_state; i++) {
      if (IS_GLO(states[i].mesid) && states[i].mesid.sat > NUM_SATS_GLO) {
        /* GLO mesid is of the form 100+FCN, discard this message */
        return true;
      }
    }
  }
  return false;
}

static sbp2nmea_sbp_id_t recode_legacy_sbp_id(const sbp2nmea_t *state,
                                              sbp2nmea_sbp_id_t sbp_id) {
  if (state->requested_mode == SBP2NMEA_MODE_GNSS) {
    if (sbp_id == SBP2NMEA_SBP_GPS_TIME) {
      return SBP2NMEA_SBP_GPS_TIME_GNSS;
    }
    if (sbp_id == SBP2NMEA_SBP_UTC_TIME) {
      return SBP2NMEA_SBP_UTC_TIME_GNSS;
    }
    if (sbp_id == SBP2NMEA_SBP_POS_LLH_COV) {
      return SBP2NMEA_SBP_POS_LLH_COV_GNSS;
    }
    if (sbp_id == SBP2NMEA_SBP_VEL_NED) {
      return SBP2NMEA_SBP_VEL_NED_GNSS;
    }
  }
  return sbp_id;
}

static void update_mode(sbp2nmea_t *state, sbp2nmea_sbp_id_t sbp_id) {
  if (state->requested_mode == SBP2NMEA_MODE_BEST &&
      state->actual_mode == SBP2NMEA_MODE_GNSS &&
      sbp_id == SBP2NMEA_SBP_GROUP_META) {
    state->actual_mode = SBP2NMEA_MODE_FUSED;
  }
}

static void check_fused_wagon_complete(sbp2nmea_t *state,
                                       sbp2nmea_sbp_id_t sbp_id) {
  if (state->actual_mode != SBP2NMEA_MODE_FUSED) {
    return;
  }
  if (sbp_id == SBP2NMEA_SBP_GROUP_META) {
    state->fused_wagon_complete = false;
  } else if (sbp_id == SBP2NMEA_SBP_SOLN_META) {
    state->fused_wagon_complete = true;
  }
}

void sbp2nmea(sbp2nmea_t *state,
              const sbp_msg_t *sbp_msg,
              sbp2nmea_sbp_id_t sbp_id) {
  sbp_id = recode_legacy_sbp_id(state, sbp_id);
  update_mode(state, sbp_id);
  check_fused_wagon_complete(state, sbp_id);

  if (sbp2nmea_discard_sbp(sbp_id, sbp_msg)) {
    return;
  }
  sbp2nmea_msg_set(state, sbp_msg, sbp_id);
  check_nmea_send(state);
}

void sbp2nmea_base_id_set(sbp2nmea_t *state, const uint16_t base_sender_id) {
  state->base_sender_id = base_sender_id;
}

uint16_t sbp2nmea_base_id_get(const sbp2nmea_t *state) {
  return state->base_sender_id;
}

uint8_t sbp2nmea_num_obs_get(const sbp2nmea_t *state) { return state->num_obs; }

const sbp_v4_gnss_signal_t *sbp2nmea_nav_sids_get(const sbp2nmea_t *state) {
  return state->nav_sids;
}

static void unpack_obs_header(const sbp_observation_header_t *header,
                              sbp_v4_gps_time_t *obs_time,
                              u8 *total,
                              u8 *count) {
  *obs_time = header->t;
  *total = (header->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (header->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

void sbp2nmea_obs(sbp2nmea_t *state, const sbp_msg_obs_t *sbp_obs) {
  uint8_t count;
  sbp_v4_gps_time_t obs_time;
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

  for (int i = 0; i < sbp_obs->n_obs; i++) {
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

const sbp_msg_t *sbp2nmea_msg_get(const sbp2nmea_t *state,
                                  sbp2nmea_sbp_id_t id,
                                  bool consider_mode) {
  return &state->sbp_state[translate_sbpid(state, id, consider_mode)];
}

void sbp2nmea_msg_set(sbp2nmea_t *state,
                      const sbp_msg_t *sbp_msg,
                      sbp2nmea_sbp_id_t id) {
  MEMCPY_S(&state->sbp_state[id],
           sizeof(state->sbp_state[id]),
           sbp_msg,
           sizeof(*sbp_msg));
}

void sbp2nmea_rate_set(sbp2nmea_t *state, int rate, sbp2nmea_nmea_id_t id) {
  state->nmea_state[id].rate = rate;
}

void sbp2nmea_soln_freq_set(sbp2nmea_t *state, float soln_freq) {
  state->soln_freq = soln_freq;
}

void sbp2nmea_cog_threshold_set(sbp2nmea_t *state, float cog_thd_mps) {
  state->cog_threshold_mps = cog_thd_mps;
}

void sbp2nmea_cog_stationary_threshold_set(sbp2nmea_t *state,
                                           double cog_stationary_thd_mps) {
  state->cog_update_threshold_mps = cog_stationary_thd_mps;
}

void sbp2nmea_init(sbp2nmea_t *state,
                   sbp2nmea_mode_t mode,
                   void (*cb_sbp_to_nmea)(char *msg, void *ctx),
                   void *ctx) {
  memset(state, 0, sizeof(*state));
  state->cog_threshold_mps = NMEA_COG_STATIC_LIMIT_MPS;
  state->cb_sbp_to_nmea = cb_sbp_to_nmea;
  state->ctx = ctx;
  state->requested_mode = mode;
  state->actual_mode = SBP2NMEA_MODE_GNSS;
  state->last_non_stationary_cog = 0;
  state->cog_update_threshold_mps = NMEA_COG_STATIC_LIMIT_MPS;
}
