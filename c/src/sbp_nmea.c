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

#include <math.h>
#include <nmea.h>
#include <sbp_nmea.h>
#include <sbp_nmea_internal.h>
#include <stdio.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>

bool gpgga_ready(struct sbp_nmea_state *state) {
  if (state->sbp_gps_time.tow == state->sbp_pos_llh.tow &&
      state->sbp_gps_time.tow == state->sbp_utc_time.tow &&
      state->sbp_gps_time.tow == state->sbp_dops.tow &&
      state->sbp_gps_time.tow == state->sbp_age_corr.tow &&
      state->sbp_gps_time.tow != state->gpgga_last_tow) {
    state->gpgga_last_tow = state->sbp_gps_time.tow;
    return true;
  }
  return false;
}

bool gsa_ready(struct sbp_nmea_state *state) {
  if ((state->count == state->total - 1) &&
      state->obs_time.tow != state->gsa_last_tow) {
    state->gsa_last_tow = state->obs_time.tow;
    state->total = 0;
    return true;
  }
  return false;
}

bool gprmc_ready(struct sbp_nmea_state *state) {
  if (state->sbp_gps_time.tow == state->sbp_pos_llh.tow &&
      state->sbp_gps_time.tow == state->sbp_utc_time.tow &&
      state->sbp_gps_time.tow == state->sbp_vel_ned.tow &&
      state->sbp_gps_time.tow != state->gprmc_last_tow) {
    state->gprmc_last_tow = state->sbp_gps_time.tow;
    return true;
  }
  return false;
}

bool gpvtg_ready(struct sbp_nmea_state *state) {
  if (state->sbp_pos_llh.tow == state->sbp_vel_ned.tow &&
      state->sbp_pos_llh.tow != state->gpvtg_last_tow) {
    state->gpvtg_last_tow = state->sbp_pos_llh.tow;
    return true;
  }
  return false;
}

bool gpgll_ready(struct sbp_nmea_state *state) {
  if (state->sbp_gps_time.tow == state->sbp_pos_llh.tow &&
      state->sbp_gps_time.tow == state->sbp_utc_time.tow &&
      state->sbp_gps_time.tow != state->gpgll_last_tow) {
    state->gpgll_last_tow = state->sbp_gps_time.tow;
    return true;
  }
  return false;
}

bool gpzda_ready(struct sbp_nmea_state *state) {
  if (state->sbp_gps_time.tow == state->sbp_utc_time.tow &&
      state->sbp_gps_time.tow != state->gpzda_last_tow) {
    state->gpzda_last_tow = state->sbp_gps_time.tow;
    return true;
  }
  return false;
}

void check_nmea_send(struct sbp_nmea_state *state) {
  /* Check collaborative time stamps for all messages */
  if (gpgga_ready(state)) {
    if (check_nmea_rate(
            state->gpgga_rate, state->sbp_gps_time.tow, state->soln_freq)) {
      send_gpgga(state);
    }
  }
  if (gsa_ready(state)) {
    if (check_nmea_rate(
            state->gsa_rate, state->obs_time.tow, state->soln_freq)) {
      send_gsa(state);
    }
  }
  if (gprmc_ready(state)) {
    if (check_nmea_rate(
            state->gprmc_rate, state->sbp_gps_time.tow, state->soln_freq)) {
      send_gprmc(state);
    }
  }
  if (gpvtg_ready(state)) {
    if (check_nmea_rate(
            state->gpvtg_rate, state->sbp_gps_time.tow, state->soln_freq)) {
      send_gpvtg(state);
    }
  }
  if (gpgll_ready(state)) {
    if (check_nmea_rate(
            state->gpgll_rate, state->sbp_gps_time.tow, state->soln_freq)) {
      send_gpgll(state);
    }
  }
  if (gpzda_ready(state)) {
    if (check_nmea_rate(
            state->gpzda_rate, state->sbp_gps_time.tow, state->soln_freq)) {
      send_gpzda(state);
    }
  }
}

void sbp2nmea_gps_time(const msg_gps_time_t *sbp_gps_time,
                       struct sbp_nmea_state *state) {
  if ((sbp_gps_time->flags & TIME_SOURCE_MASK) == NO_TIME) {
    return;
  }
  state->sbp_gps_time = *sbp_gps_time;

  check_nmea_send(state);
}

void sbp2nmea_utc_time(const msg_utc_time_t *sbp_utc_time,
                       struct sbp_nmea_state *state) {
  if ((sbp_utc_time->flags & TIME_SOURCE_MASK) == NO_TIME) {
    return;
  }
  state->sbp_utc_time = *sbp_utc_time;

  check_nmea_send(state);
}

void sbp2nmea_pos_llh(const msg_pos_llh_t *sbp_pos_llh,
                      struct sbp_nmea_state *state) {
  if ((sbp_pos_llh->flags & TIME_SOURCE_MASK) == NO_TIME) {
    return;
  }
  state->sbp_pos_llh = *sbp_pos_llh;

  check_nmea_send(state);
}

void sbp2nmea_vel_ned(const msg_vel_ned_t *sbp_vel_ned,
                      struct sbp_nmea_state *state) {
  if ((sbp_vel_ned->flags & TIME_SOURCE_MASK) == NO_TIME) {
    return;
  }
  state->sbp_vel_ned = *sbp_vel_ned;

  check_nmea_send(state);
}

void sbp2nmea_dops(const msg_dops_t *sbp_dops, struct sbp_nmea_state *state) {
  if ((sbp_dops->flags & TIME_SOURCE_MASK) == NO_TIME) {
    return;
  }
  state->sbp_dops = *sbp_dops;

  check_nmea_send(state);
}

void sbp2nmea_age_corrections(const msg_age_corrections_t *sbp_age_corr,
                              struct sbp_nmea_state *state) {
  state->sbp_age_corr = *sbp_age_corr;
  check_nmea_send(state);
}

void sbp2nmea_baseline_heading(const msg_baseline_heading_t *sbp_heading,
                               struct sbp_nmea_state *state) {
  if ((sbp_heading->flags & TIME_SOURCE_MASK) == NO_TIME) {
    return;
  }
  state->sbp_heading = *sbp_heading;

  check_nmea_send(state);
}

void sbp2nmea_set_base_id(const uint16_t base_sender_id,
                          struct sbp_nmea_state *state) {
  state->base_sender_id = base_sender_id;
}

void sbp_normalize_gps_time(sbp_gps_time_t *sbp_obs_time) {
  gps_time_t obs_time;
  obs_time.wn = sbp_obs_time->wn;
  obs_time.tow = sbp_obs_time->tow;
  normalize_gps_time(&obs_time);
  sbp_obs_time->wn = obs_time.wn;
  sbp_obs_time->tow = obs_time.tow;
}

void unpack_obs_header(const observation_header_t *header,
                       sbp_gps_time_t *obs_time,
                       u8 *total,
                       u8 *count) {
  obs_time->wn = header->t.wn;
  obs_time->tow = round(((double)header->t.tow) / 1e3 +
                        ((double)header->t.ns_residual) / 1e9);
  sbp_normalize_gps_time(obs_time);
  *total = (header->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (header->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

double sbp_gpsdifftime(sbp_gps_time_t *sbp_end, sbp_gps_time_t *sbp_begin) {
  gps_time_t end;
  gps_time_t begin;
  end.wn = sbp_end->wn;
  end.tow = sbp_end->tow;
  begin.wn = sbp_begin->wn;
  begin.tow = sbp_begin->tow;
  return gpsdifftime(&end, &begin);
}

void sbp2nmea_obs(const msg_obs_t *sbp_obs,
                  uint8_t num_obs,
                  struct sbp_nmea_state *state) {
  (void)sbp_obs;
  (void)state;

  uint8_t count;
  sbp_gps_time_t obs_time;
  unpack_obs_header(&sbp_obs->header, &obs_time, &state->total, &count);

  /* Count zero means it's the first message in a sequence */
  if (count == 0) {
    state->num_obs = 0;
  } else if ((fabs(sbp_gpsdifftime(&obs_time, &state->obs_time)) >
              FLOAT_EQUALITY_EPS) ||
             (state->obs_time.wn != obs_time.wn) ||
             ((state->count + 1) != count)) {
    /* Obs message missed, reset sequence */
    return;
  }
  state->count = count;
  state->obs_time = obs_time;

  for (int i = 0; i < num_obs; i++) {
    if (!(sbp_obs->obs[i].flags & OBSERVATION_VALID)) {
      state->nav_sids[state->num_obs] = sbp_obs->obs[i].sid;
      state->num_obs++;
    }
  }

  check_nmea_send(state);
}

void sbp2nmea_set_gpgga_rate(const int gpgga_rate, struct sbp_nmea_state *state) {
  state->gpgga_rate = gpgga_rate;
}

void sbp2nmea_set_gprmc_rate(const int gprmc_rate, struct sbp_nmea_state *state) {
  state->gprmc_rate = gprmc_rate;
}

void sbp2nmea_set_gpvtg_rate(const int gpvtg_rate, struct sbp_nmea_state *state) {
  state->gpvtg_rate = gpvtg_rate;
}

void sbp2nmea_set_gphdt_rate(const int gphdt_rate, struct sbp_nmea_state *state) {
  state->gphdt_rate = gphdt_rate;
}

void sbp2nmea_set_gpgll_rate(const int gpgll_rate, struct sbp_nmea_state *state) {
  state->gpgll_rate = gpgll_rate;
}

void sbp2nmea_set_gpzda_rate(const int gpzda_rate, struct sbp_nmea_state *state) {
  state->gpzda_rate = gpzda_rate;
}

void sbp2nmea_set_gsa_rate(const int gsa_rate, struct sbp_nmea_state *state) {
  state->gsa_rate = gsa_rate;
}

void sbp2nmea_set_soln_freq(const int soln_freq, struct sbp_nmea_state *state) {
  state->soln_freq = soln_freq;
}

void sbp2nmea_init(struct sbp_nmea_state *state,
                   void (*cb_sbp_to_nmea)(u8 msg_id[])) {
  state->base_sender_id = 0;

  state->gpgga_last_tow = TOW_INVALID;
  state->gpgsv_last_tow = TOW_INVALID;
  state->gprmc_last_tow = TOW_INVALID;
  state->gpvtg_last_tow = TOW_INVALID;
  state->gphdt_last_tow = TOW_INVALID;
  state->gpgll_last_tow = TOW_INVALID;
  state->gpzda_last_tow = TOW_INVALID;
  state->gsa_last_tow = TOW_INVALID;

  state->num_obs = 0;
  state->count = 0;
  state->total = 0;
  state->obs_time.wn = 0;
  state->obs_time.tow = 0;

  state->gpgga_rate = 0;
  state->gpgsv_rate = 0;
  state->gprmc_rate = 0;
  state->gpvtg_rate = 0;
  state->gphdt_rate = 0;
  state->gpgll_rate = 0;
  state->gpzda_rate = 0;
  state->gsa_rate = 0;

  state->soln_freq = 0;

  state->cb_sbp_to_nmea = cb_sbp_to_nmea;
}
