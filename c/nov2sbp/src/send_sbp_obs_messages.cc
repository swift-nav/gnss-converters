/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/**
 * This implementation is entirely lifted from PFWP.
 */
#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/signal.h>

#include <cassert>

#include "swiftnav_conversion_helpers.h"

namespace Novatel {

extern "C" {

#include <libsbp/observation.h>
#include <libsbp/sbp.h>

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)
#define MSG_OBS_HEADER_MAX_SIZE MSG_OBS_HEADER_SEQ_MASK

#define MSG_OBS_P_MULTIPLIER ((double)5e1)
#define MSG_OBS_CN0_MULTIPLIER ((float)4)
#define MSG_OBS_LF_OVERFLOW (1 << 8)
#define MSG_OBS_DF_OVERFLOW (1 << 8)
#define MSG_OBS_LF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_DF_MULTIPLIER ((double)(1 << 8))
#define MSG_OBS_FLAGS_CODE_VALID ((u8)(1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID ((u8)(1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN ((u8)(1 << 2))
#define MSG_OBS_FLAGS_MEAS_DOPPLER_VALID ((u8)(1 << 3))
#define MSG_OBS_FLAGS_RAIM_EXCLUSION ((u8)(1 << 7))

/** \defgroup sbp_utils SBP Utils
 * Convert to and from SBP message types and other useful functions.
 * \{ */

static void round_time_nano(const gps_time_t *t_in, sbp_gps_time_t *t_out) {
  t_out->wn = static_cast<uint16_t>(t_in->wn);
  t_out->tow = static_cast<uint32_t>(round(t_in->tow * 1e3));
  t_out->ns_residual =
      static_cast<int32_t>(round((t_in->tow - t_out->tow / 1e3) * 1e9));
  /* week roll-over */
  if (t_out->tow >= WEEK_MS) {
    t_out->wn++;
    t_out->tow -= WEEK_MS;
  }
}

static sbp_gnss_signal_t sid_to_sbp(const gnss_signal_t from) {
  sbp_gnss_signal_t sbp_sid;
  sbp_sid.sat = static_cast<uint8_t>(from.sat);
  sbp_sid.code = static_cast<uint8_t>(from.code);
  return sbp_sid;
}

static void pack_obs_header(const gps_time_t *t,
                            u8 total,
                            u8 count,
                            observation_header_t *msg) {
  round_time_nano(t, &msg->t);
  msg->n_obs = static_cast<uint8_t>(((total << MSG_OBS_HEADER_SEQ_SHIFT) |
                                     (count & MSG_OBS_HEADER_SEQ_MASK)));
}

static u8 nm_flags_to_sbp(nav_meas_flags_t from) {
  u8 to = 0;
  if (0 != (from & NAV_MEAS_FLAG_CODE_VALID)) {
    to |= MSG_OBS_FLAGS_CODE_VALID;
  }
  if (0 != (from & NAV_MEAS_FLAG_PHASE_VALID)) {
    to |= MSG_OBS_FLAGS_PHASE_VALID;
  }
  if (0 != (from & NAV_MEAS_FLAG_HALF_CYCLE_KNOWN)) {
    to |= MSG_OBS_FLAGS_HALF_CYCLE_KNOWN;
  }
  if (0 != (from & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
    to |= MSG_OBS_FLAGS_MEAS_DOPPLER_VALID;
  }
  if (0 != (from & NAV_MEAS_FLAG_RAIM_EXCLUSION)) {
    to |= MSG_OBS_FLAGS_RAIM_EXCLUSION;
  }
  return to;
}

/** Pack GPS observables into a `msg_obs_content_t` struct.
 * For use in constructing a `MSG_NEW_OBS` SBP message.
 *
 * \param P Pseudorange in meters
 * \param L Carrier phase in cycles
 * \param D Measured doppler in Hz
 * \param cn0 Signal-to-noise ratio
 * \param lock_time Lock time gives an indication of the time
                    for which a signal has maintained continuous phase lock in
                    second
 * \param flags Observation flags from nav_meas_flags_t
 * \param sid Signal ID
 * \param msg Pointer to a `msg_obs_content_t` struct to fill out
 * \return `0` on success or `-1` on an overflow error
 */
static s8 pack_obs_content(double P,
                           double L,
                           double D,
                           double cn0,
                           double lock_time,
                           nav_meas_flags_t flags,
                           gnss_signal_t sid,
                           packed_obs_content_t *msg) {
  // If the sid is not valid, it means it is a signal that Novatel tracks
  // which we do not want to support at this time.
  if (!sid_valid(sid)) {
    return -1;
  }

  s64 P_fp = llround(P * MSG_OBS_P_MULTIPLIER);
  if (P < 0 || P_fp > UINT32_MAX) {
    return -1;
  }

  msg->P = static_cast<uint32_t>(P_fp);

  double Li = floor(-L);
  if (Li < INT32_MIN || Li > INT32_MAX) {
    return -1;
  }

  double Lf = -L - Li;

  msg->L.i = static_cast<int32_t>(Li);
  u16 frac_part_cp = static_cast<uint16_t>(round(Lf * MSG_OBS_LF_MULTIPLIER));
  if (frac_part_cp >= MSG_OBS_LF_OVERFLOW) {
    frac_part_cp = 0;
    msg->L.i += 1;
  }
  msg->L.f = static_cast<uint8_t>(frac_part_cp);

  double Di = floor(D);
  if (Di < INT16_MIN || Di > INT16_MAX) {
    return -1;
  }

  double Df = D - Di;

  msg->D.i = static_cast<int16_t>(Di);
  u16 frac_part_d = static_cast<uint16_t>(round(Df * MSG_OBS_DF_MULTIPLIER));
  if (frac_part_d >= MSG_OBS_DF_OVERFLOW) {
    frac_part_d = 0;
    msg->D.i += 1;
  }
  msg->D.f = static_cast<uint8_t>(frac_part_d);

  if (0 != (flags & NAV_MEAS_FLAG_CN0_VALID)) {
    s32 cn0_fp = static_cast<int32_t>(lround(cn0 * MSG_OBS_CN0_MULTIPLIER));
    if (cn0 < 0 || cn0_fp > UINT8_MAX) {
      return -1;
    }

    msg->cn0 = static_cast<uint8_t>(cn0_fp);
  } else {
    msg->cn0 = 0;
  }

  msg->lock = encode_lock_time(lock_time);

  msg->flags = nm_flags_to_sbp(flags);

  msg->sid = sid_to_sbp(sid);

  return 0;
}

void send_sbp_obs_messages(uint8_t n,
                           const mini_navigation_measurement_t *m,
                           const gps_time_t *t,
                           void (*sbp_send_msg)(uint32_t, size_t, uint8_t *)) {
  u32 msg_obs_max_size = SBP_MAX_PAYLOAD_LEN;
  static u8 buff[256];

  if ((0 == n) || (nullptr == m) || (nullptr == t)) {
    gps_time_t t_dummy = GPS_TIME_UNKNOWN;
    pack_obs_header(&t_dummy, 1, 0, (observation_header_t *)buff);  // NOLINT
    sbp_send_msg(SBP_MSG_OBS, sizeof(observation_header_t), buff);
    return;
  }

  /* Upper limit set by SBP framing size, preventing underflow */
  u16 msg_payload_size =
      MAX(MIN((u16)MAX(msg_obs_max_size, 0), SBP_MAX_PAYLOAD_LEN),
          sizeof(observation_header_t)) -
      sizeof(observation_header_t);

  /* Lower limit set by sending at least 1 observation */
  msg_payload_size = MAX(msg_payload_size, sizeof(packed_obs_content_t));

  /* Round down the number of observations per message */
  u16 obs_in_msg = msg_payload_size / sizeof(packed_obs_content_t);

  /* Round up the number of messages */
  u16 total = MIN((n + obs_in_msg - 1) / obs_in_msg, MSG_OBS_HEADER_MAX_SIZE);

  u8 obs_i = 0;
  for (u8 count = 0; count < total; count++) {
    u8 curr_n = MIN(n - obs_i, static_cast<uint8_t>(obs_in_msg));
    pack_obs_header(t,
                    static_cast<uint8_t>(total),
                    count,
                    (observation_header_t *)buff);  // NOLINT
    packed_obs_content_t *obs =
        (packed_obs_content_t *)&buff[sizeof(observation_header_t)];  // NOLINT

    for (u8 i = 0; i < curr_n; i++, obs_i++) {
      if (pack_obs_content(m[obs_i].raw_pseudorange,
                           m[obs_i].raw_carrier_phase,
                           m[obs_i].raw_measured_doppler,
                           m[obs_i].cn0,
                           m[obs_i].lock_time,
                           m[obs_i].flags,
                           m[obs_i].sid,
                           &obs[i]) < 0) {
        /* Error packing this observation, skip it. */
        i--;
        curr_n--;
      }
    }

    // clang-format off
    sbp_send_msg(
        SBP_MSG_OBS,
        sizeof(observation_header_t) + curr_n * sizeof(packed_obs_content_t),
        buff);
    // clang-format on
  }
}
}
}  // namespace Novatel
