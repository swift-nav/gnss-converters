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

/* Per suggestion from https://stackoverflow.com/a/59780476/749342
 * to fix a compatibility issue with %zd format specifiers.
 */
#if defined(__GNUC__) && defined(__MINGW32__)
#define __USE_MINGW_ANSI_STDIO 1
#endif

/**
 * Per suggestion from
 * https://stackoverflow.com/questions/44382862/how-to-printf-a-size-t-without-warning-in-mingw-w64-gcc-7-1
 * to fix a compatibility issue with %zd format specifiers.
 */
#ifdef _WIN32
#ifdef _WIN64
#define PRI_SSIZE "d"
#define FORMAT_SSIZE(x) ((int)x)
#else
#define PRI_SSIZE "d"
#define FORMAT_SSIZE(x) ((int)x)
#endif
#else
#define PRI_SSIZE "zd"
#define FORMAT_SSIZE(x) (x)
#endif

#include <assert.h>
#include <gnss-converters/options.h>
#include <gnss-converters/rtcm3_sbp.h>
#include <libsbp/v4/gnss.h>
#include <libsbp/v4/logging.h>
#include <libsbp/v4/sbp_msg.h>
#include <math.h>
#include <rtcm3/bits.h>
#include <rtcm3/decode.h>
#include <rtcm3/encode.h>
#include <rtcm3/eph_decode.h>
#include <rtcm3/logging.h>
#include <rtcm3/ssr_decode.h>
#include <rtcm3/sta_decode.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/edc.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/fifo_byte.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/sid_set.h>
#include <swiftnav/signal.h>
#include <unistd.h>

#include "rtcm3_sbp_internal.h"
#include "rtcm3_utils.h"

#define SBP_GLO_FCN_OFFSET 8
#define SBP_GLO_FCN_UNKNOWN 0

#define RTCM3_PREAMBLE 0xD3

static void validate_base_obs_sanity(struct rtcm3_sbp_state *state,
                                     const gps_time_t *obs_time,
                                     const gps_time_t *rover_time);
static uint16_t extract_msg_len(const uint8_t *buf);
static bool verify_crc(uint8_t *buf, uint16_t buf_len);

void rtcm2sbp_init(struct rtcm3_sbp_state *state,
                   time_truth_t *time_truth,
                   void (*cb_rtcm_to_sbp)(uint16_t sender_id,
                                          sbp_msg_type_t msg_type,
                                          const sbp_msg_t *msg,
                                          void *context),
                   void (*cb_base_obs_invalid)(double timediff, void *context),
                   void *context) {
  assert(IS_POWER_OF_TWO(RTCM3_FIFO_SIZE));
  assert(RTCM3_FIFO_SIZE > (RTCM3_MSG_OVERHEAD + RTCM3_MAX_MSG_LEN));

  state->time_truth = time_truth;

  state->use_time_from_input_obs = false;
  state->time_from_input_data.wn = WN_UNKNOWN;
  state->time_from_input_data.tow = 0;

  state->leap_seconds = 0;
  state->leap_second_known = false;

  state->sender_id = 0;
  state->cb_rtcm_to_sbp = cb_rtcm_to_sbp;
  state->cb_base_obs_invalid = cb_base_obs_invalid;
  state->context = context;

  state->last_gps_time.wn = WN_UNKNOWN;
  state->last_gps_time.tow = 0;
  state->last_glo_time.wn = WN_UNKNOWN;
  state->last_glo_time.tow = 0;
  state->last_1230_received.wn = WN_UNKNOWN;
  state->last_1230_received.tow = 0;
  state->last_msm_received.wn = WN_UNKNOWN;
  state->last_msm_received.tow = 0;

  state->sent_msm_warning = false;
  for (u8 i = 0; i < UNSUPPORTED_CODE_MAX; i++) {
    state->sent_code_warning[i] = false;
  }

  for (u8 i = 0; i < sizeof(state->glo_sv_id_fcn_map); i++) {
    state->glo_sv_id_fcn_map[i] = MSM_GLO_FCN_UNKNOWN;
  }

  memset(state->obs_buffer, 0, sizeof(state->obs_buffer));
  memset(&state->obs_time, 0, sizeof(state->obs_time));
  state->obs_to_send = 0;

  rtcm_init_logging(&rtcm_log_callback_fn, state);

  sbp_state_init(&state->sbp_state);
  sbp_state_set_io_context(&state->sbp_state, context);

  fifo_init(&(state->fifo), state->fifo_buf, RTCM3_FIFO_SIZE);
}

void rtcm2sbp_decode_payload(const uint8_t *payload,
                             uint32_t payload_length,
                             struct rtcm3_sbp_state *state) {
  (void)payload_length;
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, payload, payload_length * 8);

  if (payload_length < 2) {
    return;
  }

  uint16_t byte = 0;
  uint16_t message_type =
      (payload[byte] << 4) | ((payload[byte + 1] >> 4) & 0xf);

  enum time_truth_state time_truth_state;
  if (state->use_time_from_input_obs) {
    time_truth_state = TIME_TRUTH_SOLVED;
  } else {
    time_truth_get(
        state->time_truth, &time_truth_state, &state->time_from_input_data);
  }
  switch (message_type) {
    /** msg_base_pos_ecef_t messages */
    case 1005:
    case 1006:

    /** log messages */
    case 1029:

    /** MSM warning messages */
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
    case 1123:

    /** Swift proprietary message */
    case 4062:

    /** NDF */
    case 4075:

    /** RTCM ephemerid messages for constellations GPS/BDS/GAL. GLO/QZSS is left
     * out for the moment */
    case 1019:
    case 1042:
    case 1045:
    case 1046:
      break;
    default:
      if (time_truth_state != TIME_TRUTH_SOLVED) {
        return;
      }
  }

  if (time_truth_state == TIME_TRUTH_SOLVED) {
    state->leap_seconds =
        get_gps_utc_offset(&state->time_from_input_data, NULL);
    state->leap_second_known = true;
  }

  if (verbosity_level > VERB_HIGH) {
    log_info("MID: %5u", message_type);
  }

  switch (message_type) {
    case 1001:
    case 1003:
      break;
    case 1002: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1002_bitstream(&bitstream, &new_rtcm_obs)) {
        /* Need to check if we've got obs in the buffer from the previous epoch
         and send before accepting the new message */
        add_gps_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1004: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1004_bitstream(&bitstream, &new_rtcm_obs)) {
        /* Need to check if we've got obs in the buffer from the previous epoch
         and send before accepting the new message */
        add_gps_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1005: {
      rtcm_msg_1005 msg_1005;
      if (RC_OK == rtcm3_decode_1005_bitstream(&bitstream, &msg_1005)) {
        sbp_msg_t msg;
        sbp_msg_base_pos_ecef_t *sbp_base_pos = &msg.base_pos_ecef;
        rtcm3_1005_to_sbp(&msg_1005, sbp_base_pos);
        state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(msg_1005.stn_id),
                              SbpMsgBasePosEcef,
                              &msg,
                              state->context);
      }
      break;
    }
    case 1006: {
      rtcm_msg_1006 msg_1006;
      if (RC_OK == rtcm3_decode_1006_bitstream(&bitstream, &msg_1006)) {
        sbp_msg_t msg;
        sbp_msg_base_pos_ecef_t *sbp_base_pos = &msg.base_pos_ecef;
        rtcm3_1006_to_sbp(&msg_1006, sbp_base_pos);
        state->cb_rtcm_to_sbp(
            rtcm_stn_to_sbp_sender_id(msg_1006.msg_1005.stn_id),
            SbpMsgBasePosEcef,
            &msg,
            state->context);
      }
      break;
    }
    case 1007:
    case 1008:
      break;
    case 1010: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1010_bitstream(&bitstream, &new_rtcm_obs) &&
          state->leap_second_known) {
        add_glo_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1012: {
      rtcm_obs_message new_rtcm_obs;
      if (RC_OK == rtcm3_decode_1012_bitstream(&bitstream, &new_rtcm_obs) &&
          state->leap_second_known) {
        add_glo_obs_to_buffer(&new_rtcm_obs, state);
      }
      break;
    }
    case 1019: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_gps_eph_bitstream(&bitstream, &msg_eph)) {
        sbp_msg_t msg;
        sbp_msg_ephemeris_gps_t *sbp_gps_eph = &msg.ephemeris_gps;
        if (rtcm3_gps_eph_to_sbp(&msg_eph, sbp_gps_eph, state)) {
          state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(0),
                                SbpMsgEphemerisGps,
                                &msg,
                                state->context);
        }
      }
      break;
    }
    case 1020: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_glo_eph_bitstream(&bitstream, &msg_eph)) {
        sbp_msg_t msg;
        sbp_msg_ephemeris_glo_t *sbp_glo_eph = &msg.ephemeris_glo;
        if (rtcm3_glo_eph_to_sbp(&msg_eph, sbp_glo_eph, state)) {
          rtcm2sbp_set_glo_fcn(
              sbp_glo_eph->common.sid, sbp_glo_eph->fcn, state);
          state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(0),
                                SbpMsgEphemerisGlo,
                                &msg,
                                state->context);
        }
      }
      break;
    }
    case 1042: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_bds_eph_bitstream(&bitstream, &msg_eph)) {
        sbp_msg_t msg;
        sbp_msg_ephemeris_bds_t *sbp_bds_eph = &msg.ephemeris_bds;
        rtcm3_bds_eph_to_sbp(&msg_eph, sbp_bds_eph, state);
        state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(0),
                              SbpMsgEphemerisBds,
                              &msg,
                              state->context);
      }
      break;
    }
    case 1044: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_qzss_eph_bitstream(&bitstream, &msg_eph)) {
        sbp_msg_t msg;
        sbp_msg_ephemeris_qzss_t *sbp_qzss_eph = &msg.ephemeris_qzss;
        rtcm3_qzss_eph_to_sbp(&msg_eph, sbp_qzss_eph, state);
        state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(0),
                              SbpMsgEphemerisQzss,
                              &msg,
                              state->context);
      }
      break;
    }
    case 1045: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_gal_eph_fnav_bitstream(&bitstream, &msg_eph)) {
        sbp_msg_t msg;
        sbp_msg_ephemeris_gal_t *sbp_gal_eph = &msg.ephemeris_gal;
        if (rtcm3_gal_eph_to_sbp(
                &msg_eph, EPH_SOURCE_GAL_FNAV, sbp_gal_eph, state)) {
          state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(0),
                                SbpMsgEphemerisGal,
                                &msg,
                                state->context);
        }
      }
      break;
    }

    case 1046: {
      rtcm_msg_eph msg_eph;
      if (RC_OK == rtcm3_decode_gal_eph_inav_bitstream(&bitstream, &msg_eph)) {
        sbp_msg_t msg;
        sbp_msg_ephemeris_gal_t *sbp_gal_eph = &msg.ephemeris_gal;
        if (rtcm3_gal_eph_to_sbp(
                &msg_eph, EPH_SOURCE_GAL_INAV, sbp_gal_eph, state)) {
          state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(0),
                                SbpMsgEphemerisGal,
                                &msg,
                                state->context);
        }
      }
      break;
    }
    case 1029: {
      rtcm_msg_1029 msg_1029;
      if (RC_OK == rtcm3_decode_1029_bitstream(&bitstream, &msg_1029)) {
        send_1029(&msg_1029, state);
      }
      break;
    }
    case 1033: {
      rtcm_msg_1033 msg_1033;
      if (RC_OK == rtcm3_decode_1033_bitstream(&bitstream, &msg_1033) &&
          no_1230_received(state)) {
        sbp_msg_t msg;
        sbp_msg_glo_biases_t *sbp_glo_cpb = &msg.glo_biases;
        memset(sbp_glo_cpb, 0, sizeof(*sbp_glo_cpb));

        rtcm3_1033_to_sbp(&msg_1033, sbp_glo_cpb);
        state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(msg_1033.stn_id),
                              SbpMsgGloBiases,
                              &msg,
                              state->context);
      }
      break;
    }
    case 1230: {
      rtcm_msg_1230 msg_1230;
      if (RC_OK == rtcm3_decode_1230_bitstream(&bitstream, &msg_1230)) {
        sbp_msg_t msg;
        sbp_msg_glo_biases_t *sbp_glo_cpb = &msg.glo_biases;
        memset(sbp_glo_cpb, 0, sizeof(*sbp_glo_cpb));
        rtcm3_1230_to_sbp(&msg_1230, sbp_glo_cpb);
        state->cb_rtcm_to_sbp(rtcm_stn_to_sbp_sender_id(msg_1230.stn_id),
                              SbpMsgGloBiases,
                              &msg,
                              state->context);
        state->last_1230_received = state->time_from_input_data;
      }
      break;
    }

    /* The following two chunks of messages handle converting separate SSR orbit
     * correction and SSR clock correction messages into a single SBP message
     * containing both orbit and clock corrections. This code makes the
     * following assumptions:
     *  1. Each pair of orbit and clock messages contain data for the same group
     * of satellites, this implies that we don't support the multimessage flag
     * in the RTCM message. We do match the clock correction satid with the
     * orbit correction satid, so the corrections don't have to be in the same
     * order but we will silently drop any correction that we can't find a
     * matching pair for.
     *  2. The clock and orbit messages are sent at the same frequency. The
     * epoch time of the two messages must match for them to be considered
     * pairs.
     */
    case 1057:
    case 1063:
    case 1240:
    case 1246:
    case 1258: {
      rtcm_msg_orbit msg_orbit;
      if (RC_OK == rtcm3_decode_orbit_bitstream(&bitstream, &msg_orbit)) {
        uint8_t constellation = msg_orbit.header.constellation;
        /* If we already have a matching clock message perform the conversion */
        if (state->orbit_clock_cache[constellation].contains_clock &&
            !state->orbit_clock_cache[constellation].contains_orbit &&
            state->orbit_clock_cache[constellation]
                    .data.clock.header.epoch_time ==
                msg_orbit.header.epoch_time &&
            state->orbit_clock_cache[constellation].data.clock.header.iod_ssr ==
                msg_orbit.header.iod_ssr &&
            state->orbit_clock_cache[constellation]
                    .data.clock.header.constellation ==
                msg_orbit.header.constellation) {
          rtcm3_ssr_separate_orbit_clock_to_sbp(
              &state->orbit_clock_cache[constellation].data.clock,
              &msg_orbit,
              state);
          state->orbit_clock_cache[constellation].contains_clock = false;
        } else { /* Store the decoded message in the cache */
          state->orbit_clock_cache[constellation].data.orbit = msg_orbit;
          state->orbit_clock_cache[constellation].contains_clock = false;
          state->orbit_clock_cache[constellation].contains_orbit = true;
        }
      }
      break;
    }
    case 1058:
    case 1064:
    case 1241:
    case 1247:
    case 1259: {
      rtcm_msg_clock msg_clock;
      if (RC_OK == rtcm3_decode_clock_bitstream(&bitstream, &msg_clock)) {
        uint8_t constellation = msg_clock.header.constellation;
        /* If we already have a matching orbit message perform the conversion */
        if (!state->orbit_clock_cache[constellation].contains_clock &&
            state->orbit_clock_cache[constellation].contains_orbit &&
            state->orbit_clock_cache[constellation]
                    .data.orbit.header.epoch_time ==
                msg_clock.header.epoch_time &&
            state->orbit_clock_cache[constellation].data.orbit.header.iod_ssr ==
                msg_clock.header.iod_ssr &&
            state->orbit_clock_cache[constellation]
                    .data.orbit.header.constellation ==
                msg_clock.header.constellation) {
          rtcm3_ssr_separate_orbit_clock_to_sbp(
              &msg_clock,
              &state->orbit_clock_cache[constellation].data.orbit,
              state);
          state->orbit_clock_cache[constellation].contains_orbit = false;
        } else { /* Store the decoded message in the cache */
          state->orbit_clock_cache[constellation].data.clock = msg_clock;
          state->orbit_clock_cache[constellation].contains_clock = true;
          state->orbit_clock_cache[constellation].contains_orbit = false;
        }
      }
      break;
    }
    case 1059:
    case 1065:
    case 1242:
    case 1248:
    case 1260: {
      rtcm_msg_code_bias msg_code_bias;
      if (RC_OK ==
          rtcm3_decode_code_bias_bitstream(&bitstream, &msg_code_bias)) {
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
      if (RC_OK ==
          rtcm3_decode_orbit_clock_bitstream(&bitstream, &msg_orbit_clock)) {
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
      if (RC_OK ==
          rtcm3_decode_phase_bias_bitstream(&bitstream, &msg_phase_bias)) {
        rtcm3_ssr_phase_bias_to_sbp(&msg_phase_bias, state);
      }
      break;
    }
    case 1074:
    case 1084:
    case 1094:
    case 1124: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm4_bitstream(&bitstream, &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1075:
    case 1085:
    case 1095:
    case 1125: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm5_bitstream(&bitstream, &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1076:
    case 1086:
    case 1096:
    case 1126: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm6_bitstream(&bitstream, &new_rtcm_msm)) {
        add_msm_obs_to_buffer(&new_rtcm_msm, state);
      }
      break;
    }
    case 1077:
    case 1087:
    case 1097:
    case 1127: {
      rtcm_msm_message new_rtcm_msm;
      if (RC_OK == rtcm3_decode_msm7_bitstream(&bitstream, &new_rtcm_msm)) {
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
      send_MSM_warning(&bitstream, state);
      break;
    }
    case 4062: {
      rtcm_msg_swift_proprietary swift_msg;
      if (RC_OK == rtcm3_decode_4062_bitstream(&bitstream, &swift_msg)) {
        sbp_msg_t sbp_msg;
        if (SBP_OK == sbp_message_decode(swift_msg.data,
                                         swift_msg.len,
                                         NULL,
                                         (sbp_msg_type_t)swift_msg.msg_type,
                                         &sbp_msg)) {
          state->cb_rtcm_to_sbp(swift_msg.sender_id,
                                swift_msg.msg_type,
                                &sbp_msg,
                                state->context);
        }
      }
      break;
    }
    case 4075: {
      rtcm_msg_ndf ndf_msg;
      if (RC_OK == rtcm3_decode_4075(payload, &ndf_msg)) {
        handle_ndf_frame(&ndf_msg, state);
      }
      break;
    }
    default:
      break;
  }

  /* check if the message was the final MSM message in the epoch, and if so send
   * out the SBP buffer */
  if (message_type >= MSM_MSG_TYPE_MIN && message_type <= MSM_MSG_TYPE_MAX) {
    /* The Multiple message bit DF393 is the same regardless of MSM msg type */
    if (bitstream.len < MSM_MULTIPLE_BIT_OFFSET) {
      return;
    }
    if (rtcm_getbitu(bitstream.data, MSM_MULTIPLE_BIT_OFFSET, 1) == 0) {
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

void add_glo_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  gps_time_t obs_time = GPS_TIME_UNKNOWN;
  if (!compute_glo_time(new_rtcm_obs->header.tow_ms,
                        &obs_time,
                        &state->time_from_input_data,
                        state)) {
    return;
  }

  if (!gps_time_valid(&obs_time)) {
    return;
  }

  const bool within_input_data_time =
      fabs(gpsdifftime(&obs_time, &state->time_from_input_data) -
           state->leap_seconds) <= GLO_SANITY_THRESHOLD_S;

  const bool within_last_gps_time =
      gps_time_valid(&state->last_gps_time) &&
      fabs(gpsdifftime(&obs_time, &state->last_gps_time) -
           state->leap_seconds) <= GLO_SANITY_THRESHOLD_S;

  if (!within_input_data_time && !within_last_gps_time) {
    return;
  }

  if (is_msm_active(&obs_time, state)) {
    /* Stream potentially contains also MSM observations, so discard the legacy
     * observation messages */
    return;
  }

  if (!gps_time_valid(&state->last_glo_time) ||
      gpsdifftime(&obs_time, &state->last_glo_time) > 0.5 / SECS_MS) {
    state->last_glo_time = obs_time;
    add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
  }
}

void add_gps_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  gps_time_t obs_time = GPS_TIME_UNKNOWN;
  compute_gps_time(new_rtcm_obs->header.tow_ms,
                   &obs_time,
                   &state->time_from_input_data,
                   state);

  if (!gps_time_valid(&obs_time)) {
    return;
  }

  if (is_msm_active(&obs_time, state)) {
    /* Stream potentially contains also MSM observations, so discard the legacy
     * observation messages */
    return;
  }

  if (!gps_time_valid(&state->last_gps_time) ||
      gpsdifftime(&obs_time, &state->last_gps_time) > 0.5 / SECS_MS) {
    state->last_gps_time = obs_time;
    add_obs_to_buffer(new_rtcm_obs, &obs_time, state);
  }
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
    /* DF046: 0 - C/A code, 1 - P code, 2 - Reserved, 3 - Reserved */
    if (rtcm_code == 0) {
      code = CODE_GLO_L2OF;
    } else {
      /* CODE_GLO_L2P currently not supported in sbp */
      /* warn and then replace with a supported code for now */
      send_unsupported_code_warning(UNSUPPORTED_CODE_GLO_L2P, state);
      code = CODE_GLO_L2OF;
    }
  }
  return code;
}

/* Transforms the newly received obs to sbp */
void add_obs_to_buffer(const rtcm_obs_message *new_rtcm_obs,
                       gps_time_t *obs_time,
                       struct rtcm3_sbp_state *state) {
  /* Build an SBP time stamp */
  sbp_v4_gps_time_t new_obs_time = {.wn = obs_time->wn,
                                    .tow = (u32)rint(obs_time->tow * SECS_MS),
                                    .ns_residual = 0};

  /* Check if the buffer already has obs of the same time */
  if (state->obs_to_send != 0 &&
      (state->obs_time.tow != new_obs_time.tow ||
       state->sender_id !=
           rtcm_stn_to_sbp_sender_id(new_rtcm_obs->header.stn_id))) {
    /* We either have missed a message, or we have a new station. Either way,
     send through the current buffer and clear before adding new obs */
    send_observations(state);
  }

  state->obs_time = new_obs_time;
  state->sender_id = rtcm_stn_to_sbp_sender_id(new_rtcm_obs->header.stn_id);

  /* Convert new obs directly in to state obs_buffer */
  for (u8 sat = 0; sat < new_rtcm_obs->header.n_sat; ++sat) {
    for (u8 freq = 0; freq < NUM_FREQS; ++freq) {
      const rtcm_freq_data *rtcm_freq = &new_rtcm_obs->sats[sat].obs[freq];
      if (rtcm_freq->flags.fields.valid_pr == 1 &&
          rtcm_freq->flags.fields.valid_cp == 1) {
        if (state->obs_to_send >= MAX_OBS_PER_EPOCH) {
          send_buffer_full_error(state);
          return;
        }

        sbp_packed_obs_content_t *sbp_freq =
            &state->obs_buffer[state->obs_to_send];
        sbp_freq->flags = 0;
        sbp_freq->P = 0.0;
        sbp_freq->L.i = 0;
        sbp_freq->L.f = 0.0;
        sbp_freq->D.i = 0;
        sbp_freq->D.f = 0.0;
        sbp_freq->cn0 = 0.0;
        sbp_freq->lock = 0.0;

        sbp_freq->sid.sat = new_rtcm_obs->sats[sat].svId;
        if (gps_obs_message(new_rtcm_obs->header.msg_num)) {
          if (sbp_freq->sid.sat >= 1 && sbp_freq->sid.sat <= 32) {
            /* GPS PRN, see DF009 */
            sbp_freq->sid.code =
                get_gps_sbp_code(freq, new_rtcm_obs->sats[sat].obs[freq].code);
          } else if (sbp_freq->sid.sat >= 40 && sbp_freq->sid.sat <= 58 &&
                     freq == 0) {
            /* SBAS L1 PRN */
            sbp_freq->sid.code = CODE_SBAS_L1CA;
            sbp_freq->sid.sat += 80;
          } else {
            /* invalid PRN or code */
            continue;
          }
        } else if (glo_obs_message(new_rtcm_obs->header.msg_num)) {
          if (sbp_freq->sid.sat >= 1 && sbp_freq->sid.sat <= 24) {
            /* GLO PRN, see DF038 */
            code_t glo_sbp_code = get_glo_sbp_code(
                freq, new_rtcm_obs->sats[sat].obs[freq].code, state);
            if (glo_sbp_code == CODE_INVALID) {
              continue;
            }
            sbp_freq->sid.code = glo_sbp_code;

          } else {
            /* invalid PRN or slot number uknown*/
            continue;
          }
        }

        if (rtcm_freq->flags.fields.valid_pr == 1) {
          double pseudorange = rtcm_freq->pseudorange;
          pseudorange += sbp_signal_biases[sbp_freq->sid.code];
          sbp_freq->P = (u32)rint(pseudorange * MSG_OBS_P_MULTIPLIER);
          sbp_freq->flags |= MSG_OBS_FLAGS_CODE_VALID;
        }
        if (rtcm_freq->flags.fields.valid_cp == 1) {
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

        if (rtcm_freq->flags.fields.valid_cnr == 1) {
          sbp_freq->cn0 = (u8)rint(rtcm_freq->cnr * MSG_OBS_CN0_MULTIPLIER);
        } else {
          sbp_freq->cn0 = 0;
        }

        if (rtcm_freq->flags.fields.valid_lock == 1) {
          sbp_freq->lock = rtcm3_encode_lock_time(rtcm_freq->lock);
        }

        if (verbosity_level > VERB_HIGH) {
          log_info("OBS: %3d %8s %14.3lf %14.3lf %14.3lf %2d 0x%02x",
                   sbp_freq->sid.sat,
                   code_to_string(sbp_freq->sid.code),
                   sbp_freq->P / MSG_OBS_P_MULTIPLIER,
                   sbp_freq->L.i + sbp_freq->L.f / MSG_OBS_LF_MULTIPLIER,
                   sbp_freq->cn0 / MSG_OBS_CN0_MULTIPLIER,
                   sbp_freq->lock,
                   sbp_freq->flags);
        }

        state->obs_to_send++;
      }
    }
  }

  /* If we aren't expecting another message, send the buffer */
  if (0 == new_rtcm_obs->header.sync) {
    send_observations(state);
  }
}

/**
 * Split the observation buffer into SBP messages and send them
 */
void send_observations(struct rtcm3_sbp_state *state) {
  /* We want the ceiling of n_obs divided by max obs in a single message to get
   * total number of messages needed */
  const u8 total_messages =
      1 + (state->obs_to_send > 0
               ? (state->obs_to_send - 1) / SBP_MSG_OBS_OBS_MAX
               : 0);

  assert(state->obs_to_send <= MAX_OBS_PER_EPOCH);
  assert(total_messages <= SBP_MAX_OBS_SEQ);

  /* Write the SBP observation messages */
  u8 buffer_obs_index = 0;
  for (u8 msg_num = 0; msg_num < total_messages; ++msg_num) {
    sbp_msg_t msg;
    sbp_msg_obs_t *sbp_obs = &msg.obs;
    memset(sbp_obs, 0, sizeof(*sbp_obs));

    /* Write the header */
    sbp_obs->header.t = state->obs_time;
    /* Note: SBP n_obs puts total messages in the first nibble and msg_num in
     * the second. This differs from all the other instances of n_obs in this
     * module where it is used as observation count. */
    sbp_obs->header.n_obs = (total_messages << 4) + msg_num;

    /* Write the observations */
    const u8 remaining_obs = state->obs_to_send - buffer_obs_index;
    const u8 obs_to_copy = MIN(SBP_MSG_OBS_OBS_MAX, remaining_obs);
    memcpy(sbp_obs->obs,
           &state->obs_buffer[buffer_obs_index],
           SBP_OBS_SIZE * obs_to_copy);
    buffer_obs_index += obs_to_copy;

    sbp_obs->n_obs = obs_to_copy;

    state->cb_rtcm_to_sbp(state->sender_id, SbpMsgObs, &msg, state->context);
  }
  /* clear the observation buffer, so also header.n_obs is set to zero */
  memset(state->obs_buffer, 0, OBS_BUFFER_SIZE);
  memset(&state->obs_time, 0, sizeof(state->obs_time));
  state->obs_to_send = 0;
}

void rtcm3_1005_to_sbp(const rtcm_msg_1005 *rtcm_1005,
                       sbp_msg_base_pos_ecef_t *sbp_base_pos) {
  sbp_base_pos->x = rtcm_1005->arp_x;
  sbp_base_pos->y = rtcm_1005->arp_y;
  sbp_base_pos->z = rtcm_1005->arp_z;
}

void rtcm3_1006_to_sbp(const rtcm_msg_1006 *rtcm_1006,
                       sbp_msg_base_pos_ecef_t *sbp_base_pos) {
  sbp_base_pos->x = rtcm_1006->msg_1005.arp_x;
  sbp_base_pos->y = rtcm_1006->msg_1005.arp_y;
  sbp_base_pos->z = rtcm_1006->msg_1005.arp_z;
}

void rtcm3_1033_to_sbp(const rtcm_msg_1033 *rtcm_1033,
                       sbp_msg_glo_biases_t *sbp_glo_bias) {
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
                       sbp_msg_glo_biases_t *sbp_glo_bias) {
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

/* Convert from SBP FCN (1..14 with unknown marked with 0) to
 * RTCM FCN (0..13 with unknown marked with 255) */
static u8 sbp_fcn_to_rtcm(u8 sbp_fcn) {
  if (SBP_GLO_FCN_UNKNOWN == sbp_fcn) {
    return MSM_GLO_FCN_UNKNOWN;
  }
  s8 rtcm_fcn = sbp_fcn + MSM_GLO_FCN_OFFSET - SBP_GLO_FCN_OFFSET;
  if (rtcm_fcn < 0 || rtcm_fcn > MSM_GLO_MAX_FCN) {
    log_info("Ignoring invalid GLO FCN %d", sbp_fcn);
    return MSM_GLO_FCN_UNKNOWN;
  }
  return rtcm_fcn;
}

void rtcm2sbp_set_glo_fcn(sbp_v4_gnss_signal_t sid,
                          u8 sbp_fcn,
                          struct rtcm3_sbp_state *state) {
  /* convert FCN from SBP representation to RTCM representation */
  if (sid.sat < GLO_FIRST_PRN || sid.sat >= GLO_FIRST_PRN + NUM_SATS_GLO) {
    /* invalid PRN */
    log_info("Ignoring invalid GLO PRN %u", sid.sat);
    return;
  }
  state->glo_sv_id_fcn_map[sid.sat] = sbp_fcn_to_rtcm(sbp_fcn);
}

void compute_gps_message_time(u32 tow_ms,
                              gps_time_t *obs_time,
                              const gps_time_t *rover_time) {
  if (tow_ms >= WEEK_SECS * SECS_MS) {
    log_error("Invalid GPS time received.");
    obs_time->wn = WN_UNKNOWN;
    return;
  }
  assert(gps_time_valid(rover_time));

  obs_time->tow = (double)tow_ms / SECS_MS;
  obs_time->wn = rover_time->wn;
  double timediff = gpsdifftime(obs_time, rover_time);
  if (timediff < -WEEK_SECS / 2.0 && rover_time->wn != INT16_MAX) {
    obs_time->wn = rover_time->wn + 1;
  } else if (timediff > WEEK_SECS / 2.0 && rover_time->wn != 0) {
    obs_time->wn = rover_time->wn - 1;
  }
  assert(gps_time_valid(obs_time));
}

void beidou_tow_to_gps_tow(u32 *tow_ms) {
  /* BDS system time has a constant offset */
  *tow_ms += BDS_SECOND_TO_GPS_SECOND * SECS_MS;
  if (*tow_ms >= WEEK_SECS * SECS_MS) {
    *tow_ms -= WEEK_SECS * SECS_MS;
  }
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
bool compute_glo_time(u32 tod_ms,
                      gps_time_t *obs_time,
                      const gps_time_t *rover_time,
                      struct rtcm3_sbp_state *state) {
  if (!state->leap_second_known) {
    obs_time->wn = WN_UNKNOWN;
    return false;
  }
  if (tod_ms >= (DAY_SECS + 1) * SECS_MS) {
    log_error("Invalid GLO time received.");
    obs_time->wn = WN_UNKNOWN;
    return false;
  }
  if (!gps_time_valid(rover_time)) {
    return false;
  }

  /* Approximate DOW from the reference GPS time */
  u8 glo_dow = (u8)(rover_time->tow / DAY_SECS);
  s32 glo_tod_ms = tod_ms - UTC_SU_OFFSET * HOUR_SECS * SECS_MS;

  obs_time->wn = rover_time->wn;
  s32 tow_ms =
      glo_dow * DAY_SECS * SECS_MS + glo_tod_ms + state->leap_seconds * SECS_MS;
  while (tow_ms < 0) {
    tow_ms += WEEK_SECS * SECS_MS;
    obs_time->wn -= 1;
  }
  obs_time->tow = tow_ms / (double)SECS_MS;

  if (!normalize_gps_time_safe(obs_time)) {
    return false;
  }

  /* check for day rollover against reference time */
  double timediff = gpsdifftime(obs_time, rover_time);
  if (fabs(timediff) > DAY_SECS / 2.0) {
    obs_time->tow += (timediff < 0 ? 1 : -1) * DAY_SECS;
    return normalize_gps_time_safe(obs_time);
  }

  return true;
}

static void validate_base_obs_sanity(struct rtcm3_sbp_state *state,
                                     const gps_time_t *obs_time,
                                     const gps_time_t *rover_time) {
  if (!gps_time_valid(obs_time) || !gps_time_valid(rover_time)) {
    return;
  }

  double timediff = gpsdifftime(rover_time, obs_time);

  /* exclude base measurements with time stamp too far in the future */
  if (timediff >= BASE_FUTURE_THRESHOLD_S &&
      state->cb_base_obs_invalid != NULL) {
    state->cb_base_obs_invalid(timediff, state->context);
  }
}

bool no_1230_received(struct rtcm3_sbp_state *state) {
  if (!gps_time_valid(&state->last_1230_received) ||
      gpsdifftime(&state->time_from_input_data, &state->last_1230_received) >
          MSG_1230_TIMEOUT_SEC) {
    return true;
  }
  return false;
}

void send_1029(rtcm_msg_1029 *msg_1029, struct rtcm3_sbp_state *state) {
  uint8_t message[SBP_MAX_PAYLOAD_LEN] = {0};
  uint8_t preamble_size = sizeof(RTCM_LOG_PREAMBLE) - 1;
  uint8_t message_size =
      MIN(SBP_MSG_LOG_TEXT_MAX, preamble_size + msg_1029->utf8_code_units_n);

  memcpy(message, RTCM_LOG_PREAMBLE, preamble_size);
  memcpy(message + preamble_size,
         msg_1029->utf8_code_units,
         message_size - preamble_size);

  /* Check if we've had to truncate the string - we can check for the bit
   * pattern that denotes a 4 byte code unit as it is the super set of all bit
   * patterns (2,3 and 4 byte code units) */
  if (message_size == SBP_MSG_LOG_TEXT_MAX) {
    if ((message[message_size - 1] & 0xF8) == 0xF0 ||
        (message[message_size - 1] & 0xE0) == 0xE0 ||
        (message[message_size - 1] & 0xE0) == 0xC0) {
      /* We've truncated a 2, 3 or 4 byte code unit */
      message_size--;
    } else if ((message[message_size - 1] & 0xF8) == 0xF0 ||
               (message[message_size - 1] & 0xE0) == 0xE0) {
      /* We've truncated a 3 or 4 byte code unit */
      message_size -= 2;
    } else if ((message[message_size - 1] & 0xF8) == 0xF0) {
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
  if (length >= SBP_MAX_PAYLOAD_LEN) {
    return;
  }

  sbp_msg_t msg;
  sbp_msg_log_t *sbp_log_msg = &msg.log;
  sbp_msg_log_text_init(sbp_log_msg);
  sbp_log_msg->level = level;

  if (!sbp_msg_log_text_set_raw(
          sbp_log_msg, (const char *)message, length, false, NULL)) {
    return;
  }

  state->cb_rtcm_to_sbp(
      rtcm_stn_to_sbp_sender_id(stn_id), SbpMsgLog, &msg, state->context);
}

void send_MSM_warning(const swiftnav_in_bitstream_t *bitstream,
                      struct rtcm3_sbp_state *state) {
  if (!state->sent_msm_warning) {
    /* Only send 1 warning */
    state->sent_msm_warning = true;
    /* Get the stn ID as well */
    uint32_t stn_id = 0;
    if (swiftnav_in_bitstream_getbitu(bitstream, &stn_id, 12, 24)) {
      uint8_t msg[39] = "MSM1-3 Messages currently not supported";
      send_sbp_log_message(
          RTCM_MSM_LOGGING_LEVEL, msg, sizeof(msg), stn_id, state);
    }
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

/* return true if conversion to SID succeeded, and the SID as a pointer */
static bool get_sid_from_msm(const rtcm_msm_header *header,
                             u8 satellite_index,
                             u8 signal_index,
                             sbp_v4_gnss_signal_t *sid,
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

static bool unsupported_signal(sbp_v4_gnss_signal_t *sid) {
  switch (sid->code) {
    case CODE_GLO_L1P:
    case CODE_GLO_L2P:
    case CODE_BDS3_B1CI:
    case CODE_BDS3_B1CQ:
    case CODE_BDS3_B1CX:
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

void add_msm_obs_to_buffer(const rtcm_msm_message *new_rtcm_obs,
                           struct rtcm3_sbp_state *state) {
  rtcm_constellation_t cons = to_constellation(new_rtcm_obs->header.msg_num);

  gps_time_t obs_time = GPS_TIME_UNKNOWN;
  if (RTCM_CONSTELLATION_GLO == cons) {
    compute_glo_time(new_rtcm_obs->header.tow_ms,
                     &obs_time,
                     &state->time_from_input_data,
                     state);
    if (!gps_time_valid(&obs_time)) {
      return;
    }

    const bool within_input_data_time =
        fabs(gpsdifftime(&obs_time, &state->time_from_input_data) -
             state->leap_seconds) <= GLO_SANITY_THRESHOLD_S;

    const bool within_last_gps_time =
        gps_time_valid(&state->last_gps_time) &&
        fabs(gpsdifftime(&obs_time, &state->last_gps_time) -
             state->leap_seconds) <= GLO_SANITY_THRESHOLD_S;

    if (!within_input_data_time && !within_last_gps_time) {
      return;
    }

  } else {
    u32 tow_ms = new_rtcm_obs->header.tow_ms;

    if (RTCM_CONSTELLATION_BDS == cons) {
      beidou_tow_to_gps_tow(&tow_ms);
    }

    compute_gps_time(tow_ms, &obs_time, &state->time_from_input_data, state);

    if (!gps_time_valid(&obs_time)) {
      return;
    }
  }

  /* We see some receivers output observations with TOW = 0 on startup, but this
   * can cause a large time jump when the actual time is decoded. As such,
   * ignore TOW = 0 if these are the first time stamps we see */
  if (!gps_time_valid(&state->last_gps_time) &&
      fabs(obs_time.tow) <= FLOAT_EQUALITY_EPS) {
    return;
  }

  if (!is_msm_active(&obs_time, state) && state->obs_to_send > 0) {
    /* This is the first MSM observation, so clear the already decoded legacy
     * messages from the observation buffer to avoid duplicates */
    memset(state->obs_buffer, 0, sizeof(state->obs_buffer));
    memset(&state->obs_time, 0, sizeof(state->obs_time));
    state->obs_to_send = 0;
  }

  state->last_gps_time = obs_time;
  state->last_glo_time = obs_time;
  state->last_msm_received = obs_time;

  /* Transform the newly received obs to sbp */
  sbp_v4_gps_time_t new_obs_time;

  /* Build an SBP time stamp */
  new_obs_time.wn = obs_time.wn;
  new_obs_time.tow = (u32)rint(obs_time.tow * SECS_MS);
  new_obs_time.ns_residual = 0;

  /* Check if the buffer already has obs of the same time */
  if (state->obs_to_send != 0 &&
      (state->obs_time.tow != new_obs_time.tow ||
       state->sender_id !=
           rtcm_stn_to_sbp_sender_id(new_rtcm_obs->header.stn_id))) {
    /* We either have missed a message, or we have a new station. Either way,
      send through the current buffer and clear before adding new obs */
    send_buffer_not_empty_warning(state);
    send_observations(state);
  }

  state->obs_time = new_obs_time;
  state->sender_id = rtcm_stn_to_sbp_sender_id(new_rtcm_obs->header.stn_id);

  // Convert new obs directly in to state buffer
  uint8_t num_sats = msm_get_num_satellites(&new_rtcm_obs->header);
  uint8_t num_sigs = msm_get_num_signals(&new_rtcm_obs->header);

  u8 cell_index = 0;
  for (u8 sat = 0; sat < num_sats; sat++) {
    for (u8 sig = 0; sig < num_sigs; sig++) {
      if (new_rtcm_obs->header.cell_mask[sat * num_sigs + sig]) {
        sbp_v4_gnss_signal_t sid = {CODE_INVALID, 0};
        const rtcm_msm_signal_data *data = &new_rtcm_obs->signals[cell_index];
        bool sid_valid =
            get_sid_from_msm(&new_rtcm_obs->header, sat, sig, &sid, state);
        bool constel_valid =
            sid_valid && !constellation_mask[code_to_constellation(sid.code)];
        bool supported = sid_valid && !unsupported_signal(&sid);
        if (sid_valid && constel_valid && supported &&
            data->flags.fields.valid_pr) {
          if (state->obs_to_send >= MAX_OBS_PER_EPOCH) {
            send_buffer_full_error(state);
            return;
          }

          /* get GLO FCN */
          uint8_t glo_fcn = MSM_GLO_FCN_UNKNOWN;
          msm_get_glo_fcn(&new_rtcm_obs->header,
                          sat,
                          new_rtcm_obs->sats[sat].glo_fcn,
                          state->glo_sv_id_fcn_map,
                          &glo_fcn);

          double freq;
          bool freq_valid =
              msm_signal_frequency(&new_rtcm_obs->header, sig, glo_fcn, &freq);

          double code_bias_m, phase_bias_c;
          msm_glo_fcn_bias(
              &new_rtcm_obs->header, sig, glo_fcn, &code_bias_m, &phase_bias_c);

          sbp_packed_obs_content_t *sbp_freq =
              &state->obs_buffer[state->obs_to_send];

          sbp_freq->flags = 0;
          sbp_freq->P = 0.0;
          sbp_freq->L.i = 0;
          sbp_freq->L.f = 0.0;
          sbp_freq->D.i = 0;
          sbp_freq->D.f = 0.0;
          sbp_freq->cn0 = 0.0;
          sbp_freq->lock = 0.0;

          sbp_freq->sid = sid;

          if (data->flags.fields.valid_pr) {
            double pseudorange_m = data->pseudorange_ms * GPS_C / 1000;
            pseudorange_m += sbp_signal_biases[sbp_freq->sid.code];
            pseudorange_m += code_bias_m;
            sbp_freq->P = (u32)rint(pseudorange_m * MSG_OBS_P_MULTIPLIER);
            sbp_freq->flags |= MSG_OBS_FLAGS_CODE_VALID;
          }
          if (data->flags.fields.valid_cp && freq_valid) {
            double carrier_phase_cyc = data->carrier_phase_ms * freq / 1000;
            carrier_phase_cyc += phase_bias_c;
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

          if (data->flags.fields.valid_cnr) {
            sbp_freq->cn0 = (u8)rint(data->cnr * MSG_OBS_CN0_MULTIPLIER);
          } else {
            sbp_freq->cn0 = 0;
          }

          if (data->flags.fields.valid_lock) {
            sbp_freq->lock = rtcm3_encode_lock_time(data->lock_time_s);
          }

          if (data->flags.fields.valid_dop && freq_valid) {
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

          if (verbosity_level > VERB_HIGH) {
            log_info("OBS: %3d %8s %14.3lf %14.3lf %14.3lf %2d 0x%02x",
                     sbp_freq->sid.sat,
                     code_to_string(sbp_freq->sid.code),
                     sbp_freq->P / MSG_OBS_P_MULTIPLIER,
                     sbp_freq->L.i + sbp_freq->L.f / MSG_OBS_LF_MULTIPLIER,
                     sbp_freq->cn0 / MSG_OBS_CN0_MULTIPLIER,
                     sbp_freq->lock,
                     sbp_freq->flags);
          }

          state->obs_to_send++;
        }
        cell_index++;
      }
    }
  }
}

/**
 * Processes a stream of data, converting RTCM messages in to SBP and outputting
 * via the callback functions in the `state` parameter. This function assumes
 * that the `rtcm3_sbp_state` object has already been fully populated.
 * `read_func` will be called repeatedly until it returns a zero or negative
 * value.
 *
 * Note: See `main()` in rtcm3tosbp.c for an example of how to use this function
 *
 * @param state An already populated state object
 * @param read_stream_func The function to call to read from the stream. `buf`
 * is the buffer to write data into, `len` is the size of the buffer in bytes,
 * `ctx` is the context pointer from the `state` argument, the return value is
 * the number of bytes written into `buf` with negative values indicating an
 * error.
 * @return The last value of read_func, either 0 or a negative value indicating
 * an error.
 */
int rtcm2sbp_process(struct rtcm3_sbp_state *state,
                     int (*read_stream_func)(uint8_t *buf,
                                             size_t len,
                                             void *ctx)) {
  uint8_t inbuf[RTCM3_BUFFER_SIZE];
  ssize_t read_sz = read_stream_func(inbuf, RTCM3_BUFFER_SIZE, state->context);
  if (read_sz < 1) {
    return read_sz;
  }

  ssize_t write_sz = fifo_write(&state->fifo, inbuf, read_sz);
  if (write_sz != read_sz) {
    log_info("%" PRI_SSIZE " bytes read but only %" PRI_SSIZE
             " written to FIFO",
             FORMAT_SSIZE(read_sz),
             FORMAT_SSIZE(write_sz));
    fifo_init(&state->fifo, state->fifo_buf, RTCM3_FIFO_SIZE);
  }

  uint8_t buf[RTCM3_FIFO_SIZE] = {0};
  fifo_size_t avail_sz = fifo_peek(&state->fifo, buf, RTCM3_FIFO_SIZE);

  uint32_t index = 0;
  while (index + RTCM3_MSG_OVERHEAD < avail_sz) {
    if (RTCM3_PREAMBLE != buf[index]) {
      index++;
      continue;
    }

    uint16_t msg_len = extract_msg_len(&buf[index]);
    if ((msg_len == 0) || (msg_len > RTCM3_MAX_MSG_LEN)) {
      index++;
      continue;
    }
    if (index + RTCM3_MSG_OVERHEAD + msg_len > avail_sz) {
      break;
    }

    uint8_t *rtcm_msg = &buf[index];
    if (!verify_crc(rtcm_msg, RTCM3_FIFO_SIZE - index)) {
      index++;
      continue;
    }

    rtcm2sbp_decode_frame(rtcm_msg, msg_len + RTCM3_MSG_OVERHEAD, state);
    index += msg_len + RTCM3_MSG_OVERHEAD;
  }

  fifo_size_t removed_sz = fifo_remove(&state->fifo, index);
  if (removed_sz != index) {
    log_info("Tried to remove %" PRIu32 " bytes from FIFO, only got %" PRIu32,
             index,
             removed_sz);
    fifo_init(&state->fifo, state->fifo_buf, RTCM3_FIFO_SIZE);
  }

  return read_sz;
}

static uint16_t extract_msg_len(const uint8_t *buf) {
  return (((uint16_t)buf[1] << 8) | buf[2]) & RTCM3_MAX_MSG_LEN;
}

/* buf_len is the total allocated space - can be much bigger than
   the actual message */
static bool verify_crc(uint8_t *buf, uint16_t buf_len) {
#ifdef GNSS_CONVERTERS_DISABLE_CRC_VALIDATION
  return true;
#endif
  uint16_t msg_len = extract_msg_len(buf);
  if (buf_len < msg_len + RTCM3_MSG_OVERHEAD) {
    log_info("CRC failure! Buffer %u too short for message length %u",
             buf_len,
             msg_len);
    return false;
  }
  uint32_t computed_crc = crc24q(buf, 3 + msg_len, 0);
  uint32_t frame_crc = (buf[msg_len + 3] << 16) | (buf[msg_len + 4] << 8) |
                       (buf[msg_len + 5] << 0);
  if (frame_crc != computed_crc) {
    log_info("CRC failure! frame: %08" PRIX32 " computed: %08" PRIX32,
             frame_crc,
             computed_crc);
  }
  return (frame_crc == computed_crc);
}
