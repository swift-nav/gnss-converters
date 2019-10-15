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
#include <rtcm3/eph_encode.h>
#include <rtcm3/logging.h>
#include <rtcm3/ssr_decode.h>
#include <swiftnav/edc.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/fifo_byte.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/sid_set.h>
#include <swiftnav/signal.h>

#include "gnss-converters/sbp_rtcm3.h"
#include "rtcm3_utils.h"
#include "sbp_rtcm3_internal.h"

#define SBP_GLO_FCN_OFFSET 8
#define SBP_GLO_FCN_UNKNOWN 0

#define RTCM3_PREAMBLE 0xD3
#define RTCM3_MAX_MSG_LEN 0x3FF

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
}

/* Difference between two sbp time stamps in seconds */
double sbp_diff_time(const sbp_gps_time_t *end,
                     const sbp_gps_time_t *beginning) {
  s32 week_diff = end->wn - beginning->wn;
  double dt = (double)end->tow / SECS_MS - (double)beginning->tow / SECS_MS;
  dt += week_diff * SEC_IN_WEEK;
  return dt;
}

static u16 sbp_sender_to_rtcm_stn_id(u16 sbp_id) {
  /* Read the lowest 12 bits of the sender ID.
   * Note that the conversion betweeen SBP sender ID and RTCM station ID thus is
   * not reversible */
  return sbp_id & 0x0FFF;
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
    case 1019: {
      rtcm_msg_eph *msg_1019 = (rtcm_msg_eph *)rtcm_msg;
      return rtcm3_encode_gps_eph(msg_1019, buff);
    }
    case 1033: {
      rtcm_msg_1033 *msg_1033 = (rtcm_msg_1033 *)rtcm_msg;
      return rtcm3_encode_1033(msg_1033, buff);
    }
    case 1045: {
      rtcm_msg_eph *msg_1045 = (rtcm_msg_eph *)rtcm_msg;
      return rtcm3_encode_gal_eph_fnav(msg_1045, buff);
    }
    case 1046: {
      rtcm_msg_eph *msg_1046 = (rtcm_msg_eph *)rtcm_msg;
      return rtcm3_encode_gal_eph_inav(msg_1046, buff);
    }
    case 1230: {
      rtcm_msg_1230 *msg_1230 = (rtcm_msg_1230 *)rtcm_msg;
      return rtcm3_encode_1230(msg_1230, buff);
    }
    case 1074:
    case 1084:
    case 1094:
    case 1114:
    case 1124: {
      rtcm_msm_message *msg_msm = (rtcm_msm_message *)rtcm_msg;
      return rtcm3_encode_msm4(msg_msm, buff);
    }
    case 1075:
    case 1085:
    case 1095:
    case 1115:
    case 1125: {
      rtcm_msm_message *msg_msm = (rtcm_msm_message *)rtcm_msg;
      return rtcm3_encode_msm5(msg_msm, buff);
    }
    case 4062: {
      rtcm_msg_swift_proprietary *swift_msg =
          (rtcm_msg_swift_proprietary *)rtcm_msg;
      return rtcm3_encode_4062(swift_msg, buff);
    }
    default:
      log_error("%d is not a supported message", message_type);
      assert(0);
      break;
  }
  return 0;
}

/* Encode RTCM frame into the given buffer of at least RTCM3_MAX_MSG_LEN bytes,
 * returns frame length */
u16 encode_rtcm3_frame(const void *rtcm_msg, u16 message_type, u8 *frame) {
  memset(frame, 0, RTCM3_MAX_MSG_LEN);

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
  // -1 to account for NULL terminator
  strncpy(
      state->ant_descriptor, ant_descriptor, sizeof(state->ant_descriptor) - 1);
  strncpy(
      state->rcv_descriptor, rcv_descriptor, sizeof(state->rcv_descriptor) - 1);
  state->ant_known = true;
};

void gps_tow_to_beidou_tow(u32 *tow_ms) {
  /* BDS system time has a constant offset */
  if (*tow_ms < BDS_SECOND_TO_GPS_SECOND * S_TO_MS) {
    *tow_ms += SEC_IN_WEEK * S_TO_MS;
  }
  *tow_ms -= BDS_SECOND_TO_GPS_SECOND * S_TO_MS;
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
  bool freq_valid = msm_signal_frequency(
      &msg->header, signal_index, sat_data->glo_fcn, &freq);

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
  u8 frame[RTCM3_MAX_MSG_LEN];
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

  u8 frame[RTCM3_MAX_MSG_LEN];
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

void sbp2rtcm_sbp_osr_cb(const u16 sender_id,
                         const u8 len,
                         const u8 msg[],
                         struct rtcm3_out_state *state) {
  rtcm_msg_swift_proprietary osr_msg;
  osr_msg.msg_type = SBP_MSG_OSR;
  osr_msg.sender_id = sender_id;
  osr_msg.len = len;
  memcpy(osr_msg.data, msg, len);

  u8 frame[RTCM3_MAX_MSG_LEN];
  u16 frame_size = encode_rtcm3_frame(&osr_msg, 4062, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

void sbp2rtcm_sbp_ssr_orbit_clock_cb(const u16 sender_id,
                                     const u8 len,
                                     const u8 msg[],
                                     struct rtcm3_out_state *state) {
  rtcm_msg_swift_proprietary ssr_msg;
  ssr_msg.msg_type = SBP_MSG_SSR_ORBIT_CLOCK;
  ssr_msg.sender_id = sender_id;
  ssr_msg.len = len;
  memcpy(ssr_msg.data, msg, len);

  u8 frame[RTCM3_MAX_MSG_LEN];
  u16 frame_size = encode_rtcm3_frame(&ssr_msg, 4062, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

void sbp2rtcm_sbp_ssr_phase_biases_cb(const u16 sender_id,
                                      const u8 len,
                                      const u8 msg[],
                                      struct rtcm3_out_state *state) {
  rtcm_msg_swift_proprietary ssr_msg;
  ssr_msg.msg_type = SBP_MSG_SSR_PHASE_BIASES;
  ssr_msg.sender_id = sender_id;
  ssr_msg.len = len;
  memcpy(ssr_msg.data, msg, len);

  u8 frame[RTCM3_MAX_MSG_LEN];
  u16 frame_size = encode_rtcm3_frame(&ssr_msg, 4062, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

void sbp2rtcm_sbp_ssr_code_biases_cb(const u16 sender_id,
                                     const u8 len,
                                     const u8 msg[],
                                     struct rtcm3_out_state *state) {
  rtcm_msg_swift_proprietary ssr_msg;
  ssr_msg.msg_type = SBP_MSG_SSR_CODE_BIASES;
  ssr_msg.sender_id = sender_id;
  ssr_msg.len = len;
  memcpy(ssr_msg.data, msg, len);

  u8 frame[RTCM3_MAX_MSG_LEN];
  u16 frame_size = encode_rtcm3_frame(&ssr_msg, 4062, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

void sbp2rtcm_sbp_ssr_gridded_correction_cb(const u16 sender_id,
                                            const u8 len,
                                            const u8 msg[],
                                            struct rtcm3_out_state *state) {
  rtcm_msg_swift_proprietary ssr_msg;
  ssr_msg.msg_type = SBP_MSG_SSR_GRIDDED_CORRECTION;
  ssr_msg.sender_id = sender_id;
  ssr_msg.len = len;
  memcpy(ssr_msg.data, msg, len);

  u8 frame[RTCM3_MAX_MSG_LEN];
  u16 frame_size = encode_rtcm3_frame(&ssr_msg, 4062, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

void sbp2rtcm_sbp_ssr_grid_definition_cb(const u16 sender_id,
                                         const u8 len,
                                         const u8 msg[],
                                         struct rtcm3_out_state *state) {
  rtcm_msg_swift_proprietary ssr_msg;
  ssr_msg.msg_type = SBP_MSG_SSR_GRID_DEFINITION;
  ssr_msg.sender_id = sender_id;
  ssr_msg.len = len;
  memcpy(ssr_msg.data, msg, len);

  u8 frame[RTCM3_MAX_MSG_LEN];
  u16 frame_size = encode_rtcm3_frame(&ssr_msg, 4062, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

void sbp2rtcm_sbp_ssr_stec_correction_cb(const u16 sender_id,
                                         const u8 len,
                                         const u8 msg[],
                                         struct rtcm3_out_state *state) {
  rtcm_msg_swift_proprietary ssr_msg;
  ssr_msg.msg_type = SBP_MSG_SSR_STEC_CORRECTION;
  ssr_msg.sender_id = sender_id;
  ssr_msg.len = len;
  memcpy(ssr_msg.data, msg, len);

  u8 frame[RTCM3_MAX_MSG_LEN];
  u16 frame_size = encode_rtcm3_frame(&ssr_msg, 4062, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

void sbp2rtcm_sbp_gps_eph_cb(const u16 sender_id,
                             const u8 len,
                             const u8 msg[],
                             struct rtcm3_out_state *state) {
  (void)len;
  (void)sender_id;
  msg_ephemeris_gps_t *sbp_gps_eph = (msg_ephemeris_gps_t *)msg;

  rtcm_msg_eph msg_gps_eph;
  u8 frame[RTCM3_MAX_MSG_LEN];  // Max RTCM message length is 1023 Bytes

  /* generate and send the gps ephemeris message */
  sbp_to_rtcm3_gps_eph(sbp_gps_eph, &msg_gps_eph, state);
  u16 frame_size = encode_rtcm3_frame(&msg_gps_eph, 1019, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}

void sbp2rtcm_sbp_gal_eph_cb(const u16 sender_id,
                             const u8 len,
                             const u8 msg[],
                             struct rtcm3_out_state *state) {
  (void)len;
  (void)sender_id;
  msg_ephemeris_gal_t *sbp_gal_eph = (msg_ephemeris_gal_t *)msg;

  rtcm_msg_eph msg_gal_eph;
  u8 frame[RTCM3_MAX_MSG_LEN];  // Max RTCM message length is 1023 Bytes

  /* generate and send the gal ephemeris message - message type depends on
   * source */
  sbp_to_rtcm3_gal_eph(sbp_gal_eph, &msg_gal_eph, state);

  u16 message_type = sbp_gal_eph->source == EPH_SOURCE_GAL_INAV ? 1046 : 1045;
  u16 frame_size = encode_rtcm3_frame(&msg_gal_eph, message_type, frame);
  state->cb_sbp_to_rtcm(frame, frame_size, state->context);
}
