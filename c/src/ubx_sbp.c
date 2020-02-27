/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gnss-converters/ubx_sbp.h>
#include <libsbp/imu.h>
#include <libsbp/orientation.h>
#include <libsbp/vehicle.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <swiftnav/common.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/signal.h>
#include <ubx/decode.h>
#include <ubx/ubx_messages.h>
#include <ubx_ephemeris/bds.h>
#include <ubx_ephemeris/gal.h>
#include <ubx_ephemeris/gps.h>

/* TODO(STAR-918) should probably consolidate these into central .h file */
#define SBP_OBS_LF_MULTIPLIER 256
#define SBP_OBS_DF_MULTIPLIER 256

#define UBX_SBP_LAT_LON_SCALING (1e-7)
#define UBX_SBP_HEIGHT_SCALING (1e-3)

#define UBX_HNR_PVT_FIX_TYPE_NONE (0x00)
#define UBX_HNR_PVT_FIX_TYPE_DEAD_RECKONING (0x01)
#define UBX_HNR_PVT_FIX_TYPE_2D (0x02)
#define UBX_HNR_PVT_FIX_TYPE_3D (0x03)
#define UBX_HNR_PVT_FIX_TYPE_COMBINED (0x04)
#define UBX_HNR_PVT_FIX_TYPE_TIME (0x05)

#define UBX_HNR_PVT_DGNSS_MASK (0x02)

#define UBX_NAV_ATT_SCALING (10)       /* 1e-5->1e-6 degrees */
#define UBX_NAV_ATT_ACC_SCALING (1e-5) /* 1e-5->1 degrees */

#define SBP_ORIENT_EULER_INS_MASK (0x01)

#define UBX_NAV_PVT_FIX_TYPE_NONE (0x00)
#define UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING (0x01)
#define UBX_NAV_PVT_FIX_TYPE_2D (0x02)
#define UBX_NAV_PVT_FIX_TYPE_3D (0x03)
#define UBX_NAV_PVT_FIX_TYPE_COMBINED (0x04)
#define UBX_NAV_PVT_FIX_TYPE_TIME (0x05)

#define UBX_NAV_PVT_DGNSS_MASK (0x02)
#define UBX_NAV_PVT_INS_MASK (0x04)

#define UBX_NAV_PVT_CARR_SOLN_MASK (0xC0)
#define UBX_NAV_PVT_SOLN_NO_PHASE (0x00)
#define UBX_NAV_PVT_SOLN_FLOAT_RTK (0x40)
#define UBX_NAV_PVT_SOLN_FIXED_RTK (0x80)

#define UBX_NAV_VELECEF_SCALING (10) /* cm->mm */

#define SBP_LLH_INVALID_SOLN_MASK (0x00)
#define SBP_LLH_FLOAT_RTK_MASK (0x03)
#define SBP_LLH_FIXED_RTK_MASK (0x04)
#define SBP_LLH_DEAD_RECKONING_MASK (0x05)
#define SBP_LLH_DGNSS_MASK (0x02)
#define SBP_LLH_SPP_MASK (0x01)
#define SBP_LLH_INS_MASK (0x08)

#define UBX_SBP_TOW_MS_SCALING (1e3)
#define UBX_SBP_TOW_NS_SCALING (1e6)

#define UBX_SBP_PSEUDORANGE_SCALING (50)

#define SBP_OBS_DOPPLER_MASK (1 << 3)
#define SBP_OBS_TRACK_STATE_MASK (0x07)

/* Scale factor to go from ESF-RAW sensor time tags to seconds.
 * This value is not specified in the UBX protocol spec and has been derived
 * from the Bosch BMI 160 datasheet, so it might only be valid for the M8L,
 * which we know uses this IMU.
 */
static const double ubx_sensortime_scale = 39.0625e-6;

/* A single SBP message can only fit a maximum number of observations in it
 * floor((0xFF - sizeof(observation_header_t)) / sizeof(packed_obs_content_t))
 */
#define SBP_MAX_NUM_OBS 14

/* Syncs data between HNR_PVT and NAV_PVT */
struct ubx_pvt_state {
  u8 num_sats;
  u8 flags;
  u8 fix_type;
};

static struct ubx_pvt_state pvt_state;

static int reload_ubx_buffer(struct ubx_sbp_state *state) {
  state->index = 0;
  int bytes_read = state->read_stream_func(
      state->read_buffer, UBX_BUFFER_SIZE, state->context);
  state->bytes_in_buffer = bytes_read > 0 ? bytes_read : 0;
  return bytes_read;
}

/** Attempts to read `length` bytes into `buf`, implemented as a memcpy from
 * state->read_buffer. If there are not enough bytes in the read_buffer, we
 * attempt to reload the read_buffer. If we still do not have sufficient bytes,
 * we assume EOF reached and return -1.
 */
static int read_ubx_bytes(u8 *buf, size_t length, struct ubx_sbp_state *state) {
  int bytes_read = 0;
  while (state->index + length > state->bytes_in_buffer) {
    if (state->bytes_in_buffer == 0) {
      /* Try again in case we perfectly aligned read */
      int reload_bytes = reload_ubx_buffer(state);
      if (reload_bytes <= 0) {
        return reload_bytes;
      } else {
        continue;
      }
    }

    size_t remaining_bytes = state->bytes_in_buffer - state->index;
    memcpy(
        buf + bytes_read, &state->read_buffer[state->index], remaining_bytes);
    length -= remaining_bytes;
    bytes_read += remaining_bytes;
    int reload_bytes = reload_ubx_buffer(state);
    if (reload_bytes <= 0) {
      return reload_bytes;
    }
  }

  memcpy(buf + bytes_read, &state->read_buffer[state->index], length);
  state->index += length;
  bytes_read += length;

  return bytes_read;
}

/** UBX Frame:
 * SYNC_CHAR_1 | SYNC_CHAR_2 | CLASS | MSG_ID | 2-byte Length | Payload |
 * CHCKSUM_BYTE_1 | CHCKSUM_BYTE_2
 */
static int read_ubx_frame(u8 *frame, struct ubx_sbp_state *state) {
  int ret;
  do {
    ret = read_ubx_bytes(frame, 1, state);
    if (ret <= 0) {
      return ret == 0 ? -1 : ret;
    }

    if (frame[0] != UBX_SYNC_CHAR_1) {
      continue;
    }

    while (frame[0] == UBX_SYNC_CHAR_1) {
      ret = read_ubx_bytes(frame, 1, state);
      if (ret <= 0) {
        return ret == 0 ? -1 : ret;
      }
    }

    if (frame[0] != UBX_SYNC_CHAR_2) {
      continue;
    }

    ret = read_ubx_bytes(frame, 4, state);
    if (ret <= 0) {
      return ret == 0 ? -1 : ret;
    }

    /* First two bytes are class and msg ID */
    u16 payload_length = frame[2] + (frame[3] << 8);
    /* Assume massive payload is due to corrupted message. 2 bytes for header, 2
     * bytes for length, 2 bytes for checksum not counted in payload_length
     */
    if (payload_length > UBX_FRAME_SIZE - 6) {
      fprintf(stderr,
              "UBX payload_length for class 0x%X and ID 0x%X too large: %d; "
              "possible corrupted frame\n",
              frame[0],
              frame[1],
              payload_length);
      continue;
    }

    /* +2 for checksum bytes */
    ret = read_ubx_bytes(frame + 4, payload_length + 2, state);
    if (ret <= 0) {
      return ret == 0 ? -1 : ret;
    }

    u8 checksum[2];
    ubx_checksum(frame, 4 + payload_length, (u8 *)&checksum);
    if (memcmp(&checksum, frame + 4 + payload_length, 2) != 0) {
      continue;
    } else {
      return payload_length;
    }
  } while (state->bytes_in_buffer != 0);

  return -1;
}

static code_t convert_ubx_gnssid_sigid(u8 gnss_id, u8 sig_id) {
  /* GPS L1C/A */
  if ((gnss_id == 0) && (sig_id == 0)) {
    return CODE_GPS_L1CA;
    /* GPS L2 CL */
  } else if ((gnss_id == 0) && (sig_id == 3)) {
    return CODE_GPS_L2CL;
    /* GPS L2 CM */
  } else if ((gnss_id == 0) && (sig_id == 4)) {
    return CODE_GPS_L2CM;
    /* Galileo E1 C */
  } else if ((gnss_id == 2) && (sig_id == 0)) {
    return CODE_GAL_E1C;
    /* Galileo E1 B */
  } else if ((gnss_id == 2) && (sig_id == 1)) {
    return CODE_GAL_E1B;
    /* Galileo E5 bI */
  } else if ((gnss_id == 2) && (sig_id == 5)) {
    return CODE_GAL_E7I;
    /* Galileo E5 bQ */
  } else if ((gnss_id == 2) && (sig_id == 6)) {
    return CODE_GAL_E7Q;
    /* BeiDou B1I D1 */
  } else if ((gnss_id == 3) && (sig_id == 0)) {
    return CODE_BDS2_B1;
    /* BeiDou B1I D2 */
  } else if ((gnss_id == 3) && (sig_id == 1)) {
    return CODE_BDS2_B1;
    /* BeiDou B2I D1 */
  } else if ((gnss_id == 3) && (sig_id == 2)) {
    return CODE_BDS2_B2;
    /* BeiDou B2I D2 */
  } else if ((gnss_id == 3) && (sig_id == 3)) {
    return CODE_BDS2_B2;
    /* QZSS L1C/A */
  } else if ((gnss_id == 5) && (sig_id == 0)) {
    return CODE_QZS_L1CA;
    /* QZSS L2 CM */
  } else if ((gnss_id == 5) && (sig_id == 4)) {
    return CODE_QZS_L2CM;
    /* QZSS L2 CL */
  } else if ((gnss_id == 5) && (sig_id == 5)) {
    return CODE_QZS_L2CL;
    /* GLONASS L1 OF */
  } else if ((gnss_id == 6) && (sig_id == 0)) {
    return CODE_GLO_L1OF;
    /* GLONASS L2 OF */
  } else if ((gnss_id == 6) && (sig_id == 2)) {
    return CODE_GLO_L2OF;
  }

  return CODE_INVALID;
}

static bool pack_carrier_phase(double L_in, carrier_phase_t *L_out) {
  double Li = floor(L_in);
  if (Li < INT32_MIN || Li > INT32_MAX) {
    return false;
  }

  double Lf = L_in - Li;

  L_out->i = (s32)Li;
  u16 frac_part_cp = (u16)round(Lf * SBP_OBS_LF_MULTIPLIER);
  if (frac_part_cp >= UINT8_MAX) {
    frac_part_cp = 0;
    L_out->i += 1;
  }
  L_out->f = frac_part_cp;

  return true;
}

/* TODO(STAR-920) Move pack_* functions into libsbp or libswitfnav */
static bool pack_doppler(double D_in, doppler_t *D_out) {
  double Di = floor(D_in);
  if (Di < INT16_MIN || Di > INT16_MAX) {
    return false;
  }

  double Df = D_in - Di;

  D_out->i = (s16)Di;
  u16 frac_part_d = (u16)round(Df * SBP_OBS_DF_MULTIPLIER);
  if (frac_part_d >= UINT8_MAX) {
    frac_part_d = 0;
    D_out->i += 1;
  }
  D_out->f = frac_part_d;

  return true;
}

s32 convert24b(u32 value) {
  const int MODULO = 1 << 24;
  const int MAX_VALUE = (1 << 23) - 1;
  s32 result = *(s32 *)&(value);
  if (result > MAX_VALUE) {
    result -= MODULO;
  }
  return result;
}

void check_overflow(s32 *value) {
  if (*value > INT16_MAX) {
    *value = INT16_MAX;
  }
  if (*value < INT16_MIN) {
    *value = INT16_MIN;
  }
}

static int fill_msg_obs(const u8 buf[], msg_obs_t *msg) {
  ubx_rxm_rawx rxm_rawx;
  if (ubx_decode_rxm_rawx(buf, &rxm_rawx) != RC_OK) {
    return -1;
  }

  /* convert from sec to ms */
  double ms = UBX_SBP_TOW_MS_SCALING * rxm_rawx.rcv_tow;
  double ns = UBX_SBP_TOW_NS_SCALING * modf(ms, &ms);
  msg->header.t.tow = (u32)ms;
  msg->header.t.ns_residual = (s32)ns;
  if (msg->header.t.ns_residual > 500000) {
    msg->header.t.ns_residual -= 1000000;
    msg->header.t.tow += 1;
  }

  msg->header.t.wn = rxm_rawx.rcv_wn;
  /* Not proper SBP n_obs. Needed to pass total number of measurements
   * to handle_rxm_rawx
   */
  msg->header.n_obs = rxm_rawx.num_meas;

  for (int i = 0; i < msg->header.n_obs; i++) {
    /* convert units: meter -> 2 cm */
    msg->obs[i].P =
        (u32)(UBX_SBP_PSEUDORANGE_SCALING * rxm_rawx.pseudorange_m[i]);
    pack_carrier_phase(rxm_rawx.carrier_phase_cycles[i], &msg->obs[i].L);
    pack_doppler(rxm_rawx.doppler_hz[i], &msg->obs[i].D);
    /* check for overflow */
    if (rxm_rawx.cno_dbhz[i] > 0x3F) {
      msg->obs[i].cn0 = 0xFF;
    } else {
      msg->obs[i].cn0 = 4 * rxm_rawx.cno_dbhz[i];
    }
    /* TODO(STAR-919) converts from u32 ms, to double s; encode_lock_time
     * internally converts back to u32 ms.
     */
    msg->obs[i].lock =
        encode_lock_time((double)rxm_rawx.lock_time[i] / SECS_MS);
    msg->obs[i].flags = 0;
    /* currently assumes all doppler are valid */
    msg->obs[i].flags |= SBP_OBS_DOPPLER_MASK;
    msg->obs[i].flags |= rxm_rawx.track_state[i] & SBP_OBS_TRACK_STATE_MASK;
    msg->obs[i].sid.sat = rxm_rawx.sat_id[i];
    msg->obs[i].sid.code =
        convert_ubx_gnssid_sigid(rxm_rawx.gnss_id[i], rxm_rawx.sig_id[i]);
  }

  return 0;
}

static int fill_msg_orient_euler(const u8 buf[], msg_orient_euler_t *msg) {
  ubx_nav_att nav_att;
  if (ubx_decode_nav_att(buf, &nav_att) != RC_OK) {
    return -1;
  }

  msg->tow = nav_att.i_tow;

  msg->roll = nav_att.roll * UBX_NAV_ATT_SCALING;
  msg->pitch = nav_att.pitch * UBX_NAV_ATT_SCALING;
  msg->yaw = nav_att.heading * UBX_NAV_ATT_SCALING;
  msg->roll_accuracy = (float)(nav_att.acc_roll * UBX_NAV_ATT_ACC_SCALING);
  msg->pitch_accuracy = (float)(nav_att.acc_pitch * UBX_NAV_ATT_ACC_SCALING);
  msg->yaw_accuracy = (float)(nav_att.acc_heading * UBX_NAV_ATT_ACC_SCALING);

  msg->flags = 0;
  if ((pvt_state.fix_type == UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING) ||
      (pvt_state.fix_type == UBX_NAV_PVT_FIX_TYPE_COMBINED)) {
    msg->flags |= SBP_ORIENT_EULER_INS_MASK;
  }

  return 0;
}

static int fill_msg_pos_llh(const u8 buf[], msg_pos_llh_t *msg) {
  ubx_nav_pvt nav_pvt;
  if (ubx_decode_nav_pvt(buf, &nav_pvt) != RC_OK) {
    return -1;
  }

  msg->tow = nav_pvt.i_tow;
  msg->lat = UBX_SBP_LAT_LON_SCALING * nav_pvt.lat;
  msg->lon = UBX_SBP_LAT_LON_SCALING * nav_pvt.lon;
  /* convert from mm to m */
  msg->height = UBX_SBP_HEIGHT_SCALING * nav_pvt.height;
  /* bounding u32 -> u16 conversion */
  u16 max_accuracy = UINT16_MAX;
  msg->h_accuracy = (u16)(nav_pvt.horizontal_accuracy > max_accuracy
                              ? max_accuracy
                              : nav_pvt.horizontal_accuracy);
  msg->v_accuracy = (u16)(nav_pvt.vertical_accuracy > max_accuracy
                              ? max_accuracy
                              : nav_pvt.vertical_accuracy);
  msg->n_sats = nav_pvt.num_sats;
  pvt_state.num_sats = nav_pvt.num_sats;

  msg->flags = 0;
  if ((nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_NONE) ||
      (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_TIME)) {
    msg->flags |= SBP_LLH_INVALID_SOLN_MASK;
  } else if (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING) {
    msg->flags |= SBP_LLH_DEAD_RECKONING_MASK;
  } else if ((nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_2D) ||
             (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_3D) ||
             (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_COMBINED)) {
    if ((nav_pvt.flags & UBX_NAV_PVT_DGNSS_MASK) == 0) {
      msg->flags |= SBP_LLH_SPP_MASK;
    } else {
      u8 carr_soln = nav_pvt.flags & UBX_NAV_PVT_CARR_SOLN_MASK;
      switch (carr_soln) {
        case UBX_NAV_PVT_SOLN_NO_PHASE:
          msg->flags |= SBP_LLH_DGNSS_MASK;
          break;
        case UBX_NAV_PVT_SOLN_FLOAT_RTK:
          msg->flags |= SBP_LLH_FLOAT_RTK_MASK;
          break;
        case UBX_NAV_PVT_SOLN_FIXED_RTK:
          msg->flags |= SBP_LLH_FIXED_RTK_MASK;
          break;
        default:
          fprintf(stderr,
                  "Invalid NAV_PVT.flags carrier solution value: %d\n",
                  carr_soln);
          break;
      }
    }
  }

  if ((nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING) ||
      (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_COMBINED)) {
    msg->flags |= SBP_LLH_INS_MASK;
  }

  pvt_state.flags = msg->flags;
  pvt_state.fix_type = nav_pvt.fix_type;

  return 0;
}

static int fill_msg_vel_ecef(const u8 buf[], msg_vel_ecef_t *msg) {
  ubx_nav_velecef nav_velecef;
  if (ubx_decode_nav_velecef(buf, &nav_velecef) != RC_OK) {
    return -1;
  }

  msg->tow = nav_velecef.i_tow;
  msg->x = nav_velecef.ecefVX * UBX_NAV_VELECEF_SCALING;
  msg->y = nav_velecef.ecefVY * UBX_NAV_VELECEF_SCALING;
  msg->z = nav_velecef.ecefVZ * UBX_NAV_VELECEF_SCALING;
  msg->accuracy = nav_velecef.speed_acc;
  msg->n_sats = pvt_state.num_sats;
  msg->flags = 0;
  /* Assume either invalid or dead reckoning. Temp. hack */
  if (pvt_state.fix_type > 0) {
    msg->flags |= SBP_LLH_DEAD_RECKONING_MASK;
  } else {
    msg->flags = 0;
  }
  /* Assume INS always used */
  msg->flags |= SBP_LLH_INS_MASK;

  return 0;
}

static int fill_msg_pos_llh_hnr(const u8 buf[], msg_pos_llh_t *msg) {
  ubx_hnr_pvt hnr_pvt;
  if (ubx_decode_hnr_pvt(buf, &hnr_pvt) != RC_OK) {
    return -1;
  }

  msg->tow = hnr_pvt.i_tow;
  msg->lat = UBX_SBP_LAT_LON_SCALING * hnr_pvt.lat;
  msg->lon = UBX_SBP_LAT_LON_SCALING * hnr_pvt.lon;
  /* convert from mm to m */
  msg->height = UBX_SBP_HEIGHT_SCALING * hnr_pvt.height;
  /* bounding u32 -> u16 conversion */
  u16 max_accuracy = UINT16_MAX;
  msg->h_accuracy = (u16)(hnr_pvt.horizontal_accuracy > max_accuracy
                              ? max_accuracy
                              : hnr_pvt.horizontal_accuracy);
  msg->v_accuracy = (u16)(hnr_pvt.vertical_accuracy > max_accuracy
                              ? max_accuracy
                              : hnr_pvt.vertical_accuracy);

  msg->n_sats = pvt_state.num_sats;
  msg->flags = pvt_state.flags;

  return 0;
}

static bool is_odo(u8 data_type) {
  return (ESF_REAR_LEFT_WHEEL_TICKS == data_type) ||
         (ESF_REAR_RIGHT_WHEEL_TICKS == data_type) ||
         (ESF_FRONT_LEFT_WHEEL_TICKS == data_type) ||
         (ESF_FRONT_RIGHT_WHEEL_TICKS == data_type) ||
         (ESF_SINGLE_TICK == data_type) || (ESF_SPEED == data_type);
}

static u8 get_odo_velocity_source(u8 data_type) {
  switch (data_type) {
    case ESF_REAR_LEFT_WHEEL_TICKS:
      return 1;
    case ESF_REAR_RIGHT_WHEEL_TICKS:
      return 0;
    case ESF_FRONT_LEFT_WHEEL_TICKS:
      return 2;
    case ESF_FRONT_RIGHT_WHEEL_TICKS:
      return 3;
    case ESF_SINGLE_TICK:
      return 0;
    case ESF_SPEED:
      return 3;
    default:
      assert(false && "Unsupported data type");
      return 0;
  }
}

static void set_odo_time(u32 msss,
                         const struct ubx_esf_state *esf_state,
                         msg_odometry_t *msg_odo) {
  if (esf_state->tow_offset_set) {
    double tow_s = ubx_convert_msss_to_tow(msss, esf_state);
    const u8 time_source_gps = 1;
    msg_odo->flags = time_source_gps;
    msg_odo->tow = (u32)(tow_s * SECS_MS);
  } else {
    const u8 time_source_invalid = 0;
    msg_odo->flags = time_source_invalid;
    msg_odo->tow = msss;
  }
}

static void handle_esf_meas(struct ubx_sbp_state *state, u8 *inbuf) {
  ubx_esf_meas esf_meas;
  if (ubx_decode_esf_meas(inbuf, &esf_meas) != RC_OK) {
    return;
  }
  u8 num_meas = (esf_meas.flags >> 11) & 0x1F;

  int idx;
  for (idx = 0; idx < num_meas; idx++) {
    u32 data_type = ((u32)esf_meas.data[idx] & 0x3F000000) >> 24;
    u32 data_value = ((u32)esf_meas.data[idx] & 0xFFFFFF);

    if (is_odo(data_type)) {
      msg_odometry_t msg_odo;
      set_odo_time(esf_meas.calib_tag, &state->esf_state, &msg_odo);
      u8 velocity_source = get_odo_velocity_source(data_type);
      u32 tick_count = data_value & 0x3FFFFF;
      u32 direction = (data_value & 0x800000) >> 23;
      msg_odo.flags |= direction << 5;
      msg_odo.flags |= velocity_source << 3;
      if (ESF_SPEED == data_type) {
        msg_odo.velocity = data_value;
      } else {
        msg_odo.velocity = tick_count;
      }

      state->cb_ubx_to_sbp(SBP_MSG_ODOMETRY,
                           sizeof(msg_odo),
                           (u8 *)&msg_odo,
                           state->sender_id,
                           state->context);
    }
  }
}

double ubx_convert_msss_to_tow(u32 msss, const struct ubx_esf_state *state) {
  const double MS_SECS = 1. / SECS_MS;
  double tow = MS_SECS * msss + state->time_since_startup_tow_offset;
  if (!state->tow_offset_set) {
    return -1;
  } else if (msss < state->last_sync_msss - 100) {
    /* Rollover occured since last sync */
    const u32 msss_max = UINT32_MAX;
    const double time_since_startup_rollover = MS_SECS * msss_max + MS_SECS;
    tow += time_since_startup_rollover;
  }

  return tow;
}

struct sbp_imuraw_timespec {
  u32 tow;
  u8 tow_f;
};

static struct sbp_imuraw_timespec convert_tow_to_imuraw_time(double tow) {
  struct sbp_imuraw_timespec ret;
  double tow_in_ms = tow * SECS_MS;
  ret.tow = (u32)tow_in_ms;
  ret.tow_f = (u32)((tow_in_ms - ret.tow) * 256.0);
  return ret;
}

static s16 float_to_s16_clamped(float val) {
  s16 ret;
  if (val < INT16_MIN) {
    ret = INT16_MIN;
  } else if (val > INT16_MAX) {
    ret = INT16_MAX;
  } else {
    ret = lroundf(val);
  }
  return ret;
}

static void maybe_parse_imu_data(u32 data,
                                 u8 data_type,
                                 msg_imu_raw_t *msg,
                                 struct ubx_esf_state *esf_state) {
  s32 parsed_value = convert24b(data);
  if (data_type == ESF_X_AXIS_GYRO_ANG_RATE ||
      data_type == ESF_Y_AXIS_GYRO_ANG_RATE ||
      data_type == ESF_Z_AXIS_GYRO_ANG_RATE) {
    const float ubx_scale_angular_rate = ldexpf(1, -12);
    const float angular_rate_degs = ubx_scale_angular_rate * parsed_value;
    const float sbp_scale_rate_125degs = (float)(125.0 / 32768.0);
    s16 sbp_scaled_rate =
        float_to_s16_clamped(angular_rate_degs / sbp_scale_rate_125degs);
    switch (data_type) {
      case ESF_X_AXIS_GYRO_ANG_RATE:
        msg->gyr_x = sbp_scaled_rate;
        break;
      case ESF_Y_AXIS_GYRO_ANG_RATE:
        msg->gyr_y = sbp_scaled_rate;
        break;
      case ESF_Z_AXIS_GYRO_ANG_RATE:
        msg->gyr_z = sbp_scaled_rate;
        break;
      default:
        break;
    }
  }

  if (data_type == ESF_X_AXIS_ACCEL_SPECIFIC_FORCE ||
      data_type == ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE ||
      data_type == ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE) {
    const float ubx_scale_acc = ldexpf(1, -10);
    const float specforce_ms2 = ubx_scale_acc * parsed_value;
    const float sbp_scale_acc_4g = (float)(4.0 * 9.80665 / 32768.0);
    s16 sbp_scaled_specforce =
        float_to_s16_clamped(specforce_ms2 / sbp_scale_acc_4g);
    switch (data_type) {
      case ESF_X_AXIS_ACCEL_SPECIFIC_FORCE:
        msg->acc_x = sbp_scaled_specforce;
        break;
      case ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE:
        msg->acc_y = sbp_scaled_specforce;
        break;
      case ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE:
        msg->acc_z = sbp_scaled_specforce;
        break;
      default:
        break;
    }
  }

  if (data_type == ESF_GYRO_TEMP) {
    const double ubx_scale_temp = 0.01;
    esf_state->last_imu_temp = ubx_scale_temp * data;
  }
}

static void set_sbp_imu_time(u32 sensortime_this_message,
                             u32 sensortime_first_message,
                             u32 msss,
                             msg_imu_raw_t *msg,
                             struct ubx_esf_state *esf_state) {
  const double first_msg_tow = ubx_convert_msss_to_tow(msss, esf_state);
  const double first_msg_tss = 0.001 * msss;

  s32 sensor_time_diff = sensortime_this_message - sensortime_first_message;
  while (sensor_time_diff < 0) {
    sensor_time_diff += (1 << 24); /* unwrap 24 bit overflow */
  }

  double sensor_tow_s = ubx_sensortime_scale * sensor_time_diff + first_msg_tow;
  double sensor_tss_s = ubx_sensortime_scale * sensor_time_diff + first_msg_tss;

  double sensortime;
  if (!esf_state->tow_offset_set) {
    sensortime = sensor_tss_s;
  } else {
    sensortime = sensor_tow_s;
  }

  // This constant has been found empirically by comparing the M8L IMU angular
  // rate with a properly time stamped reference.
  const double ubx_imu_gnss_time_offset = 0.05;
  sensortime -= ubx_imu_gnss_time_offset;

  gps_time_t gps_sensortime;
  gps_sensortime.tow = sensortime;
  gps_sensortime.wn = 0;
  unsafe_normalize_gps_time(&gps_sensortime);
  sensortime = gps_sensortime.tow;

  struct sbp_imuraw_timespec timespec = convert_tow_to_imuraw_time(sensortime);
  msg->tow = timespec.tow;
  msg->tow_f = timespec.tow_f;

  if (!esf_state->tow_offset_set) {
    const u32 time_invalid_bit = (1 << 31);
    msg->tow |= time_invalid_bit;
  }
}

static bool check_imu_message_complete(const s16 *received_number_msgs,
                                       s16 current_msg_number) {
  return (received_number_msgs[ESF_X_AXIS_GYRO_ANG_RATE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_Y_AXIS_GYRO_ANG_RATE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_Z_AXIS_GYRO_ANG_RATE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_X_AXIS_ACCEL_SPECIFIC_FORCE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE] ==
              current_msg_number + 1);
}

static void send_imu_aux(struct ubx_sbp_state *state) {
  msg_imu_aux_t msg;
  memset(&msg, 0, sizeof(msg));
  const u8 imu_type_bmi160 = 0;
  msg.imu_type = imu_type_bmi160;
  const u8 gyro_range_125degs = 4;
  const u8 acc_range_4g = 1;
  msg.imu_conf = (gyro_range_125degs << 4) | acc_range_4g;
  double imu_temp_bmi160_scaled =
      (state->esf_state.last_imu_temp - 23.0) / 512.0;
  msg.temp = (s16)imu_temp_bmi160_scaled;
  state->cb_ubx_to_sbp(SBP_MSG_IMU_AUX,
                       sizeof(msg),
                       (u8 *)&msg,
                       state->sender_id,
                       state->context);
}

static bool is_imu_data(u8 data_type) {
  return (ESF_X_AXIS_GYRO_ANG_RATE == data_type) ||
         (ESF_Y_AXIS_GYRO_ANG_RATE == data_type) ||
         (ESF_Z_AXIS_GYRO_ANG_RATE == data_type) ||
         (ESF_X_AXIS_ACCEL_SPECIFIC_FORCE == data_type) ||
         (ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE == data_type) ||
         (ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE == data_type);
}

static void handle_esf_raw(struct ubx_sbp_state *state, u8 *inbuf) {
  ubx_esf_raw esf_raw;
  msg_imu_raw_t msg;
  memset(&msg, 0, sizeof(msg));
  if (ubx_decode_esf_raw(inbuf, &esf_raw) != RC_OK) {
    return;
  }

  u8 num_raw = (esf_raw.length - 4) / 8;

  const size_t MAX_MESSAGES = ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE + 1;
  s16 received_number_msgs[MAX_MESSAGES];
  memset(received_number_msgs, 0, sizeof(s16) * MAX_MESSAGES);
  s16 current_msg_number = 0;
  for (int i = 0; i < num_raw; i++) {
    u8 data_type = (esf_raw.data[i] >> 24) & 0xFF;
    u32 data = esf_raw.data[i] & 0xFFFFFF;
    set_sbp_imu_time(esf_raw.sensor_time_tag[i],
                     esf_raw.sensor_time_tag[0],
                     esf_raw.msss,
                     &msg,
                     &state->esf_state);
    maybe_parse_imu_data(data, data_type, &msg, &state->esf_state);
    if (is_imu_data(data_type)) {
      received_number_msgs[data_type]++;
      assert((received_number_msgs[data_type] == current_msg_number + 1) &&
             "Assumption of no missing IMU data violated");
    }

    if (check_imu_message_complete(received_number_msgs, current_msg_number)) {
      if (state->esf_state.imu_raw_msgs_sent % 100 == 0) {
        send_imu_aux(state);
        state->esf_state.imu_raw_msgs_sent = 0;
      }
      state->cb_ubx_to_sbp(SBP_MSG_IMU_RAW,
                           sizeof(msg),
                           (u8 *)&msg,
                           state->sender_id,
                           state->context);
      current_msg_number++;
      state->esf_state.imu_raw_msgs_sent++;
    }
  }
}

static void handle_hnr_pvt(struct ubx_sbp_state *state, u8 *inbuf) {
  msg_pos_llh_t sbp_pos_llh;
  if (fill_msg_pos_llh_hnr(inbuf, &sbp_pos_llh) == 0) {
    if (state->use_hnr) {
      state->cb_ubx_to_sbp(SBP_MSG_POS_LLH,
                           sizeof(sbp_pos_llh),
                           (u8 *)&sbp_pos_llh,
                           state->sender_id,
                           state->context);
    }
  }
}

static void handle_nav_att(struct ubx_sbp_state *state, u8 *inbuf) {
  msg_orient_euler_t sbp_orient_euler;
  if (fill_msg_orient_euler(inbuf, &sbp_orient_euler) == 0) {
    state->cb_ubx_to_sbp(SBP_MSG_ORIENT_EULER,
                         sizeof(sbp_orient_euler),
                         (u8 *)&sbp_orient_euler,
                         state->sender_id,
                         state->context);
  }
}

static void handle_nav_pvt(struct ubx_sbp_state *state, u8 *inbuf) {
  msg_pos_llh_t sbp_pos_llh;
  if (fill_msg_pos_llh(inbuf, &sbp_pos_llh) == 0) {
    if (!state->use_hnr) {
      state->cb_ubx_to_sbp(SBP_MSG_POS_LLH,
                           sizeof(sbp_pos_llh),
                           (u8 *)&sbp_pos_llh,
                           state->sender_id,
                           state->context);
    }
  }
}

static void handle_nav_velecef(struct ubx_sbp_state *state, u8 *inbuf) {
  msg_vel_ecef_t sbp_vel_ecef;
  if (fill_msg_vel_ecef(inbuf, &sbp_vel_ecef) == 0) {
    state->cb_ubx_to_sbp(SBP_MSG_VEL_ECEF,
                         sizeof(sbp_vel_ecef),
                         (u8 *)&sbp_vel_ecef,
                         state->sender_id,
                         state->context);
  }
}

static void handle_nav_status(struct ubx_sbp_state *state, u8 *inbuf) {
  ubx_nav_status nav_status;
  if (ubx_decode_nav_status(inbuf, &nav_status) != RC_OK) {
    return;
  }
  const u8 fix_type_3d = 0x03;
  const u8 fix_type_gnss_dr = 0x04;
  const u8 gps_fix_ok = 0b0001;
  bool gnss_fix_good = ((nav_status.fix_type == fix_type_3d) ||
                        (nav_status.fix_type == fix_type_gnss_dr)) &&
                       (nav_status.status_flags & gps_fix_ok);

  const u8 weeknumber_ok = 0b0100;
  const u8 tow_ok = 0b0100;
  bool time_good = (nav_status.status_flags & weeknumber_ok) &&
                   (nav_status.status_flags & tow_ok);

  if (gnss_fix_good && time_good) {
    state->esf_state.time_since_startup_tow_offset =
        0.001 * nav_status.i_tow - 0.001 * nav_status.msss;
    state->esf_state.last_sync_msss = nav_status.msss;
    state->esf_state.tow_offset_set = true;
  }
}

static void handle_rxm_rawx(struct ubx_sbp_state *state,
                            u8 *inbuf,
                            u8 *sbp_obs_buffer) {
  msg_obs_t *sbp_obs = (msg_obs_t *)sbp_obs_buffer;
  if (fill_msg_obs(inbuf, sbp_obs) == 0) {
    u8 total_messages;
    /* header.n_obs is assumed to hold the TOTAL number of observations
     * from fill_msg_obs, NOT n_obs as described in the SBP spec.
     */
    if (sbp_obs->header.n_obs > 0) {
      total_messages = 1 + ((sbp_obs->header.n_obs - 1) / SBP_MAX_NUM_OBS);
    } else {
      total_messages = 1;
    }

    u8 sbp_obs_to_send_buffer[256];
    msg_obs_t *sbp_obs_to_send = (msg_obs_t *)sbp_obs_to_send_buffer;
    sbp_obs_to_send->header.t = sbp_obs->header.t;

    u8 obs_index;
    for (u8 msg_num = 0; msg_num < total_messages; msg_num++) {
      sbp_obs_to_send->header.n_obs = (total_messages << 4) + msg_num;
      for (obs_index = 0;
           obs_index < SBP_MAX_NUM_OBS &&
           (obs_index + msg_num * SBP_MAX_NUM_OBS) < sbp_obs->header.n_obs;
           obs_index++) {
        sbp_obs_to_send->obs[obs_index] =
            sbp_obs->obs[obs_index + msg_num * SBP_MAX_NUM_OBS];
      }

      u32 total_msg_size = sizeof(observation_header_t) +
                           sizeof(packed_obs_content_t) * obs_index;
      assert(total_msg_size <= UINT8_MAX);

      state->cb_ubx_to_sbp(SBP_MSG_OBS,
                           (u8)total_msg_size,
                           (u8 *)sbp_obs_to_send,
                           state->sender_id,
                           state->context);
    }
  }
}

static void handle_rxm_sfrbx(struct ubx_sbp_state *state, u8 *buf, int sz) {
  (void)sz;
  assert(state);
  assert(buf);
  assert(sz > 0);

  ubx_rxm_sfrbx sfrbx;
  if (ubx_decode_rxm_sfrbx(buf, &sfrbx) != RC_OK) {
    return;
  }

  u8 prn = sfrbx.sat_id;
  if (UBX_GNSS_ID_GPS == sfrbx.gnss_id) {
    gps_decode_subframe(state, prn, sfrbx.data_words, sfrbx.num_words);
  } else if (UBX_GNSS_ID_BDS == sfrbx.gnss_id) {
    bds_decode_subframe(state, prn, sfrbx.data_words, sfrbx.num_words);
  } else if (UBX_GNSS_ID_GAL == sfrbx.gnss_id) {
    gal_decode_page(state, prn, sfrbx.data_words, sfrbx.num_words);
  }
}

void ubx_handle_frame(u8 *frame, struct ubx_sbp_state *state) {
  u8 class_id = frame[0];
  u8 msg_id = frame[1];

  switch (class_id) {
    case UBX_CLASS_ESF:
      switch (msg_id) {
        case UBX_MSG_ESF_RAW:
          handle_esf_raw(state, frame);
          break;
        case UBX_MSG_ESF_MEAS:
          handle_esf_meas(state, frame);
          break;
        default:
          break;
      }
      break;

    case UBX_CLASS_HNR:
      if (msg_id == UBX_MSG_HNR_PVT) {
        handle_hnr_pvt(state, frame);
      }
      break;

    case UBX_CLASS_NAV:
      switch (msg_id) {
        case UBX_MSG_NAV_ATT:
          handle_nav_att(state, frame);
          break;
        case UBX_MSG_NAV_PVT:
          handle_nav_pvt(state, frame);
          break;
        case UBX_MSG_NAV_VELECEF:
          handle_nav_velecef(state, frame);
          break;
        case UBX_MSG_NAV_STATUS:
          handle_nav_status(state, frame);
          break;
        default:
          break;
      }
      break;

    case UBX_CLASS_RXM:
      if (msg_id == UBX_MSG_RXM_RAWX) {
        u8 sbp_obs_buffer[sizeof(msg_obs_t) +
                          sizeof(packed_obs_content_t) * UBX_MAX_NUM_OBS];
        handle_rxm_rawx(state, frame, sbp_obs_buffer);
      } else if (msg_id == UBX_MSG_RXM_SFRBX) {
        handle_rxm_sfrbx(state, frame, UBX_FRAME_SIZE);
      }
      break;

    default:
      break;
  }
}

void ubx_sbp_init(struct ubx_sbp_state *state,
                  void (*cb_ubx_to_sbp)(u16 msg_id,
                                        u8 length,
                                        u8 *buff,
                                        u16 sender_id,
                                        void *context),
                  void *context) {
  memset(state, 0, sizeof(*state));
  state->index = 0;
  state->bytes_in_buffer = 0;
  state->sender_id = DEFAULT_UBX_SENDER_ID;
  state->cb_ubx_to_sbp = cb_ubx_to_sbp;
  state->context = context;
  state->use_hnr = false;
}

void ubx_set_sender_id(struct ubx_sbp_state *state, u16 sender_id) {
  state->sender_id = sender_id;
}

void ubx_set_hnr_flag(struct ubx_sbp_state *state, bool use_hnr) {
  state->use_hnr = use_hnr;
}

/**
 * Processes a stream of data, converting UBX messages into SBP and outputting
 * via the callback functions in the `state` parameter. This function assumes
 * that the `ubx_sbp_state` object has already been fully populated.
 * `read_stream_func` will be called repeatedly until it returns a zero or
 * negative value.
 *
 * @param state An already populated state object
 * @param read_stream_func The function to call to read from the stream. `buf`
 * is the buffer to write data into, `len` is the size of the buffer in bytes,
 * `ctx` is the context pointer from the `state` argument, the return value is
 * the number of bytes written into `buf` with negative values indicating an
 * error.
 * @return the last value of read_stream_func, either 0 or a negative value
 * indicating an error.
 */
int ubx_sbp_process(struct ubx_sbp_state *state,
                    int (*read_stream_func)(u8 *buff, size_t len, void *ctx)) {
  state->read_stream_func = read_stream_func;
  u8 inbuf[UBX_FRAME_SIZE];

  ssize_t ret = read_ubx_frame(inbuf, state);
  if (ret <= 0) {
    return ret;
  }

  ubx_handle_frame(inbuf, state);

  return ret;
}
