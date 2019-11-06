#include <math.h>
#include <stdint.h>
#include <string.h>

#include <gnss-converters/ubx_sbp.h>

#include <swiftnav/nav_meas.h>
#include <swiftnav/signal.h>

#include <ubx/decode.h>
#include <ubx/ubx_messages.h>

/* TODO(STAR-918) should probably consolidate these into central .h file */
#define SBP_OBS_LF_MULTIPLIER 256
#define SBP_OBS_DF_MULTIPLIER 256

#define UBX_SBP_LAT_LON_SCALING (1e-7)
#define UBX_SBP_HEIGHT_SCALING (1e-3)

#define UBX_NAV_PVT_DGNSS_MASK (0x02)
#define UBX_NAV_PVT_INS_MASK (0x04)

#define SBP_LLH_DGNSS_MASK (0x02)
#define SBP_LLH_SPP_MASK (0x01)
#define SBP_LLH_INS_MASK (0x08)

#define UBX_SBP_TOW_MS_SCALING (1e3)
#define UBX_SBP_TOW_NS_SCALING (1e6)

#define UBX_SBP_PSEUDORANGE_SCALING (50)

#define SBP_OBS_DOPPLER_MASK (1 << 3)
#define SBP_OBS_TRACK_STATE_MASK (0x07)

/* A single SBP message can only fit a maximum number of observations in it
 * floor((0xFF - sizeof(observation_header_t)) / sizeof(packed_obs_content_t))
 */
#define SBP_MAX_NUM_OBS 14

static void reload_ubx_buffer(struct ubx_sbp_state *state) {
  state->index = 0;
  state->bytes_in_buffer =
      state->read_stream_func(state->read_buffer, UBX_BUFFER_SIZE, NULL);
}

/** Attempts to read `length` bytes into `buf`, implemented as a memcpy from
 * state->read_buffer. If there are not enough bytes in the read_buffer, we
 * attempt to reload the read_buffer. If we still do not have sufficient bytes,
 * we assume EOF reached and return -1.
 */
static int read_ubx_bytes(u8 *buf, size_t length, struct ubx_sbp_state *state) {
  while (state->index + length > state->bytes_in_buffer) {
    if (state->bytes_in_buffer == 0) {
      /* Try again in case we perfectly aligned read */
      reload_ubx_buffer(state);
      if (state->bytes_in_buffer == 0) {
        /* EOF reached */
        return -1;
      } else {
        continue;
      }
    }

    size_t remaining_bytes = state->bytes_in_buffer - state->index;
    memcpy(buf, &state->read_buffer[state->index], remaining_bytes);
    length -= remaining_bytes;
    buf += remaining_bytes;
    reload_ubx_buffer(state);
  }

  memcpy(buf, &state->read_buffer[state->index], length);
  state->index += length;

  return 0;
}

/** UBX Frame:
 * SYNC_CHAR_1 | SYNC_CHAR_2 | CLASS | MSG_ID | 2-byte Length | Payload |
 * CHCKSUM_BYTE_1 | CHCKSUM_BYTE_2
 */
static int read_ubx_frame(u8 *frame, struct ubx_sbp_state *state) {
  int ret;
  do {
    ret = read_ubx_bytes(frame, 1, state);
    if (ret < 0) {
      return -1;
    }

    if (frame[0] != UBX_SYNC_CHAR_1) {
      continue;
    }

    while (frame[0] == UBX_SYNC_CHAR_1) {
      ret = read_ubx_bytes(frame, 1, state);
      if (ret == -1) {
        return -1;
      }
    }

    if (frame[0] != UBX_SYNC_CHAR_2) {
      continue;
    }

    ret = read_ubx_bytes(frame, 4, state);
    if (ret == -1) {
      return -1;
    }

    /* First two bytes are class and msg ID */
    u16 payload_length = frame[2] + (frame[3] << 8);
    /* Assume massive payload is due to corrupted message. 2 bytes for header, 2
     * bytes for length, 2 bytes for checksum not counted in payload_length
     */
    if (payload_length > UBX_FRAME_SIZE - 6) {
      fprintf(stderr,
              "UBX payload_length too large: %d; possible corrupted frame\n",
              payload_length);
      continue;
    }

    /* +2 for checksum bytes */
    ret = read_ubx_bytes(frame + 4, payload_length + 2, state);
    if (ret == -1) {
      return -1;
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
    return CODE_GAL_E5I;
    /* Galileo E5 bQ */
  } else if ((gnss_id == 2) && (sig_id == 6)) {
    return CODE_GAL_E5Q;
    /* BeiDou B1I D1 */
  } else if ((gnss_id == 3) && (sig_id == 0)) {
    return CODE_BDS2_B1;
    /* BeiDou B1I D2 */
  } else if ((gnss_id == 3) && (sig_id == 1)) {
    return CODE_BDS2_B1;
    /* BeiDou B2I D1 */
  } else if ((gnss_id == 3) && (sig_id == 2)) {
    return CODE_BDS2_B1;
    /* BeiDou B2I D2 */
  } else if ((gnss_id == 3) && (sig_id == 3)) {
    return CODE_BDS2_B1;
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
    msg->obs[i].lock = encode_lock_time((double)rxm_rawx.lock_time[i] / 1000.0);
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

  msg->flags = 0;
  /* if valid fix */
  if (nav_pvt.flags & UBX_NAV_PVT_DGNSS_MASK) {
    /* if differential corrections */
    if (nav_pvt.flags & UBX_NAV_PVT_DGNSS_MASK) {
      msg->flags |= SBP_LLH_DGNSS_MASK;
    } else {
      msg->flags |= SBP_LLH_SPP_MASK;
    }
    /* check for INS. */
    if (nav_pvt.flags == UBX_NAV_PVT_INS_MASK) {
      msg->flags |= SBP_LLH_INS_MASK;
    }
  }

  return 0;
}

static void handle_nav_pvt(struct ubx_sbp_state *state, u8 *inbuf) {
  msg_pos_llh_t sbp_pos_llh;
  if (fill_msg_pos_llh(inbuf, &sbp_pos_llh) == 0) {
    state->cb_ubx_to_sbp(SBP_MSG_POS_LLH,
                         sizeof(sbp_pos_llh),
                         (u8 *)&sbp_pos_llh,
                         state->sender_id,
                         state->context);
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

void ubx_handle_frame(u8 *frame, struct ubx_sbp_state *state) {
  u8 class_id = frame[0];
  u8 msg_id = frame[1];
  if (class_id == UBX_CLASS_NAV && msg_id == UBX_MSG_NAV_PVT) {
    handle_nav_pvt(state, frame);
  }

  if (class_id == UBX_CLASS_RXM && msg_id == UBX_MSG_RXM_RAWX) {
    u8 sbp_obs_buffer[sizeof(msg_obs_t) +
                      sizeof(packed_obs_content_t) * UBX_MAX_NUM_OBS];
    handle_rxm_rawx(state, frame, sbp_obs_buffer);
  }
}

void ubx_sbp_init(struct ubx_sbp_state *state,
                  void (*cb_ubx_to_sbp)(u16 msg_id,
                                        u8 length,
                                        u8 *buff,
                                        u16 sender_id,
                                        void *context),
                  void *context) {
  state->index = 0;
  state->bytes_in_buffer = 0;
  state->sender_id = DEFAULT_UBX_SENDER_ID;
  state->cb_ubx_to_sbp = cb_ubx_to_sbp;
  state->context = context;
}

void ubx_set_sender_id(struct ubx_sbp_state *state, u16 sender_id) {
  state->sender_id = sender_id;
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
