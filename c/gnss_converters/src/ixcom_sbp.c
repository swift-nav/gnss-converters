#include <assert.h>
#include <gnss-converters/ixcom_sbp.h>
#include <ixcom/decode.h>
#include <libsbp/v4/imu.h>
#include <libsbp/v4/vehicle.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <swiftnav/constants.h>

#define ODO_TIMESOURCE_GPS 0x1
#define WHEELTICK_TIMESOURCE_GPS 0x1

void ixcom_sbp_init(struct ixcom_sbp_state *state,
                    void (*cb_ixcom_to_sbp)(u16 sender_id,
                                            sbp_msg_type_t msg_type,
                                            const sbp_msg_t *msg,
                                            void *context),
                    void *context) {
  memset(state, 0, sizeof(*state));
  state->index = 0;
  state->sender_id = DEFAULT_IXCOM_SENDER_ID;
  state->cb_ixcom_to_sbp = cb_ixcom_to_sbp;
  state->context = context;
  state->imu_raw_msgs_sent = 0;
}

void ixcom_set_sender_id(struct ixcom_sbp_state *state, u16 sender_id) {
  state->sender_id = sender_id;
}

static int read_ixcom_bytes(struct ixcom_sbp_state *state, size_t num_bytes) {
  assert(state->index + num_bytes < IXCOM_BUFFER_SIZE);

  int ret = state->read_stream_func(
      state->read_buffer + state->index, num_bytes, state->context);

  if (ret > 0) {
    state->index += ret;
  }

  return ret;
}

/** IXCOM Frame:
 * SYNC byte | Message ID | Frame Counter | Reserved | Message Length (2 bytes)
 * | GPS Week (2 bytes) | GPS TOW (8 bytes) | Payload (variable) | CRC16 (2
 * bytes)
 */
static int read_ixcom_frame(struct ixcom_sbp_state *state) {
  int ret;

  do {
    if (state->index == 0) {
      ret = read_ixcom_bytes(state, 1);
      if (ret <= 0) {
        break;
      }

      if (state->read_buffer[0] != XCOM_SYNC_BYTE) {
        state->index = 0;
        continue;
      }
    }

    size_t header_size = sizeof(XCOMHeader);
    if (state->index > 0 && state->index < header_size) {
      ret = read_ixcom_bytes(state, header_size - state->index);
      if (ret <= 0) {
        break;
      }
    }

    if (state->index >= header_size) {
      size_t message_length =
          state->read_buffer[4] + (state->read_buffer[5] << 8);

      ret = read_ixcom_bytes(state, message_length - state->index);
      if (ret <= 0) {
        break;
      }

      if (state->index >= message_length) {
        return message_length;
      }
    }
  } while (true);

  return ret;
}

/**
 * IXCOM protocol defines tow as number of seconds + number of microseconds, sbp
 * defines it as number of milliseconds + number of milliseconds/256
 */
void convert_ixcom_to_sbp_tow(u32 ixcom_int,
                              u32 ixcom_frac,
                              u32 *sbp_int,
                              u8 *sbp_frac) {
  *sbp_int = (ixcom_int * 1000) + (ixcom_frac / 1000);
  *sbp_frac = (ixcom_frac % 1000) * 256 / 1000;
}

void send_imu_aux(struct ixcom_sbp_state *state) {
  sbp_msg_t msg;
  /* placeholder for now */
  const u8 imu_type = 255;
  msg.imu_aux.imu_type = imu_type;

  /* not used */
  msg.imu_aux.temp = 0;

  const u8 gyro_range_125degs = 4;
  const u8 acc_range_4g = 1;
  msg.imu_aux.imu_conf = (gyro_range_125degs << 4) | acc_range_4g;
  state->cb_ixcom_to_sbp(state->sender_id, SbpMsgImuAux, &msg, state->context);
}

int16_t float_to_s16_clamped(float val) {
  int16_t ret;
  if (val < INT16_MIN) {
    ret = INT16_MIN;
  } else if (val > INT16_MAX) {
    ret = INT16_MAX;
  } else {
    ret = lroundf(val);
  }
  return ret;
}

void handle_imuraw(struct ixcom_sbp_state *state) {
  assert(state);

  XCOMmsg_IMURAW imuraw;
  if (ixcom_decode_imuraw(state->read_buffer, &imuraw) == IXCOM_RC_OK) {
    sbp_msg_t sbp_imuraw;
    /* need temp variable since sbp_imuraw is packed struct */
    u32 tow;
    u8 tow_f;
    convert_ixcom_to_sbp_tow(
        imuraw.header.gps_time_sec, imuraw.header.gps_time_usec, &tow, &tow_f);
    sbp_imuraw.imu_raw.tow = tow;
    sbp_imuraw.imu_raw.tow_f = tow_f;
    const float sbp_scale_acc_4g = (float)(4.0 * 9.80665 / 32768.0);
    const float radians_to_degrees = (float)(180.0 / M_PI);
    const float sbp_scale_125_degs = (float)(125.0 / 32768.0);
    sbp_imuraw.imu_raw.acc_x =
        float_to_s16_clamped(imuraw.acc[0] / sbp_scale_acc_4g);
    sbp_imuraw.imu_raw.acc_y =
        float_to_s16_clamped(imuraw.acc[1] / sbp_scale_acc_4g);
    sbp_imuraw.imu_raw.acc_z =
        float_to_s16_clamped(imuraw.acc[2] / sbp_scale_acc_4g);
    sbp_imuraw.imu_raw.gyr_x = float_to_s16_clamped(
        imuraw.omg[0] * radians_to_degrees / sbp_scale_125_degs);
    sbp_imuraw.imu_raw.gyr_y = float_to_s16_clamped(
        imuraw.omg[1] * radians_to_degrees / sbp_scale_125_degs);
    sbp_imuraw.imu_raw.gyr_z = float_to_s16_clamped(
        imuraw.omg[2] * radians_to_degrees / sbp_scale_125_degs);

    state->cb_ixcom_to_sbp(
        state->sender_id, SbpMsgImuRaw, &sbp_imuraw, state->context);

    if (state->imu_raw_msgs_sent % 100 == 0) {
      send_imu_aux(state);
      state->imu_raw_msgs_sent = 0;
    }
    state->imu_raw_msgs_sent++;
  }
}

/**
 * Arbitrarily deciding this is velocity source 0
 */
void handle_wheeldata(struct ixcom_sbp_state *state) {
  assert(state);

  XCOMmsg_WHEELDATA wheeldata;
  if (ixcom_decode_wheeldata(state->read_buffer, &wheeldata) == IXCOM_RC_OK) {
    sbp_msg_t sbp_wheeldata;

    sbp_wheeldata.odometry.tow = wheeldata.header.gps_time_sec * 1000 +
                                 wheeldata.header.gps_time_usec / 1000;
    sbp_wheeldata.odometry.velocity = (s32)(wheeldata.speed * 1000);
    sbp_wheeldata.odometry.flags = ODO_TIMESOURCE_GPS;

    state->cb_ixcom_to_sbp(
        state->sender_id, SbpMsgOdometry, &sbp_wheeldata, state->context);

    sbp_msg_t sbp_wheeltick;

    /* wheeltick.time in microseconds, not milliseconds */
    sbp_wheeltick.wheeltick.time =
        (uint64_t)(wheeldata.header.gps_time_sec) * 1000000 +
        wheeldata.header.gps_time_usec;
    sbp_wheeltick.wheeltick.flags = WHEELTICK_TIMESOURCE_GPS;
    sbp_wheeltick.wheeltick.source = 0;
    sbp_wheeltick.wheeltick.ticks = wheeldata.ticks;

    state->cb_ixcom_to_sbp(
        state->sender_id, SbpMsgWheeltick, &sbp_wheeltick, state->context);
  }
}

void ixcom_handle_frame(struct ixcom_sbp_state *state) {
  assert(state);
  u8 msg_type = state->read_buffer[1];

  switch (msg_type) {
    case XCOM_MSGID_IMURAW:
      handle_imuraw(state);
      break;

    case XCOM_MSGID_WHEELDATA:
      handle_wheeldata(state);
      break;

    default:
      break;
  }

  state->index = 0;
}

/**
 * Processes a stream of data, converting IXCOM messages into SBP and
 * outputting via the callback functions in the `state` parameter. This
 * function assumes that the `ixcom_sbp_state` object has already been fully
 * populated. `read_stream_func` will be called repeatedly until it returns a
 * zero or negative value.
 *
 * @param state An already populated state object
 * @param read_stream_func The function to call to read from the stream. `buf`
 * is the buffer to write data into, `len` is the size of the buffer in bytes,
 * `context` is the context pointer from the `state` argument, the return value
 * is the number of bytes written into `buf` with negative values indicating an
 * error.
 * @return the last value of read_stream_func, either 0 or a negative value
 * indicating an error.
 */
int ixcom_sbp_process(struct ixcom_sbp_state *state,
                      int (*read_stream_func)(u8 *buff,
                                              size_t len,
                                              void *context)) {
  state->read_stream_func = read_stream_func;

  ssize_t ret = read_ixcom_frame(state);
  if (ret <= 0) {
    return ret;
  }

  ixcom_handle_frame(state);

  return ret;
}
