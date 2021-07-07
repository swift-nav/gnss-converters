/*
 * Copyright (C) 2019,2020 Swift Navigation Inc.
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

#include <ubx/decode.h>
#include <ubx/encode.h>
#include <ubx/ubx_messages.h>

#include <check.h>
#include <libsbp/edc.h>
#include <libsbp/legacy/logging.h>
#include <libsbp/v4/orientation.h>
#include <libsbp/v4/sbas.h>
#include <libsbp/v4/system.h>
#include <libsbp/v4/tracking.h>
#include <libsbp/v4/vehicle.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <swiftnav/nav_meas.h>
#include <swiftnav/signal.h>

#include "config.h"

#define MAX_FILE_SIZE 1048576

static FILE *fp;
static char tmp_file_name[FILENAME_MAX] = {0};

static void tmp_file_teardown() {
  if (strlen(tmp_file_name) != 0) {
    unlink(tmp_file_name);
    tmp_file_name[0] = '\0';
  }
}

// Struct containing expected results and context information for temperature
// encoding tests. Passed around using the SBP context mechanism.
struct temperature_encoding_expectations {
  const double expected_temperature;
  int msg_index;
};

static void check_encoded_crc(uint16_t sender_id,
                              sbp_msg_type_t msg_type,
                              const sbp_msg_t *sbp_msg,
                              uint16_t expected_crc) {
  uint8_t encoded[SBP_MAX_PAYLOAD_LEN];
  uint8_t written;

  s8 ret = sbp_message_encode(
      encoded, SBP_MAX_PAYLOAD_LEN, &written, msg_type, sbp_msg);
  ck_assert(ret == SBP_OK);

  uint8_t tmpbuf[5];
  tmpbuf[0] = (uint8_t)msg_type;
  tmpbuf[1] = (uint8_t)(((uint16_t)msg_type) >> 8);
  tmpbuf[2] = (uint8_t)sender_id;
  tmpbuf[3] = (uint8_t)(sender_id >> 8);
  tmpbuf[4] = (uint8_t)written;

  u16 crc = crc16_ccitt(tmpbuf, sizeof(tmpbuf), 0);
  crc = crc16_ccitt(encoded, written, crc);
  ck_assert(crc == expected_crc);
}

static const uint16_t hnr_pvt_crc[] = {57519, 39729};
static void ubx_sbp_callback_hnr_pvt(uint16_t sender_id,
                                     sbp_msg_type_t msg_type,
                                     const sbp_msg_t *sbp_msg,
                                     void *context) {
  (void)context;
  static int msg_index = 0;
  const u16 msg_order[] = {SBP_MSG_POS_LLH, SBP_MSG_ORIENT_EULER};
  ck_assert_msg(msg_type == msg_order[msg_index],
                "Unexpected SBP message created");

  check_encoded_crc(sender_id, msg_type, sbp_msg, hnr_pvt_crc[msg_index]);
  msg_index++;
}

static const uint16_t hnr_pvt_disabled_crc[] = {41409};
static void ubx_sbp_callback_hnr_pvt_disabled(uint16_t sender_id,
                                              sbp_msg_type_t msg_type,
                                              const sbp_msg_t *sbp_msg,
                                              void *context) {
  (void)context;
  static int msg_index = 0;

  /* This test depends on nav_pvt working correctly, since the first message is
   * a nav_pvt message */
  ck_assert(msg_type == SBP_MSG_POS_LLH);

  check_encoded_crc(
      sender_id, msg_type, sbp_msg, hnr_pvt_disabled_crc[msg_index]);
  msg_index++;
}

static const uint16_t rxm_rawx_crc[] = {49772, 41438, 50010, 52301, 10854};
static void ubx_sbp_callback_rxm_rawx(uint16_t sender_id,
                                      sbp_msg_type_t msg_type,
                                      const sbp_msg_t *sbp_msg,
                                      void *context) {
  (void)context;
  static int msg_index = 0;

  ck_assert(msg_type == SBP_MSG_OBS);

  check_encoded_crc(sender_id, msg_type, sbp_msg, rxm_rawx_crc[msg_index]);
  msg_index++;
}

static const u16 rxm_sfrbx_gps_crc = 0xC040;
static void ubx_sbp_callback_rxm_sfrbx_gps(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)context;

  ck_assert(msg_type == SBP_MSG_EPHEMERIS_GPS);

  check_encoded_crc(sender_id, msg_type, sbp_msg, rxm_sfrbx_gps_crc);
}

static void ubx_sbp_callback_rxm_sfrbx_glo(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)sender_id;
  (void)context;

  if (msg_type != SBP_MSG_EPHEMERIS_GLO) {
    return;
  }

  const sbp_msg_ephemeris_glo_t *eph = &sbp_msg->ephemeris_glo;

  ck_assert(eph->common.toe.wn == 2081);
  ck_assert(eph->common.toe.tow == 251118);
  ck_assert(eph->common.sid.code == CODE_GLO_L1OF);
}

static const u16 rxm_sfrbx_bds_crc = 0x3EA4;
static void ubx_sbp_callback_rxm_sfrbx_bds(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)context;

  ck_assert(msg_type == SBP_MSG_EPHEMERIS_BDS);

  check_encoded_crc(sender_id, msg_type, sbp_msg, rxm_sfrbx_bds_crc);
}

static const u16 rxm_sfrbx_gal_crc = 0x4929;
static void ubx_sbp_callback_rxm_sfrbx_gal(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)context;

  ck_assert(msg_type == SBP_MSG_EPHEMERIS_GAL);
  check_encoded_crc(sender_id, msg_type, sbp_msg, rxm_sfrbx_gal_crc);
}

static void ubx_sbp_callback_rxm_sfrbx_sbas(uint16_t sender_id,
                                            sbp_msg_type_t msg_type,
                                            const sbp_msg_t *sbp_msg,
                                            void *context) {
  (void)sbp_msg;
  (void)sender_id;
  (void)context;

  ck_assert(msg_type == SBP_MSG_SBAS_RAW);
}

static void ubx_sbp_callback_nav_sat(uint16_t sender_id,
                                     sbp_msg_type_t msg_type,
                                     const sbp_msg_t *sbp_msg,
                                     void *context) {
  (void)context;
  (void)sender_id;
  static int msg_index = 0;
  const u16 msg_order[] = {SBP_MSG_SV_AZ_EL, SBP_MSG_MEASUREMENT_STATE};
  ck_assert_msg(msg_type == msg_order[msg_index++],
                "Unexpected SBP message created");
  if (msg_type == SBP_MSG_SV_AZ_EL) {
    const sbp_msg_sv_az_el_t *msg = &sbp_msg->sv_az_el;
    ck_assert_uint_eq(msg->azel[0].sid.sat, 0);
    ck_assert_uint_eq(msg->azel[0].sid.code, CODE_GPS_L1CA);
    ck_assert_uint_eq(msg->azel[0].az, 4);
    ck_assert_int_eq(msg->azel[0].el, -1);
    ck_assert_uint_eq(msg->azel[1].sid.sat, 1);
    ck_assert_uint_eq(msg->azel[1].sid.code, CODE_GAL_E1C);
    ck_assert_uint_eq(msg->azel[1].az, 2);
    ck_assert_int_eq(msg->azel[1].el, 1);
    ck_assert_uint_eq(msg->azel[2].sid.sat, 2);
    ck_assert_uint_eq(msg->azel[2].sid.code, CODE_BDS2_B1);
    ck_assert_uint_eq(msg->azel[2].az, 30);
    ck_assert_int_eq(msg->azel[2].el, 3);
    ck_assert_int_eq(msg->n_azel, 3);
  }
  if (msg_type == SBP_MSG_MEASUREMENT_STATE) {
    const sbp_msg_measurement_state_t *msg = &sbp_msg->measurement_state;
    ck_assert_uint_eq(msg->states[0].mesid.sat, 0);
    ck_assert_uint_eq(msg->states[0].mesid.code, CODE_GPS_L1CA);
    ck_assert_uint_eq(msg->states[0].cn0, 12);
    ck_assert_uint_eq(msg->states[1].mesid.sat, 1);
    ck_assert_uint_eq(msg->states[1].mesid.code, CODE_GAL_E1C);
    ck_assert_uint_eq(msg->states[1].cn0, 16);
    ck_assert_uint_eq(msg->states[2].mesid.sat, 2);
    ck_assert_uint_eq(msg->states[2].mesid.code, CODE_BDS2_B1);
    ck_assert_uint_eq(msg->states[2].cn0, 80);
    ck_assert_uint_eq(msg->n_states, 3);
  }
}

static void ubx_sbp_callback_nav_status(uint16_t sender_id,
                                        sbp_msg_type_t msg_type,
                                        const sbp_msg_t *sbp_msg,
                                        void *context) {
  (void)context;
  (void)sender_id;
  static int msg_index = 0;
  const u16 msg_order[] = {
      SBP_MSG_OBS, SBP_MSG_IMU_AUX, SBP_MSG_IMU_RAW, SBP_MSG_GNSS_TIME_OFFSET};
  ck_assert_msg(msg_type == msg_order[msg_index++],
                "Unexpected SBP message created");
  if (msg_type == SBP_MSG_GNSS_TIME_OFFSET) {
    const sbp_msg_gnss_time_offset_t *msg = &sbp_msg->gnss_time_offset;
    // The GPS week of the obs message was 1234, so this should also be the week
    // offset.
    ck_assert_int_eq(msg->weeks, 1234);
    // The millisecond offset should be the GPS iTOW of NAV-STATUS minus MSSS of
    // NAV-STATUS
    ck_assert_int_eq(msg->milliseconds, 5670 - 1234);
    ck_assert_int_eq(msg->microseconds, 0);
    ck_assert_int_eq(msg->flags, 0);
  }
}

static void ubx_sbp_callback_imu_timestamp(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)context;
  (void)sender_id;
  static int msg_index = 0;
  u16 msg_order[23];
  msg_order[0] = SBP_MSG_IMU_AUX;
  int idx = 1;
  for (idx = 1; idx < 23; idx++) {
    msg_order[idx] = SBP_MSG_IMU_RAW;
  }

  ck_assert_msg(msg_type == msg_order[msg_index],
                "Unexpected SBP message created");
  if (msg_type == SBP_MSG_IMU_RAW) {
    const sbp_msg_imu_raw_t *msg = &sbp_msg->imu_raw;
    const uint32_t mask = 0x3FFFFFFF;
    uint32_t timing_mode = (msg->tow & (~mask)) >> 30;
    uint32_t cpu_local_time = 1;
    ck_assert_uint_eq(timing_mode, cpu_local_time);

    if (msg_index < 13) {
      // The starting MSSS of the IMU messages is 4711 and the IMU interbal is
      // 0.01 seconds. The corrected starting time (including the 50 ms
      // correction) is thus 4661, and this should increment by 10 milliseconds
      // with every additional message received.
      ck_assert_uint_eq(msg->tow & mask,
                        (unsigned int)(4661 + (msg_index - 1) * 10));
      ck_assert_uint_eq(msg->tow_f, 0);
    } else {
      // Here, we check that the decoded TOW field wraps around at the end of a
      // week. The tolerance is set to 2 LSBs of the tow_f field.
      double tow = 1.0e-3 * (msg->tow & mask) + 1.0e-3 * msg->tow_f / 256.0;
      if (msg_index < 18) {
        ck_assert_double_eq_tol(
            tow, 604799.95 + (msg_index - 13) * 0.01, 2.0e-3 / 256.0);
      } else {
        ck_assert_double_eq_tol(
            tow, 0 + (msg_index - 18) * 0.01, 2.0e-3 / 256.0);
      }
    }
  }
  msg_index++;
}

static void ubx_sbp_callback_msss_rollover(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)context;
  (void)sender_id;
  static int msg_index = 0;
  u16 msg_order[] = {SBP_MSG_IMU_AUX, SBP_MSG_IMU_RAW, SBP_MSG_IMU_RAW};

  ck_assert_msg(msg_type == msg_order[msg_index],
                "Unexpected SBP message created");
  if (msg_type == SBP_MSG_IMU_RAW) {
    const sbp_msg_imu_raw_t *msg = &sbp_msg->imu_raw;
    const uint32_t mask = 0x3FFFFFFF;
    uint32_t timing_mode = (msg->tow & (~mask)) >> 30;
    uint32_t cpu_local_time = 1;
    ck_assert_uint_eq(timing_mode, cpu_local_time);
    double tow = 1.0e-3 * (msg->tow & mask) + 1.0e-3 * msg->tow_f / 256.0;
    // This test checks that a rollover of the MSSS field in the ubx protocol
    // will not cause discontinuities in the SBP output.
    ck_assert_double_eq_tol(
        tow, 61367.245 + (msg_index - 1) * 0.01, 2.0e-3 / 256);
  }
  msg_index++;
}

static const u16 esf_meas_crc[] = {
    0xBBD3, 0xBBD3, 0xFF50, 0x5501, 0x1182, 0xBBD3, 0x69F6, 0xF101};
static void ubx_sbp_callback_esf_meas(uint16_t sender_id,
                                      sbp_msg_type_t msg_type,
                                      const sbp_msg_t *sbp_msg,
                                      void *context) {
  (void)context;

  static int msg_index = 0;
  const u8 vel_sources[] = {0, 0, 2, 3, 1, 0, 3};

  u8 velocity_source = -1;
  s32 velocity = 0;
  if (msg_index < 6) {
    ck_assert(msg_type == SBP_MSG_WHEELTICK);
    const sbp_msg_wheeltick_t *msg = &sbp_msg->wheeltick;

    const u8 time_source = msg->flags;
    velocity_source = msg->source;
    velocity = msg->ticks;
    ck_assert_int_eq(msg->time, 25269459000);
    ck_assert_int_eq(time_source, 2);
  } else {
    ck_assert(msg_type == SBP_MSG_ODOMETRY);
    const sbp_msg_odometry_t *msg = &sbp_msg->odometry;
    const u8 vel_source_mask = 0b11000;
    const u8 time_source_mask = 0x3;
    velocity_source = (msg->flags & vel_source_mask) >> 3;
    velocity = msg->velocity;
    const u8 time_source = (msg->flags & time_source_mask);

    ck_assert_int_eq(msg->tow, 25269459);
    ck_assert_int_eq(time_source, 2);
  }

  /* Check that the velocity source is correctly set - RR = 0, RL = 1, FL = 2,
   * FR = 3, single tick = 0, speed = 3 */
  ck_assert(velocity_source == vel_sources[msg_index]);
  ck_assert_int_eq(velocity, 25677);

  check_encoded_crc(sender_id, msg_type, sbp_msg, esf_meas_crc[msg_index]);
  msg_index++;
}

static const u16 esf_raw_crc[] = {0x4214,
                                  0x7CAF,
                                  0xFDED,
                                  0x3B13,
                                  0xF366,
                                  0x1CCB,
                                  0x3B74,
                                  0xFDA5,
                                  0xB864,
                                  0x920D,
                                  0x87A8};
static void ubx_sbp_callback_esf_raw(uint16_t sender_id,
                                     sbp_msg_type_t msg_type,
                                     const sbp_msg_t *sbp_msg,
                                     void *context) {
  (void)context;
  static int msg_index = 0;

  /* Check that the first message we receive contains the IMU configuration with
     the expected IMU ranges */
  if (msg_index == 0) {
    ck_assert(msg_type == SBP_MSG_IMU_AUX);
    const sbp_msg_imu_aux_t *msg = &sbp_msg->imu_aux;
    ck_assert(msg->imu_type == 0);
    const u8 gyro_mask = 0xF0;
    const u8 acc_mask = 0x0F;
    const u8 gyro_mode = (msg->imu_conf & gyro_mask) >> 4;
    const u8 acc_mode = msg->imu_conf & acc_mask;
    /* Check that gyro range is set to 125 deg/s */
    ck_assert(gyro_mode == 4);
    /* Check that accelerometer range is set to 4 g */
    ck_assert(acc_mode == 1);
  } else {
    ck_assert(msg_type == SBP_MSG_IMU_RAW);
    const sbp_msg_imu_raw_t *msg = &sbp_msg->imu_raw;
    const u32 mask = 0x3FFFFFFF;
    ck_assert_int_ge(msg->tow & mask, 834462);
    ck_assert_int_le(msg->tow & mask, 834551);
  }

  /* Check that we won't receive more than 11 messages from this test file */
  ck_assert_int_lt(msg_index, 11);
  /* Check the CRCs */
  check_encoded_crc(sender_id, msg_type, sbp_msg, esf_raw_crc[msg_index]);
  msg_index++;
}

static const uint16_t nav_att_crc[] = {63972, 57794};
static void ubx_sbp_callback_nav_att(uint16_t sender_id,
                                     sbp_msg_type_t msg_type,
                                     const sbp_msg_t *sbp_msg,
                                     void *context) {
  (void)context;
  static int msg_index = 0;

  /* This test depends on nav_pvt working correctly, since the first message is
   * a nav_pvt message */
  if (msg_index == 0) {
    ck_assert(msg_type == SBP_MSG_POS_LLH);
  } else {
    ck_assert(msg_type == SBP_MSG_ORIENT_EULER);
  }

  check_encoded_crc(sender_id, msg_type, sbp_msg, nav_att_crc[msg_index]);
  msg_index++;
}

static const uint16_t nav_pvt_crc = 23845;
static void ubx_sbp_callback_nav_pvt(uint16_t sender_id,
                                     sbp_msg_type_t msg_type,
                                     const sbp_msg_t *sbp_msg,
                                     void *context) {
  (void)context;
  ck_assert(msg_type == SBP_MSG_POS_LLH);

  check_encoded_crc(sender_id, msg_type, sbp_msg, nav_pvt_crc);
}

static const uint16_t nav_pvt_corrupted_crc = 23845;
static void ubx_sbp_callback_nav_pvt_corrupted(uint16_t sender_id,
                                               sbp_msg_type_t msg_type,
                                               const sbp_msg_t *sbp_msg,
                                               void *context) {
  (void)context;
  static int msg_index = 0;

  /* message 2 has a malformed length field
   * message 3 is incomplete
   * Both should be dropped and not passed to this callback
   */
  ck_assert(msg_index == 0);

  ck_assert(msg_type == SBP_MSG_POS_LLH);
  check_encoded_crc(sender_id, msg_type, sbp_msg, nav_pvt_corrupted_crc);

  msg_index++;
}

static const uint16_t nav_pvt_fix_type_crc[] = {32103,
                                                44234,
                                                23845,
                                                23845,
                                                56365,
                                                32103,
                                                44234,
                                                19716,
                                                19716,
                                                52236,
                                                32103,
                                                44234,
                                                32103,
                                                32103,
                                                64623,
                                                27974};
static void ubx_sbp_callback_nav_pvt_fix_type(uint16_t sender_id,
                                              sbp_msg_type_t msg_type,
                                              const sbp_msg_t *sbp_msg,
                                              void *context) {
  (void)context;
  static int msg_index = 0;

  /* Some combinations of fix_type and flags do not make sense (i.e. no_fix and
   * gnssFix true simultaneously). If we add semantic sanity checks to ubx_sbp
   * in the future, this test will need to be updated.
   */
  ck_assert(msg_type == SBP_MSG_POS_LLH);
  check_encoded_crc(
      sender_id, msg_type, sbp_msg, nav_pvt_fix_type_crc[msg_index]);

  msg_index++;
}

static const uint16_t nav_pvt_set_sender_id_crc[] = {19827};
static void ubx_sbp_callback_nav_pvt_set_sender_id(uint16_t sender_id,
                                                   sbp_msg_type_t msg_type,
                                                   const sbp_msg_t *sbp_msg,
                                                   void *context) {
  (void)context;
  static int msg_index = 0;

  ck_assert(msg_type == SBP_MSG_POS_LLH);
  check_encoded_crc(
      sender_id, msg_type, sbp_msg, nav_pvt_set_sender_id_crc[msg_index]);

  msg_index++;
}

static const uint16_t nav_vel_ecef_crc[] = {57757, 36858};
static void ubx_sbp_callback_nav_vel_ecef(uint16_t sender_id,
                                          sbp_msg_type_t msg_type,
                                          const sbp_msg_t *sbp_msg,
                                          void *context) {
  (void)context;
  static int msg_index = 0;

  /* This test depends on nav_pvt working correctly, since the first message is
   * a nav_pvt message */
  if (msg_index == 0) {
    ck_assert(msg_type == SBP_MSG_POS_LLH);
  } else if (msg_index == 1) {
    ck_assert(msg_type == SBP_MSG_VEL_ECEF);
  } else {
    ck_assert(false && "unexpected message");
  }

  check_encoded_crc(sender_id, msg_type, sbp_msg, nav_vel_ecef_crc[msg_index]);

  msg_index++;
}

static void ubx_sbp_callback_wheeltick_sign_handling(uint16_t sender_id,
                                                     sbp_msg_type_t msg_type,
                                                     const sbp_msg_t *sbp_msg,
                                                     void *context) {
  (void)sender_id;
  (void)context;
  static int msg_index = 0;
  uint16_t expected_messages[] = {
      SBP_MSG_WHEELTICK, SBP_MSG_WHEELTICK, SBP_MSG_WHEELTICK};

  ck_assert_int_eq(msg_type, expected_messages[msg_index]);
  const sbp_msg_wheeltick_t *msg_wheeltick = &sbp_msg->wheeltick;
  if (msg_index == 0) {
    // Inserted first wheeltick message has tick count 91011
    ck_assert_uint_eq(msg_wheeltick->time, 1234000);
    ck_assert_int_eq(msg_wheeltick->ticks, 91011);
    ck_assert_int_eq(msg_wheeltick->source, 2);
    ck_assert_int_eq(msg_wheeltick->flags, 2);
  } else if (msg_index == 1) {
    // Second wheeltick message should increment tick count by 10
    ck_assert_uint_eq(msg_wheeltick->time, 1244000);
    ck_assert_int_eq(msg_wheeltick->ticks, 91021);
    ck_assert_int_eq(msg_wheeltick->source, 2);
    ck_assert_int_eq(msg_wheeltick->flags, 2);
  } else if (msg_index == 1) {
    // Third wheeltick message should decrement tick count by 20
    ck_assert_uint_eq(msg_wheeltick->time, 1254000);
    ck_assert_int_eq(msg_wheeltick->ticks, 91001);
    ck_assert_int_eq(msg_wheeltick->source, 2);
    ck_assert_int_eq(msg_wheeltick->flags, 2);
  }

  msg_index++;
}

static void ubx_sbp_callback_speed_sign_handling(uint16_t sender_id,
                                                 sbp_msg_type_t msg_type,
                                                 const sbp_msg_t *sbp_msg,
                                                 void *context) {
  (void)sender_id;
  (void)context;
  static int msg_index = 0;
  uint16_t expected_messages[] = {
      SBP_MSG_ODOMETRY, SBP_MSG_ODOMETRY, SBP_MSG_ODOMETRY};

  ck_assert_int_eq(msg_type, expected_messages[msg_index]);
  const sbp_msg_odometry_t *msg_odometry = &sbp_msg->odometry;
  if (msg_index == 0) {
    // Inserted first wheeltick message has tick count 91011
    ck_assert_uint_eq(msg_odometry->tow, 1234);
    ck_assert_int_eq(msg_odometry->flags & 0b111, 2);
    ck_assert_int_eq((msg_odometry->flags & 0b11000) >> 3, 3);
    ck_assert_int_eq(msg_odometry->velocity, 91011);
  } else if (msg_index == 1) {
    ck_assert_uint_eq(msg_odometry->tow, 1244);
    ck_assert_int_eq(msg_odometry->flags & 0b111, 2);
    ck_assert_int_eq((msg_odometry->flags & 0b11000) >> 3, 3);
    ck_assert_int_eq(msg_odometry->velocity, -1);
  } else if (msg_index == 2) {
    ck_assert_uint_eq(msg_odometry->tow, 1254);
    ck_assert_int_eq(msg_odometry->flags & 0b111, 2);
    ck_assert_int_eq((msg_odometry->flags & 0b11000) >> 3, 3);
    ck_assert_int_eq(msg_odometry->velocity, 91011);
  }

  msg_index++;
}

static void ubx_sbp_callback_test_no_conversion_invalid_calibtag(
    uint16_t sender_id,
    sbp_msg_type_t msg_type,
    const sbp_msg_t *sbp_msg,
    void *context) {
  (void)sender_id;
  (void)context;
  (void)msg_type;
  (void)sbp_msg;
#ifndef GNSS_CONVERTERS_DISABLE_CRC_VALIDATION
  ck_assert_msg(false, "No SBP message output expected for this test");
#endif
}

static void ubx_sbp_callback_imu_temperature(uint16_t sender_id,
                                             sbp_msg_type_t msg_type,
                                             const sbp_msg_t *sbp_msg,
                                             void *context) {
  (void)sender_id;
  (void)context;
  struct temperature_encoding_expectations *expectations =
      (struct temperature_encoding_expectations *)context;
  uint16_t expected_messages[] = {SBP_MSG_IMU_AUX, SBP_MSG_IMU_RAW};
  ck_assert_int_eq(msg_type, expected_messages[expectations->msg_index]);

  if (expectations->msg_index == 0) {
    const sbp_msg_imu_aux_t *msg = &sbp_msg->imu_aux;
    ck_assert_int_eq(
        msg->temp,
        (int)round((expectations->expected_temperature - 23.0) * 512.0));
  }

  expectations->msg_index++;
}

int read_file_check_ubx(uint8_t *buf, size_t len, void *ctx) {
  (void)ctx;
  return fread(buf, sizeof(uint8_t), len, fp);
}

void test_UBX(struct ubx_sbp_state *state, const char *filename) {
  fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  int ret;
  do {
    ret = ubx_sbp_process(state, &read_file_check_ubx);
  } while (ret > 0);
}

static int create_esf_raw_imu_messages(uint8_t *dest,
                                       uint32_t starting_msss,
                                       double sample_interval_seconds,
                                       int num_samples) {
  ubx_esf_raw msg_esf_raw;
  memset(&msg_esf_raw, 0, sizeof(ubx_esf_raw));

  msg_esf_raw.msss = starting_msss;
  msg_esf_raw.length = 4 + 8 * 6 * num_samples;
  msg_esf_raw.class_id = UBX_CLASS_ESF;
  msg_esf_raw.msg_id = UBX_MSG_ESF_RAW;
  int sensor_idx;
  int sample_idx = 0;
  int n_bytes;
  const double ubx_clock_tick_seconds = 39.06525e-6;
  long spacing = lround(sample_interval_seconds / ubx_clock_tick_seconds);

  uint8_t sensor_tag[] = {ESF_X_AXIS_ACCEL_SPECIFIC_FORCE,
                          ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE,
                          ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE,
                          ESF_X_AXIS_GYRO_ANG_RATE,
                          ESF_Y_AXIS_GYRO_ANG_RATE,
                          ESF_Z_AXIS_GYRO_ANG_RATE};
  for (sample_idx = 0; sample_idx < num_samples; sample_idx++) {
    for (sensor_idx = 0; sensor_idx < 6; sensor_idx++) {
      msg_esf_raw.data[sample_idx * 6 + sensor_idx] =
          (sensor_tag[sensor_idx] << 24) | 0;
      msg_esf_raw.sensor_time_tag[sample_idx * 6 + sensor_idx] =
          1234 + sample_idx * spacing;
    }
  }

  dest[0] = UBX_SYNC_CHAR_1;
  dest[1] = UBX_SYNC_CHAR_2;
  n_bytes = ubx_encode_esf_raw(&msg_esf_raw, &dest[2]);
  ubx_checksum(&dest[2], n_bytes, (u8 *)&dest[2 + n_bytes]);
  n_bytes += 4;
  return n_bytes;
}

static int create_esf_raw_imu_temp_message(uint8_t *dest,
                                           uint32_t starting_msss,
                                           uint32_t data) {
  ubx_esf_raw msg_esf_raw;
  memset(&msg_esf_raw, 0, sizeof(ubx_esf_raw));

  msg_esf_raw.msss = starting_msss;
  msg_esf_raw.length = 4 + 8;
  msg_esf_raw.class_id = UBX_CLASS_ESF;
  msg_esf_raw.msg_id = UBX_MSG_ESF_RAW;

  msg_esf_raw.data[0] = (ESF_GYRO_TEMP << 24) | data;

  dest[0] = UBX_SYNC_CHAR_1;
  dest[1] = UBX_SYNC_CHAR_2;
  int n_bytes = ubx_encode_esf_raw(&msg_esf_raw, &dest[2]);
  ubx_checksum(&dest[2], n_bytes, (u8 *)&dest[2 + n_bytes]);
  n_bytes += 4;
  return n_bytes;
}

static int create_esf_meas_messages(uint8_t *dest,
                                    uint32_t calib_time_tag,
                                    bool calib_tag_valid,
                                    uint32_t external_time_tag,
                                    uint8_t data_type,
                                    uint32_t data) {
  ubx_esf_meas msg_esf_meas;
  memset(&msg_esf_meas, 0, sizeof(msg_esf_meas));

  if (calib_tag_valid) {
    msg_esf_meas.calib_tag = calib_time_tag;
    msg_esf_meas.length = 12 + 4;
  } else {
    msg_esf_meas.length = 8 + 4;
  }

  msg_esf_meas.class_id = UBX_CLASS_ESF;
  msg_esf_meas.msg_id = UBX_MSG_ESF_MEAS;
  const uint8_t kCalibTtagValid = 0b1000;
  msg_esf_meas.flags = (1 << 11) | kCalibTtagValid;
  msg_esf_meas.data[0] = (data_type << 24) | (data & 0xFFFFFF);
  msg_esf_meas.time_tag = external_time_tag;
  int n_bytes;

  dest[0] = UBX_SYNC_CHAR_1;
  dest[1] = UBX_SYNC_CHAR_2;
  n_bytes = ubx_encode_esf_meas(&msg_esf_meas, &dest[2]);
  ubx_checksum(&dest[2], n_bytes, (u8 *)&dest[2 + n_bytes]);
  n_bytes += 4;
  return n_bytes;
}

static int create_nav_sat_message(uint8_t *dest) {
  ubx_nav_sat msg_nav_sat;
  memset(&msg_nav_sat, 0, sizeof(msg_nav_sat));

  msg_nav_sat.class_id = 0x01;
  msg_nav_sat.msg_id = 0x35;
  msg_nav_sat.length = 44;
  msg_nav_sat.i_tow = 433200;
  msg_nav_sat.version = 1;
  msg_nav_sat.num_svs = 3;
  msg_nav_sat.reserved1 = 0x0000;
  msg_nav_sat.data[0].gnss_id = 0;
  msg_nav_sat.data[0].sv_id = 0;
  msg_nav_sat.data[0].cno = 3;
  msg_nav_sat.data[0].elev = -1;
  msg_nav_sat.data[0].azim = 8;
  msg_nav_sat.data[0].pr_res = 3;
  msg_nav_sat.data[0].flags = 0x00000000;
  msg_nav_sat.data[1].gnss_id = 2;
  msg_nav_sat.data[1].sv_id = 1;
  msg_nav_sat.data[1].cno = 4;
  msg_nav_sat.data[1].elev = 1;
  msg_nav_sat.data[1].azim = 4;
  msg_nav_sat.data[1].pr_res = 5;
  msg_nav_sat.data[1].flags = 0x00000000;
  msg_nav_sat.data[2].gnss_id = 3;
  msg_nav_sat.data[2].sv_id = 2;
  msg_nav_sat.data[2].cno = 20;
  msg_nav_sat.data[2].elev = 3;
  msg_nav_sat.data[2].azim = 60;
  msg_nav_sat.data[2].pr_res = 2;
  msg_nav_sat.data[2].flags = 0x00000000;

  dest[0] = UBX_SYNC_CHAR_1;
  dest[1] = UBX_SYNC_CHAR_2;
  int n_bytes = ubx_encode_nav_sat(&msg_nav_sat, &dest[2]);
  ubx_checksum(&dest[2], n_bytes, (u8 *)&dest[2 + n_bytes]);
  n_bytes += 4;
  return n_bytes;
}

static int create_nav_status_message(uint8_t *dest,
                                     uint32_t msss,
                                     uint32_t i_tow) {
  ubx_nav_status msg_nav_status;
  msg_nav_status.class_id = UBX_CLASS_NAV;
  msg_nav_status.msg_id = UBX_MSG_NAV_STATUS;
  msg_nav_status.msss = msss;
  msg_nav_status.i_tow = i_tow;
  msg_nav_status.fix_type = 3;
  msg_nav_status.status_flags = 0xF;
  msg_nav_status.fix_status = 0;
  msg_nav_status.status_flags_ext = 0;
  msg_nav_status.ttff_ms = 567;
  msg_nav_status.length = 16;
  dest[0] = UBX_SYNC_CHAR_1;
  dest[1] = UBX_SYNC_CHAR_2;
  int n_bytes = ubx_encode_nav_status(&msg_nav_status, &dest[2]);
  ubx_checksum(&dest[2], n_bytes, (u8 *)&dest[2 + n_bytes]);
  n_bytes += 4;
  return n_bytes;
}

START_TEST(test_hnr_pvt) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_hnr_pvt, NULL);

  ubx_set_hnr_flag(&state, true);
  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/hnr_pvt.ubx");
}
END_TEST

START_TEST(test_hnr_pvt_disabled) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_hnr_pvt_disabled, NULL);

  ubx_set_hnr_flag(&state, false);
  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/hnr_pvt.ubx");
}
END_TEST

START_TEST(test_nav_att) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_att, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/nav_att.ubx");
}
END_TEST

START_TEST(test_nav_pvt) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_pvt, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/nav_pvt.ubx");

  ck_assert(state.last_tow_ms < WEEK_MS);
}
END_TEST

START_TEST(test_nav_pvt_corrupted) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_pvt_corrupted, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/nav_pvt_corrupted.ubx");
}
END_TEST

START_TEST(test_nav_pvt_fix_type) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_pvt_fix_type, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/nav_pvt_fix_type.ubx");

  ck_assert(state.last_tow_ms < WEEK_MS);
}
END_TEST

START_TEST(test_nav_pvt_set_sender_id) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_pvt_set_sender_id, NULL);

  ubx_set_sender_id(&state, 12345);
  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/nav_pvt.ubx");
}
END_TEST

START_TEST(test_nav_vel_ecef) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_vel_ecef, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/nav_velecef.ubx");
}
END_TEST

START_TEST(test_rxm_rawx) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_rxm_rawx, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/rxm_rawx.ubx");
}
END_TEST

START_TEST(test_rxm_sfrbx_gps) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_rxm_sfrbx_gps, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/rxm_sfrbx_gps.ubx");
}
END_TEST

START_TEST(test_rxm_sfrbx_glo) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_rxm_sfrbx_glo, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/rxm_sfrbx_glo.ubx");
}
END_TEST

START_TEST(test_rxm_sfrbx_bds) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_rxm_sfrbx_bds, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/rxm_sfrbx_bds.ubx");
}
END_TEST

START_TEST(test_rxm_sfrbx_gal) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_rxm_sfrbx_gal, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/rxm_sfrbx_gal.ubx");
}
END_TEST

START_TEST(test_rxm_sfrbx_sbas) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_rxm_sfrbx_sbas, NULL);
  state.last_tow_ms = 250709100;

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/rxm_sfrbx_sbas.ubx");
}
END_TEST

START_TEST(test_esf_meas) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_esf_meas, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/esf_meas.ubx");
}
END_TEST

START_TEST(test_wheeltick_sign) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_wheeltick_sign_handling, NULL);
  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];
  memset(buffer, 0, 512);

  // Insert first wheeltick message
  uint32_t positive_ticks = 91011;
  int n_bytes = create_esf_meas_messages(
      buffer, 1234, true, 5678, ESF_FRONT_LEFT_WHEEL_TICKS, positive_ticks);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Increment ticks by 10
  positive_ticks += 10;
  n_bytes = create_esf_meas_messages(
      buffer, 1244, true, 5678, ESF_FRONT_LEFT_WHEEL_TICKS, positive_ticks);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Decrement ticks by 20
  uint32_t decrement_ticks_by_20 = (1 << 23) | (positive_ticks + 20);
  n_bytes = create_esf_meas_messages(buffer,
                                     1254,
                                     true,
                                     5678,
                                     ESF_FRONT_LEFT_WHEEL_TICKS,
                                     decrement_ticks_by_20);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  fclose(fp);

  test_UBX(&state, tmp_file_name);
}
END_TEST

START_TEST(test_odometry_sign) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_speed_sign_handling, NULL);
  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];
  memset(buffer, 0, 512);

  // Insert first odomety message - positive speed
  uint32_t positive_speed = 91011;
  int n_bytes = create_esf_meas_messages(
      buffer, 1234, true, 5678, ESF_SPEED, positive_speed);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Insert second odomety message - negative speed
  uint32_t negative_speed = 0xFFFFFF;  // -1 in 24 bit two's complement
  n_bytes = create_esf_meas_messages(
      buffer, 1244, true, 5678, ESF_SPEED, negative_speed);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Insert third odomety message - positive speed
  n_bytes = create_esf_meas_messages(
      buffer, 1254, true, 5678, ESF_SPEED, positive_speed);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  fclose(fp);

  test_UBX(&state, tmp_file_name);
}
END_TEST

START_TEST(test_no_conversion_invalid_calibtag) {
  struct ubx_sbp_state state;
  ubx_sbp_init(
      &state, ubx_sbp_callback_test_no_conversion_invalid_calibtag, NULL);
  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];
  memset(buffer, 0, 512);

  // Insert first odomety message - positive speed
  uint32_t positive_speed = 91011;
  int n_bytes = create_esf_meas_messages(
      buffer, 1234, false, 5678, ESF_SPEED, positive_speed);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Insert second odomety message - negative speed
  uint32_t negative_speed = 0xFFFFFF;  // -1 in 24 bit two's complement
  n_bytes = create_esf_meas_messages(
      buffer, 1244, false, 5678, ESF_SPEED, negative_speed);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Insert third odomety message - positive speed
  n_bytes = create_esf_meas_messages(
      buffer, 1254, false, 5678, ESF_SPEED, positive_speed);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  fclose(fp);

  test_UBX(&state, tmp_file_name);
}
END_TEST

START_TEST(test_nav_sat) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_sat, NULL);

  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];
  memset(buffer, 0, 512);
  int n_bytes;

  n_bytes = create_nav_sat_message(buffer);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  fclose(fp);

  // Run tests
  test_UBX(&state, tmp_file_name);
}
END_TEST

START_TEST(test_nav_status) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_nav_status, NULL);

  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];
  memset(buffer, 0, 512);
  int n_bytes;

  n_bytes = create_nav_status_message(buffer, 1234, 5670);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Create an obs message to set week number
  ubx_rxm_rawx msg_rawx;
  memset(&msg_rawx, 0, sizeof(ubx_rxm_rawx));
  msg_rawx.rcv_wn = 1234;
  msg_rawx.rcv_tow = 567.8;
  msg_rawx.class_id = UBX_CLASS_RXM;
  msg_rawx.msg_id = UBX_MSG_RXM_RAWX;
  msg_rawx.num_meas = 1;
  msg_rawx.length = 16 + 32 * msg_rawx.num_meas;
  buffer[0] = UBX_SYNC_CHAR_1;
  buffer[1] = UBX_SYNC_CHAR_2;
  n_bytes = ubx_encode_rawx(&msg_rawx, &buffer[2]);
  ubx_checksum(&buffer[2], n_bytes, (u8 *)&buffer[2 + n_bytes]);
  n_bytes += 4;
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Create IMU messages
  n_bytes = create_esf_raw_imu_messages(buffer, 4711, 0.01, 1);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Create NAV-STATUS message
  n_bytes = create_nav_status_message(buffer, 1234, 5670);
  fwrite(buffer, n_bytes, sizeof(char), fp);
  fclose(fp);

  // Run tests
  test_UBX(&state, tmp_file_name);

  ck_assert(state.last_tow_ms < WEEK_MS);
}
END_TEST

START_TEST(test_imu_timestamps) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_imu_timestamp, NULL);

  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];

  // Create IMU messages
  int n_bytes = create_esf_raw_imu_messages(buffer, 4711, 0.01, 1);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // The next IMU message has an msss which is 100 ms bigger
  n_bytes = create_esf_raw_imu_messages(buffer, 4721, 0.01, 1);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Make a burst of IMU messages in a single ESF-RAW message
  n_bytes = create_esf_raw_imu_messages(buffer, 4731, 0.01, 10);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  // Check IMU time week rollover behavior
  n_bytes = create_esf_raw_imu_messages(buffer, 604800000, 0.01, 1);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  fclose(fp);

  // Run tests
  test_UBX(&state, tmp_file_name);
}
END_TEST

START_TEST(test_imu_timestamps_msss_rollover) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_msss_rollover, NULL);

  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];
  memset(buffer, 0, 512);

  // Create IMU messages - MSSS rollover from UINT32_MAX
  int n_bytes = create_esf_raw_imu_messages(buffer, UINT32_MAX, 0.01, 1);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  n_bytes = create_esf_raw_imu_messages(buffer, 9, 0.01, 1);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  fclose(fp);

  // Run tests
  test_UBX(&state, tmp_file_name);
}
END_TEST

START_TEST(test_esf_raw) {
  struct ubx_sbp_state state;
  ubx_sbp_init(&state, ubx_sbp_callback_esf_raw, NULL);

  test_UBX(&state, RELATIVE_PATH_PREFIX "/data/esf_raw.ubx");
}
END_TEST

START_TEST(test_convert_temperature) {
  /* Check that the conversion yields the same results that would be expected
   * for the BMI160 IMU */
  ck_assert_int_eq(ubx_convert_temperature_to_bmi160(23.0), 0);
  ck_assert_int_eq(ubx_convert_temperature_to_bmi160(23.0 + 1.0 / 512), 1);
  ck_assert_int_eq(ubx_convert_temperature_to_bmi160(23.0 - 1.0 / 512), -1);
  ck_assert_int_eq(ubx_convert_temperature_to_bmi160(87.0 - 1.0 / 512),
                   INT16_MAX);
  ck_assert_int_eq(ubx_convert_temperature_to_bmi160(-41.0 + 1.0 / 512),
                   INT16_MIN + 1);

  /* Check overflow behavior */
  ck_assert_int_eq(ubx_convert_temperature_to_bmi160(-50.0), INT16_MIN + 1);
  ck_assert_int_eq(ubx_convert_temperature_to_bmi160(100.0), INT16_MAX);
}
END_TEST

START_TEST(test_encode_negative_temperature) {
  struct ubx_sbp_state state;
  struct temperature_encoding_expectations expectations = {
      .expected_temperature = -30.0};
  ubx_sbp_init(&state, ubx_sbp_callback_imu_temperature, (void *)&expectations);

  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];
  memset(buffer, 0, 512);

  int32_t gyro_temp_centi_celsius = -3000;
  // Convert to 24 bits signed integer
  uint32_t *tmp = (uint32_t *)&gyro_temp_centi_celsius;
  *tmp &= 0xFFFFFF;
  int n_bytes = create_esf_raw_imu_temp_message(buffer, 1234, *tmp);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  n_bytes = create_esf_raw_imu_messages(buffer, 1234, 0.01, 1);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  fclose(fp);

  // Run tests
  test_UBX(&state, tmp_file_name);
}
END_TEST

START_TEST(test_encode_positive_temperature) {
  struct ubx_sbp_state state;
  struct temperature_encoding_expectations expectations = {
      .expected_temperature = 30.0};
  ubx_sbp_init(&state, ubx_sbp_callback_imu_temperature, (void *)&expectations);

  // Open a temporary file for this test
  strncpy(tmp_file_name, "XXXXXX", FILENAME_MAX);
  int fd = mkstemp(tmp_file_name);
  fp = fdopen(fd, "wb");
  uint8_t buffer[512];
  memset(buffer, 0, 512);

  int32_t gyro_temp_centi_celsius = 3000;
  // Convert to 24 bits signed integer
  uint32_t *tmp = (uint32_t *)&gyro_temp_centi_celsius;
  *tmp &= 0xFFFFFF;
  int n_bytes = create_esf_raw_imu_temp_message(buffer, 1234, *tmp);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  n_bytes = create_esf_raw_imu_messages(buffer, 1234, 0.01, 1);
  fwrite(buffer, n_bytes, sizeof(char), fp);

  fclose(fp);

  // Run tests
  test_UBX(&state, tmp_file_name);
}
END_TEST

Suite *ubx_suite(void) {
  Suite *s = suite_create("UBX");

  TCase *tc_hnr = tcase_create("UBX_HNR");
  tcase_add_test(tc_hnr, test_hnr_pvt);
  tcase_add_test(tc_hnr, test_hnr_pvt_disabled);
  suite_add_tcase(s, tc_hnr);

  TCase *tc_nav = tcase_create("UBX_NAV");
  tcase_add_test(tc_nav, test_nav_att);
  tcase_add_test(tc_nav, test_nav_pvt);
  tcase_add_test(tc_nav, test_nav_pvt_corrupted);
  tcase_add_test(tc_nav, test_nav_pvt_fix_type);
  tcase_add_test(tc_nav, test_nav_pvt_set_sender_id);
  tcase_add_test(tc_nav, test_nav_vel_ecef);
  tcase_add_test(tc_nav, test_nav_sat);
  tcase_add_test(tc_nav, test_nav_status);
  tcase_add_checked_fixture(tc_nav, NULL, tmp_file_teardown);
  suite_add_tcase(s, tc_nav);

  TCase *tc_esf = tcase_create("UBX_ESF");
  tcase_add_test(tc_esf, test_imu_timestamps);
  tcase_add_test(tc_esf, test_imu_timestamps_msss_rollover);
  tcase_add_test(tc_esf, test_esf_meas);
  tcase_add_test(tc_esf, test_esf_raw);
  tcase_add_test(tc_esf, test_convert_temperature);
  tcase_add_test(tc_esf, test_encode_negative_temperature);
  tcase_add_test(tc_esf, test_encode_positive_temperature);
  tcase_add_test(tc_esf, test_wheeltick_sign);
  tcase_add_test(tc_esf, test_odometry_sign);
  tcase_add_test(tc_esf, test_no_conversion_invalid_calibtag);
  tcase_add_checked_fixture(tc_esf, NULL, tmp_file_teardown);
  suite_add_tcase(s, tc_esf);

  TCase *tc_rxm = tcase_create("UBX_RXM");
  tcase_add_test(tc_rxm, test_rxm_rawx);
  tcase_add_test(tc_rxm, test_rxm_sfrbx_gps);
  tcase_add_test(tc_rxm, test_rxm_sfrbx_glo);
  tcase_add_test(tc_rxm, test_rxm_sfrbx_bds);
  tcase_add_test(tc_rxm, test_rxm_sfrbx_gal);
  tcase_add_test(tc_rxm, test_rxm_sfrbx_sbas);
  suite_add_tcase(s, tc_rxm);

  return s;
}

int main(void) {
  int number_failed = 0;

  Suite *s = {0};

  SRunner *sr = srunner_create(s);
  srunner_set_xml(sr, "test_results.xml");

  srunner_add_suite(sr, ubx_suite());

  srunner_set_fork_status(sr, CK_NOFORK);
  srunner_run_all(sr, CK_NORMAL);
  number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
