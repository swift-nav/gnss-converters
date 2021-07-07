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

#include <swiftnav/logging.h>

#include "parser/novatel.h"
#include "swiftnav_conversion_helpers.h"

#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>

typedef int (*readfn_ptr)(uint8_t *, uint32_t, void *);
typedef int (*writefn_ptr)(uint8_t *, uint32_t, void *);

static writefn_ptr nov2sbp_writefn;

static void logfn(const char *msg) { log_warn("%s", msg); }

namespace Novatel {

// Arbitrarily chosen sender ID for the Novatel device.
static constexpr uint16_t kNovatelSbpSenderId = 5000;
static sbp_state_t sbp;

// used to track the last known time from various messages
// (note: use msg_gps_time_t instead of gps_time_t since gps_time_t
// stores tow as a double)
typedef struct timestamps_t {
  // time sent in last SBP_MSG_GPS_TIME message
  msg_gps_time_t gps_time;
  // time of last received RAWIMUSX_t message
  msg_gps_time_t imu_recv_time;
  // time of last sent SBP_MSG_IMU_AUX message
  msg_gps_time_t imu_aux_time;
} timestamps_t;

static timestamps_t time_tracker = {
    {0, 0, 0, 0},  // gps_time
    {0, 0, 0, 0},  // imu_recv_time
    {0, 0, 0, 0}   // imu_aux_time
};

/**
 * Given an array of mini navmeas types, send as many sbp obs messages
 * as necessary to convey all of the information.
 *
 * This function is an amalgam of code shamelessly lifted from PFWP.
 * It is all duplicated code, and not very pretty so it lives in a separate
 * file.
 */
extern "C" void send_sbp_obs_messages(uint8_t n,
                                      const mini_navigation_measurement_t *nm,
                                      const gps_time_t *gps_time,
                                      void (*send_sbp)(uint32_t,
                                                       size_t,
                                                       uint8_t *));

static void send_sbp_fn(uint32_t msg_id, size_t n_bytes, uint8_t *bytes) {
  auto ret = sbp_send_message(&sbp,
                              static_cast<uint16_t>(msg_id),
                              kNovatelSbpSenderId,
                              static_cast<uint8_t>(n_bytes),
                              bytes,
                              nov2sbp_writefn);
  assert(ret == SBP_OK);
}

/*
 * Send 'time_msg' as an SBP_MSG_GPS_TIME message if it is different
 * from the last sent message
 */
static void send_time_message(msg_gps_time_t *time_msg) {
  if (!gps_time_compare(&time_tracker.gps_time, time_msg)) {
    auto ret =
        sbp_send_message(&sbp,
                         SBP_MSG_GPS_TIME,
                         kNovatelSbpSenderId,
                         sizeof(*time_msg),
                         reinterpret_cast<uint8_t *>(time_msg),  // NOLINT
                         nov2sbp_writefn);
    assert(ret == SBP_OK);
  }

  time_tracker.gps_time = *time_msg;
}

////////////////////////////////////////////////////////////////////////////////
// Message callbacks.
////////////////////////////////////////////////////////////////////////////////
/**
 * Write as many observation messages as necessary to transmit all of the
 * observation data. The implementation here is largely informed by that in
 * PFWP/common_calc_pvt.c.
 */
static void write_sbp_range_cmp(const BinaryHeader *header, const void *data) {
  const auto *rangecmp =
      reinterpret_cast<const Message::RANGECMP_t *>(data);  // NOLINT
  assert(rangecmp->n_records <= MAX_CHANNELS);

  std::array<mini_navigation_measurement_t, MAX_CHANNELS> nm{};
  for (size_t i = 0; i < rangecmp->n_records; ++i) {
    convert_rangecmp_record_to_mini_navmeas(rangecmp->records.data() + i,
                                            nm.data() + i);
  }
  gps_time_t gps_time;
  gps_time.wn = static_cast<int16_t>(header->week);
  gps_time.tow = header->ms / 1000.0;
  send_sbp_obs_messages(static_cast<uint8_t>(rangecmp->n_records),
                        nm.data(),
                        &gps_time,
                        send_sbp_fn);
}

/**
 * Write an ephemeris GPS message. Nothing tricky here.
 */
static void write_sbp_gps_ephem(const BinaryHeader *header, const void *data) {
  (void)header;
  const auto *gpsephem =
      reinterpret_cast<const Message::GPSEPHEM_t *>(data);  // NOLINT
  msg_ephemeris_gps_t ephem_msg;
  convert_gpsephem_to_ephemeris_gps(gpsephem, &ephem_msg);
  auto ret =
      sbp_send_message(&sbp,
                       SBP_MSG_EPHEMERIS_GPS,
                       kNovatelSbpSenderId,
                       sizeof(ephem_msg),
                       reinterpret_cast<uint8_t *>(&ephem_msg),  // NOLINT
                       nov2sbp_writefn);
  assert(ret == SBP_OK);
}

/**
 * Write an LLH position message. Note that this must be preceded by a GPS time
 * message.
 */
static void write_sbp_best_pos(const BinaryHeader *header, const void *data) {
  const auto *bestpos =
      reinterpret_cast<const Message::BESTPOS_t *>(data);  // NOLINT

  msg_gps_time_t time_msg;
  convert_header_to_gps_time(header, &time_msg);
  send_time_message(&time_msg);

  msg_pos_llh_t llh_msg;
  convert_bestpos_to_pos_llh(header, bestpos, &llh_msg);
  auto ret = sbp_send_message(&sbp,
                              SBP_MSG_POS_LLH,
                              kNovatelSbpSenderId,
                              sizeof(llh_msg),
                              reinterpret_cast<uint8_t *>(&llh_msg),  // NOLINT
                              nov2sbp_writefn);
  assert(ret == SBP_OK);
}

static void write_sbp_best_vel(const BinaryHeader *header, const void *data) {
  const auto *bestvel =
      reinterpret_cast<const Message::BESTVEL_t *>(data);  // NOLINT

  msg_gps_time_t time_msg;
  convert_header_to_gps_time(header, &time_msg);
  send_time_message(&time_msg);

  msg_vel_ned_t vel_msg;
  convert_bestvel_to_vel_ned(header, bestvel, &vel_msg);
  auto ret = sbp_send_message(&sbp,
                              SBP_MSG_VEL_NED,
                              kNovatelSbpSenderId,
                              sizeof(vel_msg),
                              reinterpret_cast<uint8_t *>(&vel_msg),  // NOLINT
                              nov2sbp_writefn);
  assert(ret == SBP_OK);
}

static void write_sbp_ins_att(const BinaryHeader *header, const void *data) {
  const auto *insatt =
      reinterpret_cast<const Message::INSATT_t *>(data);  // NOLINT

  msg_orient_euler_t orient_euler_msg;
  convert_insatt_to_orient_euler(header, insatt, &orient_euler_msg);
  auto ret = sbp_send_message(
      &sbp,
      SBP_MSG_ORIENT_EULER,
      kNovatelSbpSenderId,
      sizeof(orient_euler_msg),
      reinterpret_cast<uint8_t *>(&orient_euler_msg),  // NOLINT
      nov2sbp_writefn);
  assert(ret == SBP_OK);
}

static void write_sbp_raw_imu(const BinaryHeader *header, const void *data) {
  const auto *rawimu =
      reinterpret_cast<const Message::RAWIMUSX_t *>(data);  // NOLINT

  msg_angular_rate_t angular_rate_msg;
  if (convert_rawimu_to_angular_rate(
          header, rawimu, &time_tracker.imu_recv_time, &angular_rate_msg)) {
    auto ret = sbp_send_message(
        &sbp,
        SBP_MSG_ANGULAR_RATE,
        kNovatelSbpSenderId,
        sizeof(angular_rate_msg),
        reinterpret_cast<uint8_t *>(&angular_rate_msg),  // NOLINT
        nov2sbp_writefn);
    assert(ret == SBP_OK);
  }

  msg_imu_raw_t imu_raw_msg;
  if (convert_rawimu_to_imu_raw(
          header, rawimu, &time_tracker.imu_recv_time, &imu_raw_msg)) {
    auto ret =
        sbp_send_message(&sbp,
                         SBP_MSG_IMU_RAW,
                         kNovatelSbpSenderId,
                         sizeof(imu_raw_msg),
                         reinterpret_cast<uint8_t *>(&imu_raw_msg),  // NOLINT
                         nov2sbp_writefn);
    assert(ret == SBP_OK);
  }

  msg_imu_aux_t imu_aux_msg;
  if (convert_rawimu_to_imu_aux(
          header, rawimu, &time_tracker.imu_aux_time, &imu_aux_msg)) {
    auto ret =
        sbp_send_message(&sbp,
                         SBP_MSG_IMU_AUX,
                         kNovatelSbpSenderId,
                         sizeof(imu_aux_msg),
                         reinterpret_cast<uint8_t *>(&imu_aux_msg),  // NOLINT
                         nov2sbp_writefn);
    assert(ret == SBP_OK);

    time_tracker.imu_aux_time.tow = static_cast<uint32_t>(header->ms);
    time_tracker.imu_aux_time.wn = header->week;
  }

  time_tracker.imu_recv_time.tow = static_cast<uint32_t>(header->ms);
  time_tracker.imu_recv_time.wn = header->week;
}

}  // namespace Novatel

static void help(char *arg, const char *additional_opts_help) {
  fprintf(stderr, "Usage: %s [options]%s\n", arg, additional_opts_help);
  fprintf(stderr, "  -h this message\n");
}

////////////////////////////////////////////////////////////////////////////////
// Main.
////////////////////////////////////////////////////////////////////////////////
/**
 * Read Novatel bytes from stdin, write SBP bytes to stdout.
 */
extern "C" int nov2sbp_main(int argc,
                            char **argv,
                            const char *additional_opts_help,
                            readfn_ptr readfn,
                            writefn_ptr writefn,
                            void *context) {
  using Novatel::Parser;
  using Novatel::sbp;
  using Novatel::write_sbp_best_pos;
  using Novatel::write_sbp_best_vel;
  using Novatel::write_sbp_gps_ephem;
  using Novatel::write_sbp_ins_att;
  using Novatel::write_sbp_range_cmp;
  using Novatel::write_sbp_raw_imu;

  int opt = -1;
  while ((opt = getopt(argc, argv, "h")) != -1) {
    switch (opt) {
      case 'h':
        help(argv[0], additional_opts_help);
        return 0;
      default:
        break;
    }
  }

  // Initialize static variables.
  sbp_state_init(&sbp);
  sbp_state_set_io_context(&sbp, context);

  nov2sbp_writefn = writefn;

  // Initialize Novatel parser.
  Novatel::Message::CallbackArray callbacks = {{
      {Novatel::Message::Id::GPSEPHEM, write_sbp_gps_ephem},
      {Novatel::Message::Id::BESTPOS, write_sbp_best_pos},
      {Novatel::Message::Id::BESTVEL, write_sbp_best_vel},
      {Novatel::Message::Id::RANGECMP, write_sbp_range_cmp},
      {Novatel::Message::Id::INSATT, write_sbp_ins_att},
      {Novatel::Message::Id::RAWIMUSX, write_sbp_raw_imu},
  }};
  Parser parser{readfn, logfn, callbacks, context};

  parser.process();

  return 0;
}
