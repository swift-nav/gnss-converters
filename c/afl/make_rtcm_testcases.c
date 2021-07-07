#include <libsbp/legacy/logging.h>
#include <rtcm3/encode.h>
#include <rtcm3/eph_encode.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/edc.h>
#include "make_afl_testcases.h"

#define RTCM3_PREAMBLE 0xD3

static uint8_t output_buf[4096];
static uint16_t output_buf_len = 0;

static void reset_output_buf(void) {
  memset(output_buf, 0, sizeof(output_buf));
  output_buf_len = 0;
}

static void add_framing(uint8_t *frame, uint16_t payload_len) {
  uint16_t bit = 0;
  setbitu(frame, bit, 8, RTCM3_PREAMBLE);
  bit += 8;
  setbitu(frame, bit, 6, 0);
  bit += 6;
  setbitu(frame, bit, 10, payload_len);
  bit += 10;
  bit += payload_len * 8;
  uint32_t crc = crc24q(frame, 3 + payload_len, 0);
  setbitu(frame, bit, 24, crc);
}

static void add_obs_to_output(const rtcm_obs_message *msg,
                              uint16_t (*encode_fn)(const rtcm_obs_message *,
                                                    uint8_t *)) {
  uint16_t payload_len = encode_fn(msg, output_buf + output_buf_len + 3);
  add_framing(output_buf + output_buf_len, payload_len);
  output_buf_len += payload_len + 6;
}

static void add_eph_to_output(const rtcm_msg_eph *msg,
                              uint16_t (*encode_fn)(const rtcm_msg_eph *,
                                                    uint8_t *)) {
  uint16_t payload_len = encode_fn(msg, output_buf + output_buf_len + 3);
  add_framing(output_buf + output_buf_len, payload_len);
  output_buf_len += payload_len + 6;
}

static void add_payload_to_output(const uint8_t *payload,
                                  uint16_t payload_len) {
  memcpy(output_buf + output_buf_len + 3, payload, payload_len);
  add_framing(output_buf + output_buf_len, payload_len);
  output_buf_len += payload_len + 6;
}

static void add_msm_to_output(const rtcm_msm_message *msg,
                              uint16_t (*encode_fn)(const rtcm_msm_message *,
                                                    uint8_t *)) {
  uint16_t payload_len = encode_fn(msg, output_buf + output_buf_len + 3);
  add_framing(output_buf + output_buf_len, payload_len);
  output_buf_len += payload_len + 6;
}

static void write_output(const char *name) {
  write_file(name, output_buf, output_buf_len);
  reset_output_buf();
}

static void make_1002_single_gps(void) {
  reset_output_buf();

  rtcm_obs_message msg;
  msg.header.msg_num = 1002;
  msg.header.div_free = 1;
  msg.header.n_sat = 1;
  msg.header.smooth = 5;
  msg.header.stn_id = 3315;
  msg.header.sync = 0;
  msg.header.tow_ms = 75000000;
  msg.sats[0].svId = 22;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].pseudorange = 21101051.619999997;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 110886758.14176261;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].cnr = 50.75;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;

  add_obs_to_output(&msg, rtcm3_encode_1002);
  write_output("1002_single.rtcm");
}

static void make_1002_multi_gps(void) {
  reset_output_buf();

  rtcm_obs_message msg;
  msg.header.msg_num = 1002;
  msg.header.div_free = 1;
  msg.header.n_sat = 1;
  msg.header.smooth = 5;
  msg.header.stn_id = 3315;
  msg.header.sync = 1;
  msg.header.tow_ms = 75000000;
  msg.sats[0].svId = 22;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].pseudorange = 21101051.619999997;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 110886758.14176261;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].cnr = 50.75;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1002);

  msg.header.sync = 0;
  msg.sats[0].svId = 26;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 117085181.63191102;
  msg.sats[0].obs[L1_FREQ].cnr = 39.25;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].pseudorange = 22280574.191999998;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  msg.sats[0].obs[L2_FREQ].code = 0;
  msg.sats[0].obs[L2_FREQ].carrier_phase = 91235228.132184699;
  msg.sats[0].obs[L2_FREQ].cnr = 35.25;
  msg.sats[0].obs[L2_FREQ].lock = 256;
  msg.sats[0].obs[L2_FREQ].pseudorange = 22280571.791999999;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1002);

  write_output("1002_multiple.rtcm");
}

static void make_1004_single_gps(void) {
  reset_output_buf();

  rtcm_obs_message msg;
  msg.header.msg_num = 1004;
  msg.header.div_free = 1;
  msg.header.n_sat = 1;
  msg.header.smooth = 5;
  msg.header.stn_id = 3315;
  msg.header.sync = 0;
  msg.header.tow_ms = 75000000;
  msg.sats[0].svId = 22;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].pseudorange = 21101051.619999997;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 110886758.14176261;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].cnr = 50.75;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1004);

  write_output("1004_single.rtcm");
}

static void make_1004_multi_gps(void) {
  reset_output_buf();

  rtcm_obs_message msg;
  msg.header.msg_num = 1004;
  msg.header.div_free = 1;
  msg.header.n_sat = 1;
  msg.header.smooth = 5;
  msg.header.stn_id = 3315;
  msg.header.sync = 1;
  msg.header.tow_ms = 75000000;
  msg.sats[0].svId = 22;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].pseudorange = 21101051.619999997;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 110886758.14176261;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].cnr = 50.75;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1004);

  msg.header.sync = 0;
  msg.sats[0].svId = 26;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 117085181.63191102;
  msg.sats[0].obs[L1_FREQ].cnr = 39.25;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].pseudorange = 22280574.191999998;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  msg.sats[0].obs[L2_FREQ].code = 0;
  msg.sats[0].obs[L2_FREQ].carrier_phase = 91235228.132184699;
  msg.sats[0].obs[L2_FREQ].cnr = 35.25;
  msg.sats[0].obs[L2_FREQ].lock = 256;
  msg.sats[0].obs[L2_FREQ].pseudorange = 22280571.791999999;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1004);

  write_output("1004_multiple.rtcm");
}

static void make_1005(void) {
  reset_output_buf();

  rtcm_msg_1005 msg;
  msg.stn_id = 2315;
  msg.ITRF = 0;
  msg.GPS_ind = 1;
  msg.GAL_ind = 0;
  msg.GLO_ind = 1;
  msg.ref_stn_ind = 0;
  msg.arp_x = -2694217.8790000002;
  msg.arp_y = -4264079.2110000001;
  msg.arp_z = 3890647.0312000001;
  msg.quart_cycle_ind = 0;
  msg.osc_ind = 1;
  msg.reserved = 0;

  uint8_t payload[4096];
  uint16_t payload_len = rtcm3_encode_1005(&msg, payload);
  add_payload_to_output(payload, payload_len);
  write_output("1005.rtcm");
}

static void make_1006(void) {
  reset_output_buf();

  rtcm_msg_1006 msg;
  msg.msg_1005.stn_id = 2315;
  msg.msg_1005.ITRF = 0;
  msg.msg_1005.GPS_ind = 1;
  msg.msg_1005.GAL_ind = 0;
  msg.msg_1005.GLO_ind = 1;
  msg.msg_1005.ref_stn_ind = 0;
  msg.msg_1005.arp_x = -2694217.8790000002;
  msg.msg_1005.arp_y = -4264079.2110000001;
  msg.msg_1005.arp_z = 3890647.0312000001;
  msg.msg_1005.quart_cycle_ind = 0;
  msg.msg_1005.osc_ind = 1;
  msg.msg_1005.reserved = 0;
  msg.ant_height = 10;

  uint8_t payload[4096];
  uint16_t payload_len = rtcm3_encode_1006(&msg, payload);
  add_payload_to_output(payload, payload_len);
  write_output("1006.rtcm");
}

static rtcm_msg_eph make_gps_eph(void) {
  rtcm_msg_eph msg;
  msg.constellation = RTCM_CONSTELLATION_GPS;
  msg.sat_id = 30;
  msg.wn = 2161;
  msg.toe = 75000;
  msg.fit_interval = 0;
  msg.health_bits = 0;
  msg.ura = 0;
  msg.data.kepler.tgd.gps_s = 8;
  msg.data.kepler.crc = 5525;
  msg.data.kepler.crs = 1079;
  msg.data.kepler.cuc = 984;
  msg.data.kepler.cus = 5421;
  msg.data.kepler.cic = -19;
  msg.data.kepler.cis = 65;
  msg.data.kepler.dn = 13228;
  msg.data.kepler.m0 = 590289140;
  msg.data.kepler.ecc = 44660822;
  msg.data.kepler.sqrta = 2702028478;
  msg.data.kepler.omega0 = -20899457;
  msg.data.kepler.omegadot = -22352;
  msg.data.kepler.w = -1906571568;
  msg.data.kepler.inc = 640744167;
  msg.data.kepler.inc_dot = -1204;
  msg.data.kepler.af0 = -914028;
  msg.data.kepler.af1 = -44;
  msg.data.kepler.af2 = 0;
  msg.data.kepler.toc = 26099;
  msg.data.kepler.iodc = 6;
  msg.data.kepler.iode = 6;
  msg.data.kepler.codeL2 = 1;
  msg.data.kepler.L2_data_bit = 1;
  return msg;
}

static void make_1010_single(void) {
  reset_output_buf();

  // Add a GPS ephemeris message so that leap seconds get set properly
  rtcm_msg_eph eph = make_gps_eph();
  add_eph_to_output(&eph, rtcm3_encode_gps_eph);

  rtcm_obs_message msg;
  msg.header.msg_num = 1010;
  msg.header.n_sat = 1;
  msg.header.div_free = 1;
  msg.header.stn_id = 3315;
  msg.header.sync = 0;
  msg.header.smooth = 5;
  msg.header.tow_ms = 85801000;
  msg.sats[0].svId = 12;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].pseudorange = 21101051.619999997;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 110886758.14176261;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].cnr = 50.75;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1010);

  write_output("1010_single.rtcm");
}

static void make_1010_multi(void) {
  reset_output_buf();

  // Add a GPS ephemeris message so that leap seconds get set properly
  rtcm_msg_eph eph = make_gps_eph();
  add_eph_to_output(&eph, rtcm3_encode_gps_eph);

  rtcm_obs_message msg;
  msg.header.msg_num = 1010;
  msg.header.div_free = 1;
  msg.header.n_sat = 1;
  msg.header.smooth = 5;
  msg.header.stn_id = 3315;
  msg.header.sync = 1;
  msg.header.tow_ms = 85801000;
  msg.sats[0].svId = 22;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].pseudorange = 21101051.619999997;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 110886758.14176261;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].cnr = 50.75;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1010);

  msg.header.sync = 0;
  msg.sats[0].svId = 26;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 117085181.63191102;
  msg.sats[0].obs[L1_FREQ].cnr = 39.25;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].pseudorange = 22280574.191999998;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  msg.sats[0].obs[L2_FREQ].code = 0;
  msg.sats[0].obs[L2_FREQ].carrier_phase = 91235228.132184699;
  msg.sats[0].obs[L2_FREQ].cnr = 35.25;
  msg.sats[0].obs[L2_FREQ].lock = 256;
  msg.sats[0].obs[L2_FREQ].pseudorange = 22280571.791999999;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1010);

  write_output("1010_multiple.rtcm");
}

static void make_1012_single(void) {
  reset_output_buf();

  // Add a GPS ephemeris message so that leap seconds get set properly
  rtcm_msg_eph eph = make_gps_eph();
  add_eph_to_output(&eph, rtcm3_encode_gps_eph);

  rtcm_obs_message msg;
  msg.header.msg_num = 1012;
  msg.header.n_sat = 1;
  msg.header.div_free = 1;
  msg.header.stn_id = 3315;
  msg.header.sync = 0;
  msg.header.smooth = 5;
  msg.header.tow_ms = 85801000;
  msg.sats[0].svId = 12;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].pseudorange = 21101051.619999997;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 110886758.14176261;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].cnr = 50.75;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1012);

  write_output("1012_single.rtcm");
}

static void make_1012_multi(void) {
  reset_output_buf();

  // Add a GPS ephemeris message so that leap seconds get set properly
  rtcm_msg_eph eph = make_gps_eph();
  add_eph_to_output(&eph, rtcm3_encode_gps_eph);

  rtcm_obs_message msg;
  msg.header.msg_num = 1012;
  msg.header.div_free = 1;
  msg.header.n_sat = 1;
  msg.header.smooth = 5;
  msg.header.stn_id = 3315;
  msg.header.sync = 1;
  msg.header.tow_ms = 85801000;
  msg.sats[0].svId = 22;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].pseudorange = 21101051.619999997;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 110886758.14176261;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].cnr = 50.75;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1012);

  msg.header.sync = 0;
  msg.sats[0].svId = 26;
  msg.sats[0].obs[L1_FREQ].code = 0;
  msg.sats[0].obs[L1_FREQ].carrier_phase = 117085181.63191102;
  msg.sats[0].obs[L1_FREQ].cnr = 39.25;
  msg.sats[0].obs[L1_FREQ].lock = 520;
  msg.sats[0].obs[L1_FREQ].pseudorange = 22280574.191999998;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L1_FREQ].flags.fields.valid_cnr = true;
  msg.sats[0].obs[L2_FREQ].code = 0;
  msg.sats[0].obs[L2_FREQ].carrier_phase = 91235228.132184699;
  msg.sats[0].obs[L2_FREQ].cnr = 35.25;
  msg.sats[0].obs[L2_FREQ].lock = 256;
  msg.sats[0].obs[L2_FREQ].pseudorange = 22280571.791999999;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_lock = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_pr = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_cp = true;
  msg.sats[0].obs[L2_FREQ].flags.fields.valid_cnr = true;
  add_obs_to_output(&msg, rtcm3_encode_1012);

  write_output("1012_multiple.rtcm");
}

rtcm_msm_message make_gps_msm(void) {
  rtcm_msm_message msg;
  memset(&msg, 0, sizeof(msg));
  msg.header.div_free = 0;
  msg.header.smooth = 0;
  msg.header.stn_id = 1;
  msg.header.tow_ms = 75000000;
  msg.header.multiple = 1;
  msg.header.iods = 0;
  msg.header.reserved = 0;
  msg.header.steering = 0;
  msg.header.ext_clock = 0;
  msg.header.satellite_mask[0] = true;
  msg.header.signal_mask[1] = true;
  msg.header.cell_mask[0] = true;
  msg.sats[0].glo_fcn = 0;
  msg.sats[0].rough_range_ms = 69.8759765625;
  msg.sats[0].rough_range_rate_m_s = -197;
  msg.signals[0].pseudorange_ms = 69.876379070803523;
  msg.signals[0].carrier_phase_ms = 69.876377786044031;
  msg.signals[0].lock_time_s = 24.576000000000001;
  msg.signals[0].hca_indicator = false;
  msg.signals[0].cnr = 47.625;
  msg.signals[0].range_rate_m_s = -196.9864;
  msg.signals[0].flags.fields.valid_pr = 1;
  msg.signals[0].flags.fields.valid_cp = 1;
  msg.signals[0].flags.fields.valid_cnr = 1;
  msg.signals[0].flags.fields.valid_lock = 1;
  msg.signals[0].flags.fields.valid_dop = 1;

  return msg;
}

rtcm_msm_message make_glo_msm(void) {
  rtcm_msm_message msg;
  memset(&msg, 0, sizeof(msg));
  msg.header.stn_id = 1;
  msg.header.tow_ms = 85782000;
  msg.header.multiple = 1;
  msg.header.iods = 0;
  msg.header.reserved = 0;
  msg.header.steering = 0;
  msg.header.ext_clock = 0;
  msg.header.div_free = 0;
  msg.header.smooth = 0;
  msg.header.satellite_mask[5] = true;
  msg.header.satellite_mask[8] = true;
  msg.header.signal_mask[1] = true;
  msg.header.cell_mask[0] = true;
  msg.header.cell_mask[1] = true;
  msg.sats[0].glo_fcn = 5;
  msg.sats[0].rough_range_ms = 65.7998046875;
  msg.sats[0].rough_range_rate_m_s = -384;
  msg.sats[1].glo_fcn = 0;
  msg.sats[1].rough_range_ms = 77.978515625;
  msg.sats[1].rough_range_rate_m_s = -761;
  msg.signals[0].pseudorange_ms = 65.799341047182679;
  msg.signals[0].carrier_phase_ms = 65.799342214129865;
  msg.signals[0].lock_time_s = 116.736;
  msg.signals[0].hca_indicator = false;
  msg.signals[0].cnr = 46.75;
  msg.signals[0].flags.fields.valid_pr = 1;
  msg.signals[0].flags.fields.valid_cp = 1;
  msg.signals[0].flags.fields.valid_cnr = 1;
  msg.signals[0].flags.fields.valid_lock = 1;
  msg.signals[0].flags.fields.valid_dop = 1;
  msg.signals[0].range_rate_m_s = -383.78989999999999;
  msg.signals[1].pseudorange_ms = 77.978388469666243;
  msg.signals[1].carrier_phase_ms = 0;
  msg.signals[1].lock_time_s = 0;
  msg.signals[1].hca_indicator = true;
  msg.signals[1].cnr = 29.4375;
  msg.signals[1].flags.fields.valid_pr = 1;
  msg.signals[1].flags.fields.valid_cp = 0;
  msg.signals[1].flags.fields.valid_cnr = 1;
  msg.signals[1].flags.fields.valid_lock = 1;
  msg.signals[1].flags.fields.valid_dop = 1;
  msg.signals[1].range_rate_m_s = -760.77650000000006;
  return msg;
}

rtcm_msm_message make_gal_msm(void) {
  rtcm_msm_message msg;
  memset(&msg, 0, sizeof(msg));

  msg.header.stn_id = 1;
  msg.header.tow_ms = 75000000;
  msg.header.multiple = 1;
  msg.header.iods = 0;
  msg.header.reserved = 0;
  msg.header.steering = 0;
  msg.header.ext_clock = 0;
  msg.header.div_free = 0;
  msg.header.smooth = 0;
  msg.header.satellite_mask[1] = true;
  msg.header.satellite_mask[5] = true;
  msg.header.signal_mask[4] = true;
  msg.header.signal_mask[22] = true;
  msg.header.cell_mask[0] = false;
  msg.header.cell_mask[1] = true;
  msg.header.cell_mask[2] = true;
  msg.header.cell_mask[3] = true;

  msg.sats[0].glo_fcn = 0;
  msg.sats[0].rough_range_ms = 91.5947265625;
  msg.sats[0].rough_range_rate_m_s = -494;
  msg.sats[1].glo_fcn = 0;
  msg.sats[1].rough_range_ms = 72.0390625;
  msg.sats[1].rough_range_rate_m_s = -165;
  msg.signals[0].pseudorange_ms = 91.595161352306604;
  msg.signals[0].carrier_phase_ms = 91.59517831588164;
  msg.signals[0].lock_time_s = 11.263999999999999;
  msg.signals[0].hca_indicator = false;
  msg.signals[0].cnr = 31.875;
  msg.signals[0].flags.fields.valid_pr = 1;
  msg.signals[0].flags.fields.valid_cp = 1;
  msg.signals[0].flags.fields.valid_cnr = 1;
  msg.signals[0].flags.fields.valid_lock = 1;
  msg.signals[0].flags.fields.valid_dop = 1;
  msg.signals[0].range_rate_m_s = -494.21469999999999;
  msg.signals[1].pseudorange_ms = 72.038599073886871;
  msg.signals[1].carrier_phase_ms = 72.038600483443588;
  msg.signals[1].lock_time_s = 32.768000000000001;
  msg.signals[1].hca_indicator = false;
  msg.signals[1].cnr = 40.8125;
  msg.signals[1].flags.fields.valid_pr = 1;
  msg.signals[1].flags.fields.valid_cp = 1;
  msg.signals[1].flags.fields.valid_cnr = 1;
  msg.signals[1].flags.fields.valid_lock = 1;
  msg.signals[1].flags.fields.valid_dop = 1;
  msg.signals[1].range_rate_m_s = -164.9059;
  msg.signals[2].pseudorange_ms = 72.03858906775713;
  msg.signals[2].carrier_phase_ms = 72.038590068928897;
  msg.signals[2].lock_time_s = 33.792000000000002;
  msg.signals[2].hca_indicator = false;
  msg.signals[2].cnr = 43.4375;
  msg.signals[2].flags.fields.valid_pr = 1;
  msg.signals[2].flags.fields.valid_cp = 1;
  msg.signals[2].flags.fields.valid_cnr = 1;
  msg.signals[2].flags.fields.valid_lock = 1;
  msg.signals[2].flags.fields.valid_dop = 1;
  msg.signals[2].range_rate_m_s = -164.86959999999999;

  return msg;
}

rtcm_msm_message make_bds_msm(void) {
  rtcm_msm_message msg;
  memset(&msg, 0, sizeof(msg));

  msg.header.stn_id = 1;
  msg.header.tow_ms = 74986000;
  msg.header.multiple = 0;
  msg.header.iods = 0;
  msg.header.reserved = 0;
  msg.header.steering = 0;
  msg.header.ext_clock = 0;
  msg.header.div_free = 0;
  msg.header.smooth = 0;
  msg.header.satellite_mask[6] = true;
  msg.header.signal_mask[1] = true;
  msg.header.cell_mask[0] = true;
  msg.sats[0].glo_fcn = 0;
  msg.sats[0].rough_range_ms = 136.111328125;
  msg.sats[0].rough_range_rate_m_s = 12;
  msg.signals[0].pseudorange_ms = 136.11147928796709;
  msg.signals[0].carrier_phase_ms = 136.11147928796709;
  msg.signals[0].lock_time_s = 0;
  msg.signals[0].hca_indicator = true;
  msg.signals[0].cnr = 24;
  msg.signals[0].flags.fields.valid_pr = 1;
  msg.signals[0].flags.fields.valid_cp = 1;
  msg.signals[0].flags.fields.valid_cnr = 1;
  msg.signals[0].flags.fields.valid_lock = 1;
  msg.signals[0].flags.fields.valid_dop = 1;
  msg.signals[0].range_rate_m_s = 12.4391;

  return msg;
}

void make_msm4(void) {
  reset_output_buf();

  // GPS ephemeris so that leap seconds gets set properly
  rtcm_msg_eph eph = make_gps_eph();
  add_eph_to_output(&eph, rtcm3_encode_gps_eph);

  rtcm_msm_message msg;
  msg = make_gps_msm();
  msg.header.msg_num = 1074;
  add_msm_to_output(&msg, rtcm3_encode_msm4);

  msg = make_glo_msm();
  msg.header.msg_num = 1084;
  add_msm_to_output(&msg, rtcm3_encode_msm4);

  msg = make_gal_msm();
  msg.header.msg_num = 1094;
  add_msm_to_output(&msg, rtcm3_encode_msm4);

  msg = make_bds_msm();
  msg.header.msg_num = 1124;
  msg.header.multiple = 0;  // End of sequence
  add_msm_to_output(&msg, rtcm3_encode_msm4);

  write_output("msm4.rtcm");
}

void make_msm5(void) {
  reset_output_buf();

  // GPS ephemeris so that leap seconds gets set properly
  rtcm_msg_eph eph = make_gps_eph();
  add_eph_to_output(&eph, rtcm3_encode_gps_eph);

  rtcm_msm_message msg;
  msg = make_gps_msm();
  msg.header.msg_num = 1075;
  add_msm_to_output(&msg, rtcm3_encode_msm5);

  msg = make_glo_msm();
  msg.header.msg_num = 1085;
  add_msm_to_output(&msg, rtcm3_encode_msm5);

  msg = make_gal_msm();
  msg.header.msg_num = 1095;
  add_msm_to_output(&msg, rtcm3_encode_msm5);

  msg = make_bds_msm();
  msg.header.msg_num = 1125;
  msg.header.multiple = 0;  // End of sequence
  add_msm_to_output(&msg, rtcm3_encode_msm5);

  write_output("msm5.rtcm");
}

void make_1029(void) {
  reset_output_buf();

  rtcm_msg_1029 msg;
  memset(&msg, 0, sizeof(msg));

  msg.mjd_num = 132;
  msg.stn_id = 2316;
  msg.unicode_chars = 3;
  msg.utc_sec_of_day = 3600;
  msg.utf8_code_units_n = 3;
  memcpy(msg.utf8_code_units, "STN", 3);

  uint8_t payload[4096];
  uint16_t payload_len = rtcm3_encode_1029(&msg, payload);
  add_payload_to_output(payload, payload_len);

  write_output("1029.rtcm");
}

void make_1033(void) {
  reset_output_buf();

  rtcm_msg_1033 msg;
  memset(&msg, 0, sizeof(msg));

  msg.stn_id = 1132;
  msg.ant_descriptor_counter = 3;
  memcpy(msg.ant_descriptor, "ANT", 3);
  msg.ant_setup_id = 32;
  msg.ant_serial_num_counter = 3;
  memcpy(msg.ant_serial_num, "123", 3);
  msg.rcv_descriptor_counter = 3;
  memcpy(msg.rcv_descriptor, "RCV", 3);
  msg.rcv_fw_version_counter = 3;
  memcpy(msg.rcv_fw_version, "1.0", 3);
  msg.rcv_serial_num_counter = 3;
  memcpy(msg.rcv_serial_num, "xxx", 3);

  uint8_t payload[4096];
  uint16_t payload_len = rtcm3_encode_1033(&msg, payload);
  add_payload_to_output(payload, payload_len);

  write_output("1033.rtcm");
}

void make_4062(void) {
  rtcm_msg_swift_proprietary msg;
  msg.msg_type = SBP_MSG_LOG;
  msg_log_t *log_msg = (msg_log_t *)msg.data;
  msg.len = sizeof(*log_msg) + 3;
  msg.sender_id = 0x1000;
  memcpy(log_msg->text, "ABC", 3);
  log_msg->level = 1;

  uint8_t payload[4096];
  uint16_t payload_len = rtcm3_encode_4062(&msg, payload);
  add_payload_to_output(payload, payload_len);

  write_output("4062.rtcm");
}

void make_rtcm_testcases(void) {
  make_1002_single_gps();
  make_1002_multi_gps();
  make_1004_single_gps();
  make_1004_multi_gps();
  make_1005();
  make_1006();
  make_1010_single();
  make_1010_multi();
  make_1012_single();
  make_1012_multi();
  make_1029();
  make_1033();
  make_4062();

  make_msm4();
  make_msm5();
}
