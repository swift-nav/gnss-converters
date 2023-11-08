#include <ubx/decode.h>
#include <ubx/encode.h>

#include "make_afl_testcases.h"

static u8 output_buf[4096];
static u16 output_buf_len = 0;

static void reset_output_buf(void) {
  memset(output_buf, 0, sizeof(output_buf));
  output_buf_len = 0;
}

static void add_payload_to_output(const uint8_t *payload,
                                  uint16_t payload_len) {
  uint8_t checksum[2];
  output_buf[output_buf_len] = UBX_SYNC_CHAR_1;
  output_buf_len++;
  output_buf[output_buf_len] = UBX_SYNC_CHAR_2;
  output_buf_len++;
  memcpy(output_buf + output_buf_len, payload, payload_len);
  ubx_checksum(output_buf + output_buf_len, payload_len, (uint8_t *)&checksum);
  memcpy(output_buf + output_buf_len + payload_len, checksum, 2);
  output_buf_len += payload_len + 2;
}

static void write_output(const char *name) {
  write_file(name, output_buf, output_buf_len);
  reset_output_buf();
}

static void make_rawx(void) {
  ubx_rxm_rawx msg;
  msg.class_id = UBX_CLASS_RXM;
  msg.msg_id = UBX_MSG_RXM_RAWX;
  msg.length = 16 + 2 * 32;
  msg.rcv_tow = 417935.5;
  msg.rcv_wn = 2157;
  msg.leap_second = 18;
  msg.num_meas = 2;
  msg.rec_status = 1;
  msg.version = 1;
  msg.reserved1[0] = 223;
  msg.reserved1[1] = 194;

  msg.pseudorange_m[0] = 38602010.392231829;
  msg.pseudorange_m[1] = 29334240.486271605;
  msg.carrier_phase_cycles[0] = 202854929.33582658;
  msg.carrier_phase_cycles[1] = 154152452.87808374;
  msg.doppler_hz[0] = -1121.99805F;
  msg.doppler_hz[1] = -3657.34644F;
  msg.gnss_id[0] = 1;
  msg.gnss_id[1] = 2;
  msg.sat_id[0] = 133;
  msg.sat_id[1] = 31;
  msg.sig_id[0] = 0;
  msg.sig_id[1] = 0;
  msg.freq_id[0] = 0;
  msg.freq_id[1] = 0;
  msg.lock_time[0] = 25260;
  msg.lock_time[1] = 19780;
  msg.cno_dbhz[0] = 46;
  msg.cno_dbhz[1] = 38;
  msg.pr_std_m[0] = 5;
  msg.pr_std_m[1] = 5;
  msg.cp_std_cycles[0] = 1;
  msg.cp_std_cycles[1] = 1;
  msg.doppler_std_hz[0] = 8;
  msg.doppler_std_hz[1] = 8;
  msg.track_state[0] = 15;
  msg.track_state[1] = 7;
  memset(msg.reserved2, 0, sizeof(msg.reserved2));

  uint8_t payload[4096];
  uint16_t payload_len = ubx_encode_rawx(&msg, payload);
  add_payload_to_output(payload, payload_len);
  write_output("rxm_rawx.ubx");
}

static void make_sfrbx_gps(void) {
  uint8_t payload[4096];
  uint16_t payload_len;

  ubx_rxm_sfrbx msg;
  memset(&msg, 0, sizeof(msg));

  msg.class_id = UBX_CLASS_RXM;
  msg.msg_id = UBX_MSG_RXM_SFRBX;

  // GPS
  msg.length = 48;
  msg.gnss_id = 0;
  msg.sat_id = 5;
  msg.reserved1 = 0;
  msg.freq_id = 0;
  msg.num_words = 10;
  msg.channel = 8;
  msg.version = 2;
  msg.reserved2 = 0;

  // GPS SF 0
  msg.data_words[0] = 583034635;
  msg.data_words[1] = 560261551;
  msg.data_words[2] = 108265506;
  msg.data_words[3] = 2490829057;
  msg.data_words[4] = 3073539499;
  msg.data_words[5] = 495939630;
  msg.data_words[6] = 2920741415;
  msg.data_words[7] = 89722010;
  msg.data_words[8] = 2151677386;
  msg.data_words[9] = 3200407867;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  // GPS SF 1
  msg.data_words[0] = 583034635;
  msg.data_words[1] = 560270011;
  msg.data_words[2] = 88086869;
  msg.data_words[3] = 2365805237;
  msg.data_words[4] = 3065253231;
  msg.data_words[5] = 934092;
  msg.data_words[6] = 72939796;
  msg.data_words[7] = 72755291;
  msg.data_words[8] = 61789736;
  msg.data_words[9] = 420257588;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  // GPS SF 2
  msg.data_words[0] = 583034635;
  msg.data_words[1] = 560237415;
  msg.data_words[2] = 1073131844;
  msg.data_words[3] = 579515222;
  msg.data_words[4] = 3220998554;
  msg.data_words[5] = 3139192658;
  msg.data_words[6] = 2262796500;
  msg.data_words[7] = 923568904;
  msg.data_words[8] = 1072283576;
  msg.data_words[9] = 88445712;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  write_output("rxm_sfrbx_gps.ubx");
}

static void make_sfrbx_sbas(void) {
  uint8_t payload[4096];
  uint16_t payload_len;

  // Inject a nav_pvt message to set last_tow_ms
  ubx_nav_pvt nav_pvt;
  nav_pvt.class_id = UBX_CLASS_NAV;
  nav_pvt.msg_id = UBX_MSG_NAV_PVT;
  nav_pvt.length = 92;
  nav_pvt.i_tow = 75000000;
  payload_len = ubx_encode_nav_pvt(&nav_pvt, payload);
  add_payload_to_output(payload, payload_len);

  ubx_rxm_sfrbx msg;
  memset(&msg, 0, sizeof(msg));

  msg.class_id = UBX_CLASS_RXM;
  msg.msg_id = UBX_MSG_RXM_SFRBX;

  // SBAS
  msg.length = 40;
  msg.gnss_id = 1;
  msg.sat_id = 133;
  msg.reserved1 = 0;
  msg.freq_id = 0;
  msg.num_words = 8;
  msg.channel = 19;
  msg.version = 2;
  msg.reserved2 = 0;
  msg.data_words[0] = 0x530e4018;
  msg.data_words[1] = 0x2800bfd;
  msg.data_words[2] = 0x8007f2c0;
  msg.data_words[3] = 0x8005fff;
  msg.data_words[4] = 0xfe9ffdff;
  msg.data_words[5] = 0xdffd61dd;
  msg.data_words[6] = 0xfa1ba3bb;
  msg.data_words[7] = 0xa19bc5a6;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  write_output("rxm_sfrbx_sbas.ubx");
}

static void make_sfrbx_gal(void) {
  uint8_t payload[4096];
  uint16_t payload_len;

  ubx_rxm_sfrbx msg;
  memset(&msg, 0, sizeof(msg));

  msg.class_id = UBX_CLASS_RXM;
  msg.msg_id = UBX_MSG_RXM_SFRBX;

  // GAL
  msg.length = 44;
  msg.gnss_id = 2;
  msg.sat_id = 7;
  msg.reserved1 = 1;
  msg.freq_id = 0;
  msg.num_words = 9;
  msg.channel = 0;
  msg.version = 2;
  msg.reserved2 = 0;

  // wtype 1
  msg.data_words[0] = 0x010a9aa4;
  msg.data_words[1] = 0xa1742e8d;
  msg.data_words[2] = 0x0026ba9d;
  msg.data_words[3] = 0xaa04c000;
  msg.data_words[4] = 0x94a8f606;
  msg.data_words[5] = 0xc094506a;
  msg.data_words[6] = 0xaaaa7a4f;
  msg.data_words[7] = 0xce7f4000;
  msg.data_words[8] = 0x01373279;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  // wtype 3
  msg.data_words[0] = 0x030abff0;
  msg.data_words[1] = 0x19c9d3bf;
  msg.data_words[2] = 0x4502e406;
  msg.data_words[3] = 0xdebec000;
  msg.data_words[4] = 0xbf9acffb;
  msg.data_words[5] = 0x2774df2a;
  msg.data_words[6] = 0xaaaa50fd;
  msg.data_words[7] = 0x54ff4000;
  msg.data_words[8] = 0x01373279;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  // wtype 5
  msg.data_words[0] = 0x0518e020;
  msg.data_words[1] = 0x0a1002e0;
  msg.data_words[2] = 0xc808cec8;
  msg.data_words[3] = 0x4cea8000;
  msg.data_words[4] = 0xaaaa91ad;
  msg.data_words[5] = 0xf4e6bc6a;
  msg.data_words[6] = 0xaaaa6960;
  msg.data_words[7] = 0x1cff4000;
  msg.data_words[8] = 0x01373279;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  // wtype 2
  msg.data_words[0] = 0x020aba5c;
  msg.data_words[1] = 0xcc98c9b9;
  msg.data_words[2] = 0xcd72f640;
  msg.data_words[3] = 0xeb980000;
  msg.data_words[4] = 0xbeba5c9e;
  msg.data_words[5] = 0xce9dcc2a;
  msg.data_words[6] = 0xaaaa4a0a;
  msg.data_words[7] = 0xf4ff4000;
  msg.data_words[8] = 0x01373279;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  // wtype 4
  msg.data_words[0] = 0x040a8700;
  msg.data_words[1] = 0x06000b6a;
  msg.data_words[2] = 0x93fbbd07;
  msg.data_words[3] = 0x6fff8000;
  msg.data_words[4] = 0xab004408;
  msg.data_words[5] = 0x7314b1ea;
  msg.data_words[6] = 0xaaaa44e8;
  msg.data_words[7] = 0xd17f4000;
  msg.data_words[8] = 0x01373279;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  write_output("rxm_sfrbx_gal.ubx");
}

static void make_sfrbx_bds(void) {
  uint8_t payload[4096];
  uint16_t payload_len;

  ubx_rxm_sfrbx msg;
  memset(&msg, 0, sizeof(msg));

  msg.class_id = UBX_CLASS_RXM;
  msg.msg_id = UBX_MSG_RXM_SFRBX;

  // BDS
  msg.length = 48;
  msg.gnss_id = 3;
  msg.sat_id = 25;
  msg.reserved1 = 0;
  msg.freq_id = 0;
  msg.num_words = 10;
  msg.channel = 5;
  msg.version = 2;
  msg.reserved2 = 0;

  // BDS SF 1
  msg.data_words[0] = 948966990;
  msg.data_words[1] = 193990677;
  msg.data_words[2] = 104252983;
  msg.data_words[3] = 679514171;
  msg.data_words[4] = 151916641;
  msg.data_words[5] = 1007030871;
  msg.data_words[6] = 769781518;
  msg.data_words[7] = 16897;
  msg.data_words[8] = 477937888;
  msg.data_words[9] = 55484755;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  // BDS SF 2
  msg.data_words[0] = 948971084;
  msg.data_words[1] = 195603081;
  msg.data_words[2] = 806165118;
  msg.data_words[3] = 829433333;
  msg.data_words[4] = 78118976;
  msg.data_words[5] = 1021126629;
  msg.data_words[6] = 92897313;
  msg.data_words[7] = 610797627;
  msg.data_words[8] = 122311100;
  msg.data_words[9] = 338146805;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  // BDS SF 3
  msg.data_words[0] = 948975171;
  msg.data_words[1] = 197277133;
  msg.data_words[2] = 72189287;
  msg.data_words[3] = 338165952;
  msg.data_words[4] = 29359465;
  msg.data_words[5] = 732036907;
  msg.data_words[6] = 962592864;
  msg.data_words[7] = 89672617;
  msg.data_words[8] = 436768620;
  msg.data_words[9] = 936228011;
  payload_len = ubx_encode_rxm_sfrbx(&msg, payload);
  add_payload_to_output(payload, payload_len);

  write_output("rxm_sfrbx_gal.ubx");
}

static void make_nav_sat(void) {
  ubx_nav_sat msg;
  msg.class_id = UBX_CLASS_NAV;
  msg.msg_id = UBX_MSG_NAV_SAT;
  msg.length = 8 + 2 * 12;
  msg.i_tow = 75000000;
  msg.version = 1;
  msg.num_svs = 2;
  msg.reserved1 = 0;

  msg.data[0].gnss_id = 0;
  msg.data[0].sv_id = 1;
  msg.data[0].cno = 0;
  msg.data[0].elev = 1;
  msg.data[0].azim = 113;
  msg.data[0].pr_res = 0;
  msg.data[0].flags = 4624;

  msg.data[1].gnss_id = 2;
  msg.data[1].sv_id = 21;
  msg.data[1].cno = 39;
  msg.data[1].elev = 22;
  msg.data[1].azim = 257;
  msg.data[1].pr_res = -42;
  msg.data[1].flags = 6431;

  uint8_t payload[4096];
  uint16_t payload_len = ubx_encode_nav_sat(&msg, payload);
  add_payload_to_output(payload, payload_len);

  write_output("nav_sat.ubx");
}

void make_ubx_testcases(void) {
  make_rawx();
  make_sfrbx_gps();
  make_sfrbx_sbas();
  make_sfrbx_gal();
  make_sfrbx_bds();
  make_nav_sat();
}
