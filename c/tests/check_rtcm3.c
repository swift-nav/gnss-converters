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

#include <check.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <rtcm3/decode.h>
#include <rtcm3/encode.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/sid_set.h>

#include "check_rtcm3.h"
#include "check_suites.h"
#include "config.h"

gps_time_t current_time;

static double expected_L1CA_bias = 0.0;
static double expected_L1P_bias = 0.0;
static double expected_L2CA_bias = 0.0;
static double expected_L2P_bias = 0.0;

static sbp_gps_time_t previous_obs_time = {.tow = 0, .wn = INVALID_TIME};
static u8 previous_n_meas = 0;
static u8 previous_num_obs = 0;

static struct rtcm3_sbp_state state;
static struct rtcm3_out_state out_state;

static const uint32_t crc24qtab[256] = {
    0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC,
    0x9F7F17, 0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23,
    0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868,
    0xD0E493, 0xDC7D65, 0x5A319E, 0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646,
    0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7, 0x0CD1E9, 0x8A9D12, 0x8604E4,
    0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE, 0xAD50D0, 0x2B1C2B,
    0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7, 0xC99F60,
    0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
    0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5,
    0xF7614E, 0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8,
    0x00903E, 0x86DCC5, 0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A,
    0xAD88F1, 0xA11107, 0x275DFC, 0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD,
    0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C, 0x7D6C62, 0xFB2099, 0xF7B96F,
    0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375, 0x15723B, 0x933EC0,
    0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C, 0xB4F302,
    0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
    0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E,
    0x4F43A5, 0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791,
    0x688E67, 0xEEC29C, 0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145,
    0x26EDBE, 0x2A7448, 0xAC38B3, 0x92C69D, 0x148A66, 0x181390, 0x9E5F6B,
    0x01207C, 0x876C87, 0x8BF571, 0x0DB98A, 0xF6092D, 0x7045D6, 0x7CDC20,
    0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A, 0x578814, 0xD1C4EF,
    0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703, 0x3F964D,
    0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
    0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498,
    0x016863, 0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE,
    0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C,
    0x4EF3E7, 0x426A11, 0xC426EA, 0x2AE476, 0xACA88D, 0xA0317B, 0x267D80,
    0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61, 0x8B654F, 0x0D29B4, 0x01B042,
    0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58, 0xEFAAFF, 0x69E604,
    0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8, 0x4E2BC6,
    0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
    0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673,
    0xB94A88, 0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC,
    0x9E874A, 0x18CBB1, 0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7,
    0xF6D10C, 0xFA48FA, 0x7C0401, 0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9,
    0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538};

static packed_obs_content_t sbp_test_data[] = {
    {1076594107, {113150797, 178}, {2025, 90}, 200, 14, 15, {8, 0}},
    {1052792371, {110649219, 83}, {1784, 3}, 204, 14, 15, {10, 0}},
    {1211420676, {127321212, 230}, {-2004, 131}, 148, 14, 15, {13, 0}},
    {1145898161, {120434726, 100}, {-691, 105}, 184, 14, 15, {15, 0}},
    {1193292504, {125415891, 224}, {-3715, 140}, 176, 14, 15, {16, 0}},
    {1212730365, {127458823, 44}, {3918, 243}, 164, 14, 15, {18, 0}},
    {1028016946, {108045309, 73}, {-373, 29}, 196, 14, 15, {20, 0}},
    {1146192135, {120465590, 214}, {-2489, 240}, 192, 14, 15, {21, 0}},
    {1022557522, {107471517, 97}, {-276, 33}, 212, 14, 15, {27, 0}},
    {1222990444, {128537159, 17}, {-902, 76}, 148, 14, 15, {30, 0}},
    {1076594282, {88169456, 168}, {1578, 51}, 176, 14, 15, {8, 1}},
    {1052792374, {86220160, 1}, {1390, 100}, 180, 14, 15, {10, 1}},
    {1145898151, {93845224, 62}, {-539, 207}, 148, 14, 15, {15, 1}},
    {1022557545, {83744032, 221}, {-215, 37}, 188, 14, 15, {27, 1}},
    {1222990548, {100158785, 33}, {-704, 56}, 132, 13, 15, {30, 1}},
    {1065177381, {113679866, 211}, {-1361, 186}, 184, 14, 15, {2, 3}},
    {996013864, {106634862, 13}, {1314, 57}, 200, 14, 15, {3, 3}},
    {1128091581, {120817628, 56}, {4471, 1}, 176, 14, 15, {4, 3}},
    {1141411804, {121901553, 33}, {-3635, 8}, 184, 14, 15, {9, 3}},
    {1079759346, {115114554, 22}, {-540, 62}, 188, 14, 15, {10, 3}},
    {1170011510, {125043767, 130}, {3080, 218}, 160, 14, 15, {11, 3}},
    {1011426617, {107981325, 222}, {-2569, 35}, 200, 14, 15, {18, 3}},
    {970789084, {103861396, 130}, {1345, 241}, 180, 14, 15, {19, 3}},
    {1065177465, {88417685, 1}, {-1059, 235}, 160, 14, 15, {2, 4}},
    {996014042, {82938235, 192}, {1022, 59}, 180, 14, 15, {3, 4}},
    {1128091758, {93969291, 225}, {3477, 209}, 160, 11, 15, {4, 4}},
    {1141411880, {94812306, 17}, {-2827, 66}, 168, 14, 15, {9, 4}},
    {1079759506, {89533572, 17}, {-420, 93}, 136, 14, 15, {10, 4}},
    {1170011343, {97256230, 93}, {2396, 185}, 128, 8, 15, {11, 4}},
    {1011426783, {83985473, 117}, {-1998, 17}, 180, 14, 15, {18, 4}},
    {970789145, {80781068, 14}, {1046, 255}, 164, 14, 15, {19, 4}},
    {1905584379, {198457625, 111}, {310, 79}, 164, 12, 15, {6, 12}},
    {1904198770, {198313311, 251}, {1273, 149}, 168, 13, 15, {9, 12}},
    {1124221717, {117082227, 36}, {-611, 155}, 180, 13, 15, {14, 12}},
    {1905584415, {153459993, 183}, {239, 150}, 168, 12, 15, {6, 13}},
    {1904198738, {153348404, 139}, {985, 53}, 176, 13, 15, {9, 13}},
    {1124219524, {90535361, 31}, {-472, 18}, 180, 13, 15, {14, 13}},
    {1308619286, {137536820, 202}, {166, 211}, 176, 13, 15, {2, 14}},
    {1367072989, {143680335, 86}, {3236, 169}, 160, 13, 15, {4, 14}},
    {1111185587, {116786402, 20}, {1435, 244}, 180, 13, 15, {11, 14}},
    {1116077611, {117300562, 142}, {-1454, 238}, 180, 13, 15, {12, 14}},
    {1243548316, {130697813, 141}, {1221, 137}, 172, 13, 15, {19, 14}},
    {1297377968, {136355345, 162}, {-2489, 229}, 168, 13, 15, {25, 14}},
    {1308619388, {105385329, 253}, {127, 202}, 160, 13, 15, {2, 20}},
    {1367073089, {110092710, 5}, {2479, 154}, 168, 13, 15, {4, 20}},
    {1111185468, {89485640, 18}, {1100, 108}, 172, 13, 15, {11, 20}},
    {1116077555, {89879619, 94}, {-1114, 214}, 172, 13, 15, {12, 20}},
    {1243548284, {100145050, 137}, {935, 195}, 164, 13, 15, {19, 20}},
    {1297378054, {104480052, 213}, {-1907, 185}, 168, 13, 15, {25, 20}}};

static uint32_t crc24q(const uint8_t *buf, uint32_t len, uint32_t crc) {
  for (uint32_t i = 0; i < len; i++) {
    crc = ((crc << 8) & 0xFFFFFF) ^ crc24qtab[((crc >> 16) ^ buf[i]) & 0xff];
  }
  return crc;
}

void update_obs_time(const msg_obs_t *msg) {
  gps_time_t obs_time = {.tow = msg[0].header.t.tow * MS_TO_S,
                         .wn = msg[0].header.t.wn};
  rtcm2sbp_set_gps_time(&obs_time, &state);
}

void sbp_callback_gps(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  static uint32_t msg_count = 0;
  /* ignore log messages */
  if (msg_id == SBP_MSG_LOG) {
    return;
  }
  if (msg_count == 3 || msg_count == 20 || msg_count == 42) {
    ck_assert_uint_eq(msg_id, SBP_MSG_BASE_POS_ECEF);
  } else if (msg_count == 4) {
    ck_assert_uint_eq(msg_id, SBP_MSG_GLO_BIASES);
  } else {
    ck_assert_uint_eq(msg_id, SBP_MSG_OBS);
    update_obs_time((msg_obs_t *)buffer);
  }
  msg_count++;
}

void sbp_callback_gps_eph(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  static bool checked_eph = false;
  /* ignore log messages */
  if (msg_id == SBP_MSG_EPHEMERIS_GPS && !checked_eph) {
    checked_eph = true;
    msg_ephemeris_gps_t *msg = (msg_ephemeris_gps_t *)buffer;
    ck_assert(msg->common.sid.sat == 1);
    ck_assert(msg->common.sid.code == CODE_GPS_L1CA);
    ck_assert(msg->common.toe.wn == 2012);
    ck_assert(msg->common.toe.tow == 489600);
    ck_assert(fabs(msg->common.ura - 2.8) < FLOAT_EPS);
    ck_assert(msg->common.fit_interval == 14400);
    ck_assert(msg->common.valid == 1);
    ck_assert(msg->common.health_bits == 0);

    ck_assert(fabs(msg->tgd - 5.587935447692871e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rs - 10.34375) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rc - 269.15625) < FLOAT_EPS);
    ck_assert(fabs(msg->c_uc - 7.599592208862305e-7) < FLOAT_EPS);
    ck_assert(fabs(msg->c_us - 6.021931767463684e-6) < FLOAT_EPS);
    ck_assert(fabs(msg->c_ic + 1.1175870895385742e-8) < FLOAT_EPS);
    ck_assert(fabs(msg->c_is - 1.4901161193847656e-7) < FLOAT_EPS);

    ck_assert(fabs(msg->dn - 4.626621288776081e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->m0 - 1.0410100899287642) < FLOAT_EPS);
    ck_assert(fabs(msg->ecc - 8.054995676502585e-3) < FLOAT_EPS);
    ck_assert(fabs(msg->sqrta - 5153.665510177612) < FLOAT_EPS);
    ck_assert(fabs(msg->omega0 - 2.4056622999366137) < FLOAT_EPS);
    ck_assert(fabs(msg->omegadot + 8.301774373653793e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->w - 0.673068866191342) < FLOAT_EPS);
    ck_assert(fabs(msg->inc - 0.9720926277884092) < FLOAT_EPS);
    ck_assert(fabs(msg->inc_dot - 6.643133856047175e-11) < FLOAT_EPS);

    ck_assert(fabs(msg->af0 + 7.250206544995308e-5) < FLOAT_EPS);
    ck_assert(fabs(msg->af1 + 3.979039320256561e-12) < FLOAT_EPS);
    ck_assert(fabs(msg->af2 - 0.0) < FLOAT_EPS);

    ck_assert(msg->toc.wn == 2012);
    ck_assert(msg->toc.tow == 489600);
    ck_assert(msg->iode == 104);
    ck_assert(msg->iodc == 104);
  }
}

void sbp_callback_glo_eph(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  static bool checked_eph = false;
  /* ignore log messages */
  if (msg_id == SBP_MSG_EPHEMERIS_GLO && !checked_eph) {
    // clang-format off
    /* Truth data from related RINEX file
     R03 2018 08 20 22 45 00  1.410758122802e-04  0.000000000000e+00  8.100000000000e+04
     7.333627929688e+03      -1.816708564758e+00  0.000000000000e+00  0.000000000000e+00
     1.684325878906e+04      -1.609944343567e+00 -9.313225746155e-10  5.000000000000e+00
    -1.763590478516e+04      -2.291357994080e+00  1.862645149231e-09  0.000000000000e+00
     */
    /* Truth from Haskell converter
    {
      "gamma": 0,
      "vel": [
        -1816.7085647583008,
        -1609.9443435668945,
        -2291.35799407959
      ],
      "iod": 28,
      "d_tau": -2.7939677238464355e-09,
      "pos": [
        7333627.9296875,
        16843258.7890625,
        -17635904.78515625
      ],
      "crc": 39648,
      "common": {
        "health_bits": 0,
        "fit_interval": 2400,
        "valid": 1,
        "sid": {
          "sat": 3,
          "code": 3
        },
        "toe": {
          "wn": 2015,
          "tow": 254718
        },
        "ura": 2.5
      },
      "tau": -0.00014107581228017807,
      "fcn": 13,
      "acc": [
        0,
        -9.313225746154785e-07,
        1.862645149230957e-06
      ]
    }*/
    // clang-format on
    checked_eph = true;
    msg_ephemeris_glo_t *msg = (msg_ephemeris_glo_t *)buffer;
    ck_assert(msg->common.sid.sat == 3);
    ck_assert(msg->common.sid.code == CODE_GLO_L1OF);
    ck_assert(msg->common.toe.wn == 2015);
    ck_assert(msg->common.toe.tow == 168318);
    ck_assert(fabs(msg->common.ura - 2.5) < FLOAT_EPS);
    ck_assert(msg->common.fit_interval == 2400);
    ck_assert(msg->common.valid == 1);
    ck_assert(msg->common.health_bits == 0);
    ck_assert(msg->iod == 28);

    ck_assert(fabs(msg->pos[0] - 7.333627929688e6) <
              GLO_SATELLITE_POSITION_EPS_METERS);
    ck_assert(fabs(msg->pos[1] - 1.684325878906e7) <
              GLO_SATELLITE_POSITION_EPS_METERS);
    ck_assert(fabs(msg->pos[2] - -1.763590478516e7) <
              GLO_SATELLITE_POSITION_EPS_METERS);

    ck_assert(fabs(msg->vel[0] - -1.816708564758e3) < FLOAT_EPS);
    ck_assert(fabs(msg->vel[1] - -1.609944343567e3) < FLOAT_EPS);
    ck_assert(fabs(msg->vel[2] - -2.291357994080e3) < FLOAT_EPS);

    ck_assert(fabs(msg->acc[0] - 0) < FLOAT_EPS);
    ck_assert(fabs(msg->acc[1] - -9.313225746155e-7) < FLOAT_EPS);
    ck_assert(fabs(msg->acc[2] - 1.862645149231e-6) < FLOAT_EPS);

    ck_assert(fabs(msg->gamma - 0) < FLOAT_EPS);
    ck_assert(fabs(msg->d_tau - -2.7939677238464355e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->tau - -0.00014107581228017807) < FLOAT_EPS);
  }
}

void sbp_callback_gal_eph(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  static bool checked_eph = false;
  /* ignore log messages */
  if (msg_id == SBP_MSG_EPHEMERIS_GAL && !checked_eph) {
    // clang-format off
/*
 E01 2018 08 14 04 00 00  -4.123143153265e-04 -8.284928298963e-12 0.000000000000e+00
     5.600000000000e+01   -7.200000000000e+01 3.055484416047e-09 4.735169303722e-01
    -3.268942236900e-06    2.873298944905e-04 7.478520274162e-06 5.440604309082e+03
     1.872000000000e+05    7.264316082001e-08-3.479377544441e-01-2.421438694000e-08
     9.920462291303e-01    1.913437500000e+02-2.811246434921e+00-5.637734834285e-09
    -7.757465986739e-10    5.170000000000e+02 2.014000000000e+03
     3.120000000000e+00    0.000000000000e+00-4.889443516731e-09-5.587935447693e-09
     1.878640000000e+05
     */
    // clang-format on
    checked_eph = true;
    msg_ephemeris_gal_t *msg = (msg_ephemeris_gal_t *)buffer;
    ck_assert(msg->common.sid.sat == 1);
    ck_assert(msg->common.sid.code == CODE_GAL_E1B);
    ck_assert(msg->common.toe.wn == 2014);
    ck_assert(msg->common.toe.tow == 187200);
    ck_assert(fabs(msg->common.ura - 3.12) < FLOAT_EPS);
    ck_assert(msg->common.fit_interval == 14400);
    ck_assert(msg->common.valid == 1);
    ck_assert(msg->common.health_bits == 0);

    ck_assert(fabs(msg->bgd_e1e5a - -4.889443516731e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->bgd_e1e5b - 0) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rs - -7.200000000000e1) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rc - 1.913437500000e2) < FLOAT_EPS);
    ck_assert(fabs(msg->c_uc - -3.268942236900e-6) < FLOAT_EPS);
    ck_assert(fabs(msg->c_us - 7.478520274162e-6) < FLOAT_EPS);
    ck_assert(fabs(msg->c_ic - 7.264316082001e-8) < FLOAT_EPS);
    ck_assert(fabs(msg->c_is - -2.421438694000e-8) < FLOAT_EPS);

    ck_assert(fabs(msg->dn - 3.055484416047e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->m0 - 4.735169303722e-1) < FLOAT_EPS);
    ck_assert(fabs(msg->ecc - 2.873298944905e-4) < FLOAT_EPS);
    ck_assert(fabs(msg->sqrta - 5.440604309082e3) < FLOAT_EPS);
    ck_assert(fabs(msg->omega0 - -3.479377544441e-1) < FLOAT_EPS);
    ck_assert(fabs(msg->omegadot - -5.637734834285e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->w - -2.811246434921e0) < FLOAT_EPS);
    ck_assert(fabs(msg->inc - 9.920462291303e-1) < FLOAT_EPS);
    ck_assert(fabs(msg->inc_dot - -7.757465986739e-10) < FLOAT_EPS);

    ck_assert(fabs(msg->af0 - -4.123143153265e-4) < FLOAT_EPS);
    ck_assert(fabs(msg->af1 - -8.284928298963e-12) < FLOAT_EPS);
    ck_assert(fabs(msg->af2 - 0.0) < FLOAT_EPS);

    ck_assert(msg->toc.wn == 2014);
    ck_assert(msg->toc.tow == 187200);
    ck_assert(msg->iode == 56);
    ck_assert(msg->iodc == 56);
  }
}

void sbp_callback_bds_eph(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  static bool checked_eph = false;
  /* ignore log messages */
  if (msg_id == SBP_MSG_EPHEMERIS_BDS && !checked_eph) {
    // clang-format off
/*
C06 2018 08 14 04 00 00 2.242858754471e-04 2.508659946443e-11 1.924458856162e-18
     6.000000000000e+00-4.404687500000e+01 6.643133856047e-10 1.319269256677e+00
    -5.727633833885e-07 7.140220026486e-03 3.428151831031e-05 6.492804334641e+03
     1.872000000000e+05 1.392327249050e-07-2.286649500564e+00 2.207234501839e-07
     9.452314900799e-01-8.311562500000e+02-2.220683918913e+00-1.816147078387e-09
     7.307447241652e-10 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00
     2.000000000000e+00 0.000000000000e+00 8.100000000000e-09-1.800000000000e-09
     0.000000000000e+00 5.000000000000e+00
     */
    // clang-format on
    checked_eph = true;
    msg_ephemeris_bds_t *msg = (msg_ephemeris_bds_t *)buffer;
    ck_assert(msg->common.sid.sat == 6);
    ck_assert(msg->common.sid.code == CODE_BDS2_B1);

    ck_assert(msg->common.toe.wn == 2014);
    ck_assert(msg->common.toe.tow == 187214);
    ck_assert(fabs(msg->common.ura - 2.0) < FLOAT_EPS);
    ck_assert(msg->common.fit_interval == 10800);
    ck_assert(msg->common.valid == 1);
    ck_assert(msg->common.health_bits == 0);

    ck_assert(fabs(msg->tgd1 - 8.100000000000e-9) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->tgd2 - -1.800000000000e-9) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->c_rs - -4.40468750000e1) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rc - -8.31156250000000e2) < FLOAT_EPS);
    ck_assert(fabs(msg->c_uc - -5.7276338338850e-7) < FLOAT_EPS);
    ck_assert(fabs(msg->c_us - 3.428151831031e-5) < FLOAT_EPS);
    ck_assert(fabs(msg->c_ic - 1.392327249050e-7) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->c_is - 2.207234501839e-7) * 1e9 < FLOAT_EPS);

    ck_assert(fabs(msg->dn - 6.643133856047e-10) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->m0 - 1.319269256677) < FLOAT_EPS);
    ck_assert(fabs(msg->ecc - 7.140220026486e-3) < FLOAT_EPS);
    ck_assert(fabs(msg->sqrta - 6.492804334641e3) < FLOAT_EPS);
    ck_assert(fabs(msg->omega0 - -2.286649500564) < FLOAT_EPS);
    ck_assert(fabs(msg->omegadot - -1.816147078387e-9) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->w - -2.220683918913) < FLOAT_EPS);
    ck_assert(fabs(msg->inc - 9.452314900799e-1) < FLOAT_EPS);
    ck_assert(fabs(msg->inc_dot - 7.307447241652e-10) * 1e9 < FLOAT_EPS);

    ck_assert(fabs(msg->af0 - 2.242858754471e-4) < FLOAT_EPS);
    ck_assert(fabs(msg->af1 - 2.508659946443e-11) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->af2 - 1.924458856162e-18) * 1e9 < FLOAT_EPS);

    ck_assert(msg->toc.wn == 2014);
    ck_assert(msg->toc.tow == 187214);
    ck_assert(msg->iode == 6);
    ck_assert(msg->iodc == 5);
  }
  return;
}

void sbp_callback_eph_wn_rollover(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  if (msg_id == SBP_MSG_EPHEMERIS_GPS) {
    msg_ephemeris_gps_t *msg = (msg_ephemeris_gps_t *)buffer;
    ck_assert(msg->common.toe.wn == 2026);
    ck_assert(msg->common.toe.tow == 0);
  }
  return;
}

void sbp_callback_1012_first(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  (void)context;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *msg = (msg_obs_t *)buffer;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    ck_assert_uint_gt(num_sbp_msgs, 2);
    update_obs_time(msg);
  }
}

void sbp_callback_glo_day_rollover(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  (void)context;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *msg = (msg_obs_t *)buffer;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    ck_assert_uint_gt(num_sbp_msgs, 2);
    update_obs_time(msg);
  }
}

void check_biases(msg_glo_biases_t *sbp_glo_msg) {
  if (sbp_glo_msg->mask & 0x01) {
    ck_assert(sbp_glo_msg->l1ca_bias / GLO_BIAS_RESOLUTION ==
              expected_L1CA_bias);
  }
  if (sbp_glo_msg->mask & 0x02) {
    ck_assert(sbp_glo_msg->l1p_bias / GLO_BIAS_RESOLUTION == expected_L1P_bias);
  }
  if (sbp_glo_msg->mask & 0x04) {
    ck_assert(sbp_glo_msg->l2ca_bias / GLO_BIAS_RESOLUTION ==
              expected_L2CA_bias);
  }
  if (sbp_glo_msg->mask & 0x08) {
    ck_assert(sbp_glo_msg->l2p_bias / GLO_BIAS_RESOLUTION == expected_L2P_bias);
  }
}

void sbp_callback_bias(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg);
  } else if (msg_id == SBP_MSG_OBS) {
    update_obs_time((msg_obs_t *)buffer);
  }
}

static bool no_duplicate_observations(msg_obs_t *sbp_obs, u8 length) {
  static sbp_gps_time_t epoch_time;
  static gnss_sid_set_t sid_set;

  if (epoch_time.wn != sbp_obs->header.t.wn ||
      epoch_time.tow != sbp_obs->header.t.tow) {
    epoch_time = sbp_obs->header.t;
    sid_set_init(&sid_set);
  }

  u8 num_obs = (length - 11) / 17;
  for (u8 i = 0; i < num_obs; i++) {
    gnss_signal_t sid = {.code = sbp_obs->obs[i].sid.code,
                         .sat = sbp_obs->obs[i].sid.sat};
    if (sid_set_contains(&sid_set, sid)) {
      return false;
    }
    sid_set_add(&sid_set, sid);
  }
  return true;
}

void sbp_callback_msm_switching(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)length;
  (void)sender_id;
  (void)context;
  const u32 MAX_OBS_GAP_S = MSM_TIMEOUT_SEC + 10;

  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *sbp_obs = (msg_obs_t *)buffer;
    if (previous_obs_time.wn != INVALID_TIME) {
      double dt = sbp_diff_time(&sbp_obs->header.t, &previous_obs_time);
      /* make sure time does not run backwards */
      ck_assert(dt >= 0);
      /* check there aren't too long gaps between observations */
      ck_assert(dt < MAX_OBS_GAP_S);
      /* check there are no duplicate sids */
      ck_assert(no_duplicate_observations(sbp_obs, length));
    }
    previous_obs_time = sbp_obs->header.t;
    update_obs_time(sbp_obs);
  }
}

void sbp_callback_msm_no_gaps(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)sender_id;
  (void)context;
  const u32 MAX_OBS_GAP_S = 1;

  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *sbp_obs = (msg_obs_t *)buffer;

    if (previous_obs_time.wn != INVALID_TIME) {
      double dt = sbp_diff_time(&sbp_obs->header.t, &previous_obs_time);
      /* make sure time does not run backwards */
      ck_assert(dt >= 0);
      /* check there's an observation for every second */
      ck_assert(dt <= MAX_OBS_GAP_S);
      /* check there are no duplicate sids */
      ck_assert(no_duplicate_observations(sbp_obs, length));

      u8 n_meas = sbp_obs->header.n_obs;
      u8 seq_counter = n_meas & 0x0F;
      u8 seq_size = n_meas >> 4;
      u8 prev_seq_counter = previous_n_meas & 0x0F;
      u8 prev_seq_size = previous_n_meas >> 4;

      ck_assert_uint_gt(seq_size, 0);

      if (seq_counter == 0) {
        /* new observation sequence, verify that the last one did complete */
        ck_assert_uint_eq(prev_seq_counter, prev_seq_size - 1);
      } else {
        /* verify that the previous sequence continues */
        ck_assert_uint_eq(sbp_obs->header.t.wn, previous_obs_time.wn);
        ck_assert_uint_eq(sbp_obs->header.t.tow, previous_obs_time.tow);
        ck_assert_uint_eq(seq_size, prev_seq_size);
        ck_assert_uint_eq(seq_counter, prev_seq_counter + 1);
      }
      if (seq_counter < seq_size - 1) {
        /* verify that all but the last packet in the sequence are full */
        ck_assert_uint_eq(length, 249);
      } else {
        /* last message in sequence, check the total number of observations */
        u8 num_obs = (seq_size - 1) * 14 + (length - 11) / 17;
        if (previous_num_obs > 0) {
          /* must not lose more than 6 observations between epochs */
          ck_assert_uint_ge(num_obs, previous_num_obs - 6);
        }
        previous_num_obs = num_obs;
      }
    }
    previous_obs_time = sbp_obs->header.t;
    previous_n_meas = sbp_obs->header.n_obs;
    update_obs_time(sbp_obs);
  }
}

/* sanity check the length and CRC of the message */
bool verify_crc(uint8_t *buffer, uint32_t buffer_length) {
  if (buffer_length < 6) {
    /* buffer too short to be a valid message */
    return false;
  }
  uint16_t message_size = ((buffer[1] & 0x3) << 8) | buffer[2];
  if (buffer_length < (uint32_t)message_size + 6) {
    /* buffer too short to contain the message */
    return false;
  }
  /* Verify CRC */
  uint32_t computed_crc = crc24q(buffer, 3 + message_size, 0);
  uint32_t frame_crc = (buffer[message_size + 3] << 16) |
                       (buffer[message_size + 4] << 8) |
                       (buffer[message_size + 5] << 0);

  return (frame_crc == computed_crc);
}

void test_RTCM3(const char *filename,
                void (*cb_rtcm_to_sbp)(u16 msg_id,
                                       u8 length,
                                       u8 *buffer,
                                       u16 sender_id,
                                       void *context),
                gps_time_t current_time_) {
  rtcm2sbp_init(&state, cb_rtcm_to_sbp, NULL, NULL);
  rtcm2sbp_set_gps_time(&current_time_, &state);
  rtcm2sbp_set_leap_second(18, &state);

  previous_obs_time.wn = INVALID_TIME;
  previous_n_meas = 0;
  previous_num_obs = 0;

  FILE *fp = fopen(filename, "rb");
  u8 buffer[MAX_FILE_SIZE];
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;

  while (buffer_index < file_size) {
    if (buffer[buffer_index] != RTCM3_PREAMBLE) {
      /* not a start of RTCM frame, seeking forward */
      buffer_index++;
      continue;
    }

    uint16_t message_size =
        ((buffer[buffer_index + 1] & 0x3) << 8) | buffer[buffer_index + 2];

    if (message_size == 0) {
      buffer_index++;
      continue;
    }
    if (message_size > 1023) {
      /* too large message */
      buffer_index++;
      continue;
    }

    if (!verify_crc(&buffer[buffer_index], file_size - buffer_index)) {
      /* CRC failure */
      buffer_index++;
      continue;
    }

    rtcm2sbp_decode_frame(&buffer[buffer_index], message_size, &state);
    /* skip pointer to the end of this message */
    buffer_index += message_size + 6;
  }
}

static s32 sbp_read_file(u8 *buff, u32 n, void *context) {
  FILE *f = (FILE *)context;
  return fread(buff, 1, n, f);
}

static void ephemeris_glo_callback(u16 sender_id,
                                   u8 len,
                                   u8 msg[],
                                   void *context) {
  (void)context;
  (void)sender_id;
  (void)len;
  msg_ephemeris_glo_t *e = (msg_ephemeris_glo_t *)msg;

  /* extract just the FCN field */
  sbp2rtcm_set_glo_fcn(e->common.sid, e->fcn, &out_state);
}

static void test_SBP(const char *filename,
                     void (*cb_sbp_to_rtcm)(u8 *buffer,
                                            u16 length,
                                            void *context),
                     gps_time_t current_time_,
                     msm_enum msm_type) {
  (void)current_time_;
  sbp2rtcm_init(&out_state, cb_sbp_to_rtcm, NULL);
  sbp2rtcm_set_leap_second(18, &out_state);

  sbp2rtcm_set_rtcm_out_mode(msm_type, &out_state);

  previous_obs_time.wn = INVALID_TIME;
  previous_n_meas = 0;
  previous_num_obs = 0;

  FILE *fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  sbp_msg_callbacks_node_t sbp_base_pos_callback_node;
  sbp_msg_callbacks_node_t sbp_glo_biases_callback_node;
  sbp_msg_callbacks_node_t sbp_obs_callback_node;
  sbp_msg_callbacks_node_t sbp_ephemeris_glo_callback_node;

  sbp_state_t s;
  sbp_state_init(&s);

  sbp_register_callback(&s,
                        SBP_MSG_BASE_POS_ECEF,
                        (void *)&sbp2rtcm_base_pos_ecef_cb,
                        &out_state,
                        &sbp_base_pos_callback_node);
  sbp_register_callback(&s,
                        SBP_MSG_GLO_BIASES,
                        (void *)&sbp2rtcm_glo_biases_cb,
                        &out_state,
                        &sbp_glo_biases_callback_node);
  sbp_register_callback(&s,
                        SBP_MSG_OBS,
                        (void *)&sbp2rtcm_sbp_obs_cb,
                        &out_state,
                        &sbp_obs_callback_node);
  sbp_register_callback(&s,
                        SBP_MSG_EPHEMERIS_GLO,
                        (void *)&ephemeris_glo_callback,
                        &out_state,
                        &sbp_ephemeris_glo_callback_node);

  sbp_state_set_io_context(&s, fp);

  while (!feof(fp)) {
    sbp_process(&s, &sbp_read_file);
  }
  fclose(fp);
}

static void rtcm_sanity_check_cb(u8 *buffer, u16 length, void *context) {
  (void)context;

  u16 byte = 0;
  ck_assert_uint_eq(buffer[byte], RTCM3_PREAMBLE);
  byte = 1;
  u16 message_size = ((buffer[byte] & 0x3) << 8) | buffer[byte + 1];
  ck_assert_uint_eq(message_size, length - RTCM3_MSG_OVERHEAD);
}

void set_expected_bias(double L1CA_bias,
                       double L1P_bias,
                       double L2CA_bias,
                       double L2P_bias) {
  expected_L1CA_bias = L1CA_bias;
  expected_L1P_bias = L1P_bias;
  expected_L2CA_bias = L2CA_bias;
  expected_L2P_bias = L2P_bias;
}

/* end rtcm helpers */

/* fixture globals and functions */

void rtcm3_setup_basic(void) {
  memset(&current_time, 0, sizeof(current_time));
  current_time.wn = 1945;
  current_time.tow = 211190;
}

/* end fixtures */

START_TEST(test_gps_time) {
  current_time.wn = 1945;
  current_time.tow = 277500;
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/RTCM3.bin", sbp_callback_gps, current_time);
}
END_TEST

START_TEST(test_glo_day_rollover) {
  current_time.wn = 1959;
  current_time.tow = 510191;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/glo_day_rollover.rtcm",
             sbp_callback_glo_day_rollover,
             current_time);
}
END_TEST

START_TEST(test_1012_first) {
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/1012_first.rtcm",
             sbp_callback_1012_first,
             current_time);
}
END_TEST

/* Test parsing of raw file with MSM5 obs */
START_TEST(test_msm5_parse) {
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/jenoba-jrr32m.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

/* Test parsing of raw file with MSM7 obs */
START_TEST(test_msm7_parse) {
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/msm7.rtcm",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

START_TEST(test_msm_missing_obs) {
  current_time.wn = 2007;
  current_time.tow = 289790;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/missing-gps.rtcm",
             sbp_callback_msm_no_gaps,
             current_time);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/dropped-packets-STR24.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/missing-beidou-STR17.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

START_TEST(test_msm_week_rollover) {
  current_time.wn = 2009;
  current_time.tow = 604200;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/week-rollover-STR17.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
  current_time.wn = 2009;
  current_time.tow = 604200;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/week-rollover-STR24.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

/* Test switching between RTCM streams: legacy->MSM->legacy->MSM. The file
 * contains 2 minutes raw data per feed (with about 10s pause when switching)
 * from these EUREF stations:
 * OSLS00NOR0 TRIMBLE NETR9         RTCM 3.1
 *          1004(1),1005(5),1007(5),1012(1),1033(5),4094(5)
 * KUNZ00CZE0 TRIMBLE SPS855        RTCM 3.2
 *          1006(10),1008(10),1013(10),1033(10),1074(1),1084(1),1094(1),1124(1),1230(10)
 * SUR400EST0 LEICA GR25            RTCM 3.2
 *          1004(1),1006(10),1008(10),1012(1),1013(10),1033(10),1230(10)
 * ORIV00FIN0 JAVAD TRE_G3TH DELTA  RTCM 3.3
 *          1006(30),1008(30),1019,1020,1077(1),1087(1),1097(1),1107(1),1117(1),1127(1),1230(60)
 *
 * Check that there are no more than 60s gaps in observations when switching
 * from MSM feed to legacy feed.
 *
 */
START_TEST(test_msm_switching) {
  current_time.wn = 2002;
  current_time.tow = 308700;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/switch-legacy-msm-legacy-msm.rtcm",
             sbp_callback_msm_switching,
             current_time);
}
END_TEST

/* parse an artificially generated stream with both legacy and MSM observations,
 * verify there's no crash or duplicate observations
 */
START_TEST(test_msm_mixed) {
  current_time.wn = 2002;
  current_time.tow = 375900;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/mixed-msm-legacy.rtcm",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

/* Test 1033 message sources */
START_TEST(test_bias_trm) {
  set_expected_bias(
      TRIMBLE_BIAS_M, TRIMBLE_BIAS_M, TRIMBLE_BIAS_M, TRIMBLE_BIAS_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/trimble.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_jav) {
  set_expected_bias(JAVAD_BIAS_L1CA_M, 0.0, 0.0, JAVAD_BIAS_L2P_M);
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/javad.rtcm", sbp_callback_bias, current_time);
}
END_TEST

START_TEST(test_bias_nov) {
  set_expected_bias(
      NOVATEL_BIAS_M, NOVATEL_BIAS_M, NOVATEL_BIAS_M, NOVATEL_BIAS_M);
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/leica.rtcm", sbp_callback_bias, current_time);
}
END_TEST

START_TEST(test_bias_sep) {
  set_expected_bias(SEPTENTRIO_BIAS_M,
                    SEPTENTRIO_BIAS_M,
                    SEPTENTRIO_BIAS_M,
                    SEPTENTRIO_BIAS_M);
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/sept.rtcm", sbp_callback_bias, current_time);
}
END_TEST

START_TEST(test_bias_nav) {
  set_expected_bias(NAVCOM_BIAS_L1CA_M, 0.0, 0.0, NAVCOM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/navcom.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_top) {
  set_expected_bias(TOPCON_BIAS_M, TOPCON_BIAS_M, TOPCON_BIAS_M, TOPCON_BIAS_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/topcon.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_hem) {
  set_expected_bias(HEMISPHERE_BIAS_L1CA_M, 0.0, 0.0, HEMISPHERE_BIAS_L2P_M);
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/hemi.rtcm", sbp_callback_bias, current_time);
}
END_TEST

/* Test 1033 messages from GEO++ */
START_TEST(test_bias_gpp_ash1) {
  set_expected_bias(GPP_ASH1_BIAS_L1CA_M, 0.0, 0.0, GPP_ASH1_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_ASH1.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_hem) {
  set_expected_bias(GPP_HEM_BIAS_L1CA_M, 0.0, 0.0, GPP_HEM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_HEM.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_jav) {
  set_expected_bias(GPP_JAV_BIAS_L1CA_M, 0.0, 0.0, GPP_JAV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_JAV.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_jps) {
  set_expected_bias(GPP_JPS_BIAS_L1CA_M, 0.0, 0.0, GPP_JPS_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_JPS.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_lei) {
  set_expected_bias(GPP_NOV_BIAS_L1CA_M, 0.0, 0.0, GPP_NOV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_LEI.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_nav) {
  set_expected_bias(GPP_NAV_BIAS_L1CA_M, 0.0, 0.0, GPP_NAV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NAV.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_nov) {
  set_expected_bias(GPP_NOV_BIAS_L1CA_M, 0.0, 0.0, GPP_NOV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NOV.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_nvr) {
  set_expected_bias(GPP_NVR_BIAS_L1CA_M, 0.0, 0.0, GPP_NVR_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NVR.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_sep1) {
  set_expected_bias(GPP_SEP_BIAS_L1CA_M, 0.0, 0.0, GPP_SEP_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_SEP1.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_sok) {
  set_expected_bias(GPP_SOK_BIAS_L1CA_M, 0.0, 0.0, GPP_SOK_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_SOK.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_tps1) {
  set_expected_bias(GPP_TPS_BIAS_L1CA_M, 0.0, 0.0, GPP_TPS_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_TPS1.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_trm) {
  set_expected_bias(GPP_TRM_BIAS_L1CA_M, 0.0, 0.0, GPP_TRM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_TRM.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_gps) {
  current_time.wn = 2012;
  current_time.tow = 489000;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/eph.rtcm",
             sbp_callback_gps_eph,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_glo) {
  current_time.wn = 2015;
  current_time.tow = 168318;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/test_glo_eph.rtcm",
             sbp_callback_glo_eph,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_gal) {
  current_time.wn = 2014;
  current_time.tow = 187816;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/test_gal_eph.rtcm",
             sbp_callback_gal_eph,
             current_time);
}
END_TEST

START_TEST(test_sbp_to_rtcm_legacy) {
  current_time.wn = 2020;
  current_time.tow = 211000;

  test_SBP(RELATIVE_PATH_PREFIX "/data/piksi-gps-glo.sbp",
           rtcm_sanity_check_cb,
           current_time,
           MSM_UNKNOWN);
}
END_TEST

START_TEST(test_sbp_to_rtcm_msm) {
  current_time.wn = 2020;
  current_time.tow = 211000;

  test_SBP(RELATIVE_PATH_PREFIX "/data/piksi-gps-glo.sbp",
           rtcm_sanity_check_cb,
           current_time,
           MSM5);
}
END_TEST

START_TEST(tc_rtcm_eph_bds) {
  current_time.wn = 2014;
  current_time.tow = 187816;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/test_bds_eph.rtcm",
             sbp_callback_bds_eph,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_wn_rollover) {
  current_time.wn = 2025;
  current_time.tow = 487800;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/eph_wn.rtcm",
             sbp_callback_eph_wn_rollover,
             current_time);
}
END_TEST

static void rtcm_roundtrip_cb(u8 *buffer, u16 length, void *context) {
  (void)context;

  u16 byte = 0;
  ck_assert_uint_eq(buffer[byte], RTCM3_PREAMBLE);
  byte = 1;
  u16 message_size = ((buffer[byte] & 0x3) << 8) | buffer[byte + 1];
  ck_assert_uint_eq(message_size, length - RTCM3_MSG_OVERHEAD);

  /* feed the received RTCM frame back to decoder */
  rtcm2sbp_decode_frame(buffer, length, &state);
}

static void sbp_roundtrip_cb(
    u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context) {
  (void)sender_id;
  (void)context;

  ck_assert_uint_eq(msg_id, SBP_MSG_OBS);
  u8 num_obs = (length - 11) / 17;
  msg_obs_t *sbp_obs = (msg_obs_t *)buffer;

  for (u8 i = 0; i < num_obs; i++) {
    packed_obs_content_t *converted_obs = &sbp_obs->obs[i];
    /* find the original SBP observation */
    packed_obs_content_t *orig_obs = NULL;
    for (u8 j = 0; j < ARRAY_SIZE(sbp_test_data); j++) {
      if (sbp_test_data[j].sid.code == converted_obs->sid.code &&
          sbp_test_data[j].sid.sat == converted_obs->sid.sat) {
        orig_obs = &sbp_test_data[j];
        break;
      }
    }
    ck_assert_ptr_ne(orig_obs, NULL);
    /* check that the sbp->msm->sbp converted observation is identical to the
     * original */
    ck_assert_uint_eq(orig_obs->P, converted_obs->P);
    ck_assert_uint_eq(orig_obs->L.i, converted_obs->L.i);
    ck_assert_uint_eq(orig_obs->L.f, converted_obs->L.f);
    ck_assert_uint_eq(orig_obs->D.i, converted_obs->D.i);
    ck_assert_uint_eq(orig_obs->D.f, converted_obs->D.f);
    ck_assert_uint_eq(orig_obs->cn0, converted_obs->cn0);
    ck_assert_uint_eq(orig_obs->lock, converted_obs->lock);
    ck_assert_uint_eq(orig_obs->flags, converted_obs->flags);
  }
}

START_TEST(test_sbp_to_msm_roundtrip) {
  current_time.wn = 2022;
  current_time.tow = 210853;
  sbp2rtcm_init(&out_state, rtcm_roundtrip_cb, NULL);
  rtcm2sbp_init(&state, sbp_roundtrip_cb, NULL, NULL);
  sbp2rtcm_set_leap_second(18, &out_state);
  rtcm2sbp_set_leap_second(18, &state);

  /* set the FCNs for couple of GLO satellites */
  sbp_gnss_signal_t sid = {2, CODE_GLO_L1OF};
  sbp2rtcm_set_glo_fcn(sid, 4, &out_state);
  rtcm2sbp_set_glo_fcn(sid, 4, &state);
  sid.sat = 3;
  sbp2rtcm_set_glo_fcn(sid, 13, &out_state);
  rtcm2sbp_set_glo_fcn(sid, 13, &state);
  sid.sat = 11;
  sbp2rtcm_set_glo_fcn(sid, 8, &out_state);
  rtcm2sbp_set_glo_fcn(sid, 8, &state);

  rtcm2sbp_set_gps_time(&current_time, &state);

  memcpy(out_state.sbp_obs_buffer, sbp_test_data, sizeof(sbp_test_data));
  out_state.n_sbp_obs = ARRAY_SIZE(sbp_test_data);

  sbp_buffer_to_msm(&out_state);
}
END_TEST

Suite *rtcm3_suite(void) {
  Suite *s = suite_create("RTCMv3");

  TCase *tc_core = tcase_create("Core");
  tcase_add_checked_fixture(tc_core, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_core, test_gps_time);
  tcase_add_test(tc_core, test_glo_day_rollover);
  tcase_add_test(tc_core, test_1012_first);
  suite_add_tcase(s, tc_core);

  TCase *tc_biases = tcase_create("Biases");
  tcase_add_checked_fixture(tc_biases, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_biases, test_bias_trm);
  tcase_add_test(tc_biases, test_bias_jav);
  tcase_add_test(tc_biases, test_bias_nov);
  tcase_add_test(tc_biases, test_bias_sep);
  tcase_add_test(tc_biases, test_bias_nav);
  tcase_add_test(tc_biases, test_bias_top);
  tcase_add_test(tc_biases, test_bias_hem);
  suite_add_tcase(s, tc_biases);

  TCase *tc_gpp_biases = tcase_create("Geo++ Biases");
  tcase_add_checked_fixture(tc_gpp_biases, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_ash1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_hem);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_jav);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_jps);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_lei);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nav);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nov);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nvr);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_sep1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_sok);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_tps1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_trm);
  suite_add_tcase(s, tc_gpp_biases);

  TCase *tc_msm = tcase_create("MSM");
  tcase_add_checked_fixture(tc_msm, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_msm, test_msm5_parse);
  tcase_add_test(tc_msm, test_msm7_parse);
  tcase_add_test(tc_msm, test_msm_switching);
  tcase_add_test(tc_msm, test_msm_mixed);
  tcase_add_test(tc_msm, test_msm_missing_obs);
  tcase_add_test(tc_msm, test_msm_week_rollover);
  suite_add_tcase(s, tc_msm);

  TCase *tc_eph = tcase_create("ephemeris");
  tcase_add_checked_fixture(tc_eph, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_eph, tc_rtcm_eph_gps);
  tcase_add_test(tc_eph, tc_rtcm_eph_glo);
  tcase_add_test(tc_eph, tc_rtcm_eph_gal);
  tcase_add_test(tc_eph, tc_rtcm_eph_bds);
  tcase_add_test(tc_eph, tc_rtcm_eph_wn_rollover);
  suite_add_tcase(s, tc_eph);

  TCase *tc_sbp_to_rtcm = tcase_create("sbp2rtcm");
  tcase_add_checked_fixture(tc_sbp_to_rtcm, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_legacy);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_msm);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_msm_roundtrip);
  suite_add_tcase(s, tc_sbp_to_rtcm);

  return s;
}
