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

#include <config.h>
#include "../src/rtcm3_sbp_internal.h"

#include "check_suites.h"

/* rtcm helper defines and functions */

#define MAX_FILE_SIZE 251818

static double expected_L1CA_bias = 0.0;
static double expected_L1P_bias = 0.0;
static double expected_L2CA_bias = 0.0;
static double expected_L2P_bias = 0.0;

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

static uint32_t crc24q(const uint8_t *buf, uint32_t len, uint32_t crc) {
  for (uint32_t i = 0; i < len; i++)
    crc = ((crc << 8) & 0xFFFFFF) ^ crc24qtab[((crc >> 16) ^ buf[i]) & 0xff];
  return crc;
}

void sbp_callback_gps(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  static uint32_t msg_count = 0;
  /* ignore log messages */
  if (msg_id == SBP_MSG_LOG) return;
  if (msg_count == 3 || msg_count == 20 || msg_count == 42) {
    ck_assert_uint_eq(msg_id, SBP_MSG_BASE_POS_ECEF);
  } else if (msg_count == 4) {
    ck_assert_uint_eq(msg_id, SBP_MSG_GLO_BIASES);
  } else {
    ck_assert_uint_eq(msg_id, SBP_MSG_OBS);
  }
  msg_count++;
}

void sbp_callback_1012_first(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *msg = (msg_obs_t *)buffer;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    ck_assert_uint_gt(num_sbp_msgs, 2);
  }
}

void sbp_callback_missing_obs(u16 msg_id,
                              u8 length,
                              u8 *buffer,
                              u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *msg = (msg_obs_t *)buffer;
    u8 num_meas = msg->header.n_obs;
    /* every epoch should have at least 80 base observations */
    ck_assert_uint_ge(num_meas, 80);
  }
}

void sbp_callback_glo_day_rollover(u16 msg_id,
                                   u8 length,
                                   u8 *buffer,
                                   u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *msg = (msg_obs_t *)buffer;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    ck_assert_uint_gt(num_sbp_msgs, 2);
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

void sbp_callback_bias(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg);
  }
}

void sbp_callback_msm(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_LOG) {
    msg_log_t *sbp_log = (msg_log_t *)buffer;
    ck_assert_uint_eq(sbp_log->level, RTCM_MSM_LOGGING_LEVEL);
  }
}

void sbp_callback_msm_switch(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  const u32 MAX_OBS_GAP_MS = (MSM_TIMEOUT_SEC + 5) * SECS_MS;

  static u32 previous_obs_tow = 0;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *sbp_obs = (msg_obs_t *)buffer;
    if (previous_obs_tow > 0) {
      /* make sure time does not run backwards */
      ck_assert_uint_ge(sbp_obs->header.t.tow, previous_obs_tow);
      /* check there aren't too long gaps between observations */
      ck_assert((sbp_obs->header.t.tow - previous_obs_tow) < MAX_OBS_GAP_MS);
    }
    previous_obs_tow = sbp_obs->header.t.tow;
  }
}

void sbp_callback_msm_mixed(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;

  static u32 previous_obs_tow = 0;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *sbp_obs = (msg_obs_t *)buffer;
    if (previous_obs_tow > 0) {
      /* make sure time does not run backwards */
      ck_assert_uint_ge(sbp_obs->header.t.tow, previous_obs_tow);
    }
    previous_obs_tow = sbp_obs->header.t.tow;
  }
}

/* sanity check the length and CRC of the message */
bool verify_crc(uint8_t *buffer, uint16_t buffer_length) {
  if (buffer_length < 6) {
    /* buffer too short to be a valid message */
    return false;
  }
  uint16_t message_size = ((buffer[1] & 0x3) << 8) | buffer[2];
  if (buffer_length < message_size + 6) {
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

void test_RTCM3(
    const char *filename,
    void (*cb_rtcm_to_sbp)(u16 msg_id, u8 length, u8 *buffer, u16 sender_id),
    gps_time_sec_t current_time) {
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, cb_rtcm_to_sbp, NULL);
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen(filename, "rb");
  u8 buffer[MAX_FILE_SIZE];
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {
    if (buffer[buffer_index] == 0xD3 &&
        verify_crc(&buffer[buffer_index], file_size - buffer_index)) {
      rtcm2sbp_decode_frame(
          &buffer[buffer_index], file_size - buffer_index, &state);
    }
    buffer_index++;
  }

  return;
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
gps_time_sec_t current_time;

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

/* Test MSM is properly rejected */
START_TEST(test_msm_reject) {
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/msm.rtcm", sbp_callback_msm, current_time);
}
END_TEST

/* Test parsing of raw file with MSM5 obs */
START_TEST(test_msm5_parse) {
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/jenoba-jrr32m.rtcm3",
             sbp_callback_msm,
             current_time);
}
END_TEST

/* Test parsing of raw file with MSM7 obs */
START_TEST(test_msm7_parse) {
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/msm7.rtcm", sbp_callback_msm, current_time);
}
END_TEST

START_TEST(test_msm_missing_obs) {
  current_time.wn = 2007;
  current_time.tow = 280000;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/missing-gps.rtcm",
             sbp_callback_missing_obs,
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
             sbp_callback_msm_switch,
             current_time);
}
END_TEST

/* parse an artificially generated stream with both legacy and MSM observations,
 * verify there's no crash
 */
START_TEST(test_msm_mixed) {
  current_time.wn = 2002;
  current_time.tow = 375900;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/mixed-msm-legacy.rtcm",
             sbp_callback_msm_mixed,
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
  tcase_add_test(tc_msm, test_msm_reject);
  tcase_add_test(tc_msm, test_msm5_parse);
  tcase_add_test(tc_msm, test_msm7_parse);
  tcase_add_test(tc_msm, test_msm_switching);
  tcase_add_test(tc_msm, test_msm_mixed);
  tcase_add_test(tc_msm, test_msm_missing_obs);
  suite_add_tcase(s, tc_msm);

  return s;
}
