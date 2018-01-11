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

#include "rtcm3_sbp_test.h"
#include "../src/rtcm3_sbp_internal.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_FILE_SIZE 26000

void sbp_callback_gps(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  static uint32_t msg_count = 0;
  if (msg_count == 3 || msg_count == 20 || msg_count == 42) {
    assert(msg_id == SBP_MSG_BASE_POS_ECEF);
  } else if (msg_count == 4) {
    assert(msg_id == SBP_MSG_GLO_BIASES);
  } else {
    assert(msg_id == SBP_MSG_OBS);
  }
  msg_count++;
}

void sbp_callback_glo_day_rollover(u16 msg_id, u8 length, u8 *buffer,
                                   u16 sender_id) {
  (void)length;
  (void)buffer;
  (void)sender_id;
  static uint32_t msg_count = 0;
  if (msg_id == SBP_MSG_OBS) {
    msg_obs_t *msg = (msg_obs_t *)buffer;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    assert(num_sbp_msgs > 2);
  }
  msg_count++;
}

void check_biases(msg_glo_biases_t *sbp_glo_msg, double l1ca_bias,
                  double l1p_bias, double l2ca_bias, double l2p_bias) {
  if (sbp_glo_msg->mask & 0x01) {
    assert(sbp_glo_msg->l1ca_bias / GLO_BIAS_RESOLUTION == l1ca_bias);
  }
  if (sbp_glo_msg->mask & 0x02) {
    assert(sbp_glo_msg->l1p_bias / GLO_BIAS_RESOLUTION == l1p_bias);
  }
  if (sbp_glo_msg->mask & 0x04) {
    assert(sbp_glo_msg->l2ca_bias / GLO_BIAS_RESOLUTION == l2ca_bias);
  }
  if (sbp_glo_msg->mask & 0x08) {
    assert(sbp_glo_msg->l2p_bias / GLO_BIAS_RESOLUTION == l2p_bias);
  }
}

void sbp_callback_trimble(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg, TRIMBLE_BIAS_M, TRIMBLE_BIAS_M, TRIMBLE_BIAS_M,
                 TRIMBLE_BIAS_M);
  }
}

void sbp_callback_sept(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg, SEPTENTRIO_BIAS_M, SEPTENTRIO_BIAS_M,
                 SEPTENTRIO_BIAS_M, SEPTENTRIO_BIAS_M);
  }
}

void sbp_callback_javad(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg, JAVAD_BIAS_L1CA_M, 0.0, 0.0, JAVAD_BIAS_L2P_M);
  }
}

void sbp_callback_leica(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg, NOVATEL_BIAS_M, NOVATEL_BIAS_M, NOVATEL_BIAS_M,
                 NOVATEL_BIAS_M);
  }
}

void sbp_callback_navcom(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg, NAVCOM_BIAS_L1CA_M, 0.0, 0.0, NAVCOM_BIAS_L2P_M);
  }
}

void sbp_callback_topcon(u16 msg_id, u8 length, u8 *buffer, u16 sender_id) {
  (void)length;
  (void)sender_id;
  if (msg_id == SBP_MSG_GLO_BIASES) {
    msg_glo_biases_t *sbp_glo_msg = (msg_glo_biases_t *)buffer;
    check_biases(sbp_glo_msg, TOPCON_BIAS_M, TOPCON_BIAS_M, TOPCON_BIAS_M, TOPCON_BIAS_M);
  }
}

void test_RTCM3_decode(void) {
  // This file is GPS only and tests basic decoding functionality
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_gps, NULL);
  gps_time_sec_t current_time;
  current_time.wn = 1945;
  current_time.tow = 277500;
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen("../../tests/data/RTCM3.bin", "rb");

  u8 buffer[MAX_FILE_SIZE];

  if (fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    buffer_index++;
  }

  return;
}

void test_day_rollover(void) {
  // This test contains GPS and GLO obs where the GPS-GLO time difference rolls
  // over a day boundary This test excercises the day rollover time matching
  // code as well as the GPS GLO decoding functionality and time matching

  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_glo_day_rollover, NULL);
  gps_time_sec_t current_time;
  current_time.wn = 1959;
  current_time.tow = 510191;
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen("../../tests/data/glo_day_rollover.rtcm", "rb");

  u8 buffer[MAX_FILE_SIZE];

  if (fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    buffer_index++;
  }

  return;
}

void test_trimble_1033(void) {
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_trimble, NULL);
  gps_time_sec_t current_time;
  current_time.wn = 1959;
  current_time.tow = 510191;
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen("../../tests/data/trimble.rtcm", "rb");

  u8 buffer[MAX_FILE_SIZE];

  if (fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    // Reset the state as we want to see every 1230 and 1033 in the file.
    state.last_1230_received.wn = 0;
    buffer_index++;
  }

  return;
}

void test_javad_1033(void) {
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_javad, NULL);
  gps_time_sec_t current_time;
  current_time.wn = 1959;
  current_time.tow = 510191;
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen("../../tests/data/javad.rtcm", "rb");

  u8 buffer[MAX_FILE_SIZE];

  if (fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    // Reset the state as we want to see every 1230 and 1033 in the file.
    state.last_1230_received.wn = 0;
    buffer_index++;
  }

  return;
}

void test_leica_1033(void) {
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_leica, NULL);
  gps_time_sec_t current_time;
  current_time.wn = 1959;
  current_time.tow = 510191;
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen("../../tests/data/leica.rtcm", "rb");

  u8 buffer[MAX_FILE_SIZE];

  if (fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    // Reset the state as we want to see every 1230 and 1033 in the file.
    state.last_1230_received.wn = 0;
    buffer_index++;
  }

  return;
}

void test_sept_1033(void) {
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_sept, NULL);
  gps_time_sec_t current_time;
  current_time.wn = 1959;
  current_time.tow = 510191;
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen("../../tests/data/sept.rtcm", "rb");

  u8 buffer[MAX_FILE_SIZE];

  if (fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    // Reset the state as we want to see every 1230 and 1033 in the file.
    state.last_1230_received.wn = 0;
    buffer_index++;
  }

  return;
}

void test_navcom_1033(void) {
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_navcom, NULL);
  gps_time_sec_t current_time;
  current_time.wn = 1959;
  current_time.tow = 510191;
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen("../../tests/data/navcom.rtcm", "rb");

  u8 buffer[MAX_FILE_SIZE];

  if (fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    // Reset the state as we want to see every 1230 and 1033 in the file.
    state.last_1230_received.wn = 0;
    buffer_index++;
  }

  return;
}

void test_topcon_1033(void) {
  struct rtcm3_sbp_state state;
  rtcm2sbp_init(&state, sbp_callback_topcon, NULL);
  gps_time_sec_t current_time;
  current_time.wn = 1959;
  current_time.tow = 510191;
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second(18, &state);

  FILE *fp = fopen("../../tests/data/topcon.rtcm", "rb");

  u8 buffer[MAX_FILE_SIZE];

  if (fp == NULL) {
    fprintf(stderr, "Can't open input file!\n");
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;
  while (buffer_index < file_size) {

    if (buffer[buffer_index] == 0xD3) {
      rtcm2sbp_decode_frame(&buffer[buffer_index], file_size - buffer_index,
                            &state);
    }
    // Reset the state as we want to see every 1230 and 1033 in the file.
    state.last_1230_received.wn = 0;
    buffer_index++;
  }

  return;
}

int main(void) {
  test_RTCM3_decode();
  test_day_rollover();
  test_trimble_1033();
  test_javad_1033();
  test_leica_1033();
  test_sept_1033();
  test_navcom_1033();
  test_topcon_1033();
}
