/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* This is a stand-alone tool that takes Swift Binary Protocol (SBP) on stdin
 * and writes RTCM3 on stdout. */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <gnss-converters/rtcm3_sbp.h>
#include <libsbp/sbp.h>

/* Write the RTCM frame to STDOUT. */
static void cb_sbp_to_rtcm(u8 *buffer, u16 n, void *context) {
  (void)(context);

  ssize_t numwritten = write(STDOUT_FILENO, buffer, n);
  if (numwritten < n) {
    fprintf(stderr, "Write failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }
}

static s32 sbp_read_stdin(u8 *buff, u32 n, void *context) {
  (void)context;
  ssize_t read_bytes = read(STDIN_FILENO, buff, n);
  if (read_bytes < 0) {
    fprintf(stderr, "Read failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }
  if (n > 0 && read_bytes == 0) {
    /* EOF */
    exit(EXIT_SUCCESS);
  }
  return (u32)read_bytes;
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
  sbp2rtcm_set_glo_fcn(
      e->common.sid, e->fcn, (struct rtcm3_out_state *)context);
}

int main(int argc, char **argv) {
  (void)(argc);
  (void)(argv);

  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, cb_sbp_to_rtcm, NULL);
  sbp2rtcm_set_leap_second(18, &state); /* TODO */

  sbp_msg_callbacks_node_t sbp_base_pos_callback_node;
  sbp_msg_callbacks_node_t sbp_glo_biases_callback_node;
  sbp_msg_callbacks_node_t sbp_obs_callback_node;
  sbp_msg_callbacks_node_t sbp_osr_callback_node;
  sbp_msg_callbacks_node_t sbp_ephemeris_glo_callback_node;

  sbp_state_t sbp_state;
  sbp_state_init(&sbp_state);

  sbp_register_callback(&sbp_state,
                        SBP_MSG_BASE_POS_ECEF,
                        (void *)&sbp2rtcm_base_pos_ecef_cb,
                        &state,
                        &sbp_base_pos_callback_node);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_GLO_BIASES,
                        (void *)&sbp2rtcm_glo_biases_cb,
                        &state,
                        &sbp_glo_biases_callback_node);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_OBS,
                        (void *)&sbp2rtcm_sbp_obs_cb,
                        &state,
                        &sbp_obs_callback_node);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_OSR,
                        (void *)&sbp2rtcm_sbp_osr_cb,
                        &state,
                        &sbp_osr_callback_node);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_EPHEMERIS_GLO,
                        (void *)&ephemeris_glo_callback,
                        &state,
                        &sbp_ephemeris_glo_callback_node);

  while (!feof(stdin)) {
    sbp_process(&sbp_state, &sbp_read_stdin);
  }
  return 0;
}
