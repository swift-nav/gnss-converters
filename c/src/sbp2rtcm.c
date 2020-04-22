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
#include <libsbp/logging.h>
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <swiftnav/gnss_time.h>
#include <unistd.h>

#include "gnss-converters/sbp_rtcm3.h"

/* Write the RTCM frame to STDOUT. */
static s32 cb_sbp_to_rtcm(u8 *buffer, u16 n, void *context) {
  (void)(context);

  ssize_t numwritten = write(STDOUT_FILENO, buffer, n);
  if (numwritten < n) {
    fprintf(stderr, "Write failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }

  return numwritten;
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
  return read_bytes;
}

typedef struct {
  sbp_msg_callbacks_node_t base_pos;
  sbp_msg_callbacks_node_t glo_biases;
  sbp_msg_callbacks_node_t obs;
  sbp_msg_callbacks_node_t osr;
  sbp_msg_callbacks_node_t ssr_orbit_clock;
  sbp_msg_callbacks_node_t ssr_phase_biases;
  sbp_msg_callbacks_node_t ssr_code_biases;
  sbp_msg_callbacks_node_t ssr_gridded_correction;
  sbp_msg_callbacks_node_t ssr_grid_definition;
  sbp_msg_callbacks_node_t ssr_stec_correction;
  sbp_msg_callbacks_node_t ephemeris_gps;
  sbp_msg_callbacks_node_t ephemeris_gal;
  sbp_msg_callbacks_node_t ephemeris_bds;
  sbp_msg_callbacks_node_t ephemeris_qzss;
  sbp_msg_callbacks_node_t ephemeris_glo;
  sbp_msg_callbacks_node_t log;
} sbp_nodes_t;

int sbp2rtcm_main(int argc, char **argv) {
  (void)(argc);
  (void)(argv);

  /* set time from systime, account for UTC<->GPS leap second difference */
  time_t ct_utc_unix = time(NULL);
  gps_time_t noleapsec = time2gps_t(ct_utc_unix);
  double gps_utc_offset = get_gps_utc_offset(&noleapsec, NULL);

  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, cb_sbp_to_rtcm, NULL);
  sbp2rtcm_set_leap_second((s8)lrint(gps_utc_offset), &state);

  sbp_nodes_t sbp_nodes;
  sbp_state_t sbp_state;
  sbp_state_init(&sbp_state);

  sbp_register_callback(&sbp_state,
                        SBP_MSG_BASE_POS_ECEF,
                        (void *)&sbp2rtcm_base_pos_ecef_cb,
                        &state,
                        &sbp_nodes.base_pos);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_GLO_BIASES,
                        (void *)&sbp2rtcm_glo_biases_cb,
                        &state,
                        &sbp_nodes.glo_biases);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_OBS,
                        (void *)&sbp2rtcm_sbp_obs_cb,
                        &state,
                        &sbp_nodes.obs);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_OSR,
                        (void *)&sbp2rtcm_sbp_osr_cb,
                        &state,
                        &sbp_nodes.osr);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_SSR_ORBIT_CLOCK,
                        (void *)&sbp2rtcm_sbp_ssr_orbit_clock_cb,
                        &state,
                        &sbp_nodes.ssr_orbit_clock);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_SSR_PHASE_BIASES,
                        (void *)&sbp2rtcm_sbp_ssr_phase_biases_cb,
                        &state,
                        &sbp_nodes.ssr_phase_biases);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_SSR_CODE_BIASES,
                        (void *)&sbp2rtcm_sbp_ssr_code_biases_cb,
                        &state,
                        &sbp_nodes.ssr_code_biases);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_SSR_GRIDDED_CORRECTION,
                        (void *)&sbp2rtcm_sbp_ssr_gridded_correction_cb,
                        &state,
                        &sbp_nodes.ssr_gridded_correction);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_SSR_GRID_DEFINITION,
                        (void *)&sbp2rtcm_sbp_ssr_grid_definition_cb,
                        &state,
                        &sbp_nodes.ssr_grid_definition);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_SSR_STEC_CORRECTION,
                        (void *)&sbp2rtcm_sbp_ssr_stec_correction_cb,
                        &state,
                        &sbp_nodes.ssr_stec_correction);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_EPHEMERIS_GPS,
                        (void *)&sbp2rtcm_sbp_gps_eph_cb,
                        &state,
                        &sbp_nodes.ephemeris_gps);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_EPHEMERIS_GLO,
                        (void *)&sbp2rtcm_sbp_glo_eph_cb,
                        &state,
                        &sbp_nodes.ephemeris_glo);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_EPHEMERIS_BDS,
                        (void *)&sbp2rtcm_sbp_bds_eph_cb,
                        &state,
                        &sbp_nodes.ephemeris_bds);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_EPHEMERIS_GAL,
                        (void *)&sbp2rtcm_sbp_gal_eph_cb,
                        &state,
                        &sbp_nodes.ephemeris_gal);
  sbp_register_callback(&sbp_state,
                        SBP_MSG_LOG,
                        (void *)&sbp2rtcm_sbp_log_cb,
                        &state,
                        &sbp_nodes.log);

  while (!feof(stdin)) {
    sbp_process(&sbp_state, &sbp_read_stdin);
  }
  return 0;
}
