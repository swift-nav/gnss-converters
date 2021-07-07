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
#include <libsbp/v4/ssr.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "check_rtcm3.h"
#include "config.h"

static bool gps_orbit_clock_processed = false;
static bool gps_separate_orbit_clock_processed = false;
static bool gps_code_bias_processed = false;
static bool gps_phase_bias_processed = false;
static bool glo_orbit_clock_processed = false;
static bool glo_separate_orbit_clock_processed = false;
static bool glo_code_bias_processed = false;
static bool gal_orbit_clock_processed = false;
static bool gal_separate_orbit_clock_processed = false;
static bool gal_code_bias_processed = false;
static bool gal_phase_bias_processed = false;
static bool bds_orbit_clock_processed = false;
static bool bds_code_bias_processed = false;
static bool bds_phase_bias_processed = false;

void sbp_callback_gps_orbit_clock(uint16_t sender_id,
                                  sbp_msg_type_t msg_type,
                                  const sbp_msg_t *sbp_msg,
                                  void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrOrbitClock && !msg_checked) {
    const sbp_msg_ssr_orbit_clock_t *sbp_orbit_clock =
        &sbp_msg->ssr_orbit_clock;
    if (sbp_orbit_clock->sid.code != CODE_GPS_L1CA) {
      // This is not a GPS message, wait for first GPS message
      return;
    }
    msg_checked = true;
    gps_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 171680);
    ck_assert(sbp_orbit_clock->sid.sat == 1);
    ck_assert(sbp_orbit_clock->sid.code == 0);
    ck_assert(sbp_orbit_clock->update_interval == 2);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 30);
    ck_assert(sbp_orbit_clock->radial == 4432);
    ck_assert(sbp_orbit_clock->along == -559);
    ck_assert(sbp_orbit_clock->cross == 1394);
    ck_assert(sbp_orbit_clock->dot_radial == 14);
    ck_assert(sbp_orbit_clock->dot_along == 18);
    ck_assert(sbp_orbit_clock->dot_cross == -22);
    ck_assert(sbp_orbit_clock->c0 == -268);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_gps_separate_orbit_clock(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrOrbitClock && !msg_checked) {
    const sbp_msg_ssr_orbit_clock_t *sbp_orbit_clock =
        &sbp_msg->ssr_orbit_clock;
    if (sbp_orbit_clock->sid.code != CODE_GPS_L1CA) {
      // This is not a GPS message, wait for first GPS message
      return;
    }
    msg_checked = true;
    gps_separate_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 513478);
    printf("sat: %d\n", sbp_orbit_clock->sid.sat);
    ck_assert(sbp_orbit_clock->sid.sat == 2);
    ck_assert(sbp_orbit_clock->sid.code == 0);
    ck_assert(sbp_orbit_clock->update_interval == 0);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 46);
    ck_assert(sbp_orbit_clock->radial == -173824);
    ck_assert(sbp_orbit_clock->along == -23407);
    ck_assert(sbp_orbit_clock->cross == -4749);
    ck_assert(sbp_orbit_clock->dot_radial == 0);
    ck_assert(sbp_orbit_clock->dot_along == 0);
    ck_assert(sbp_orbit_clock->dot_cross == 0);
    ck_assert(sbp_orbit_clock->c0 == 10278);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_gps_code_bias(uint16_t sender_id,
                                sbp_msg_type_t msg_type,
                                const sbp_msg_t *sbp_msg,
                                void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrCodeBiases && !msg_checked) {
    const sbp_msg_ssr_code_biases_t *sbp_code_bias = &sbp_msg->ssr_code_biases;
    if (sbp_code_bias->sid.code != CODE_GPS_L1CA) {
      // This is not a GPS message, wait for first GPS message
      return;
    }
    msg_checked = true;
    gps_code_bias_processed = true;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 1);
    ck_assert(sbp_code_bias->sid.code == 0);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);

    ck_assert(sbp_code_bias->biases[0].value == -340);
    ck_assert(sbp_code_bias->biases[0].code == 0);
    ck_assert(sbp_code_bias->biases[1].value == -368);
    ck_assert(sbp_code_bias->biases[1].code == 2);
    ck_assert(sbp_code_bias->biases[2].value == -338);
    ck_assert(sbp_code_bias->biases[2].code == 19);
    ck_assert(sbp_code_bias->biases[3].value == -567);
    ck_assert(sbp_code_bias->biases[3].code == 5);
    ck_assert(sbp_code_bias->biases[4].value == -566);
    ck_assert(sbp_code_bias->biases[4].code == 8);
    ck_assert(sbp_code_bias->biases[5].value == -563);
    ck_assert(sbp_code_bias->biases[5].code == 7);
    ck_assert(sbp_code_bias->biases[6].value == -607);
    ck_assert(sbp_code_bias->biases[6].code == 11);
    ck_assert(sbp_code_bias->biases[7].value == -567);
    ck_assert(sbp_code_bias->biases[7].code == 9);
    ck_assert(sbp_code_bias->biases[8].value == -315);
    ck_assert(sbp_code_bias->biases[8].code == 15);
    ck_assert(sbp_code_bias->biases[9].value == -306);
    ck_assert(sbp_code_bias->biases[9].code == 16);
  }
}

void sbp_callback_gps_phase_bias(uint16_t sender_id,
                                 sbp_msg_type_t msg_type,
                                 const sbp_msg_t *sbp_msg,
                                 void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrPhaseBiases && !msg_checked) {
    const sbp_msg_ssr_phase_biases_t *sbp_phase_bias =
        &sbp_msg->ssr_phase_biases;
    if (sbp_phase_bias->sid.code != CODE_GPS_L1CA) {
      // This is not a GPS message, wait for first GPS message
      return;
    }
    msg_checked = true;
    gps_phase_bias_processed = true;

    ck_assert(sbp_phase_bias->time.wn == 2013);
    ck_assert(sbp_phase_bias->time.tow == 171680);
    ck_assert(sbp_phase_bias->sid.sat == 1);
    ck_assert(sbp_phase_bias->sid.code == 0);
    ck_assert(sbp_phase_bias->update_interval == 2);
    ck_assert(sbp_phase_bias->iod_ssr == 0);
    ck_assert(sbp_phase_bias->dispersive_bias == 0);
    ck_assert(sbp_phase_bias->mw_consistency == 1);
    ck_assert(sbp_phase_bias->yaw == 41);
    ck_assert(sbp_phase_bias->yaw_rate == 0);

    ck_assert(sbp_phase_bias->biases[0].discontinuity_counter == 14);
    ck_assert(sbp_phase_bias->biases[0].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[0].bias == 6159);
    ck_assert(sbp_phase_bias->biases[0].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[0].code == 0);

    ck_assert(sbp_phase_bias->biases[1].discontinuity_counter == 14);
    ck_assert(sbp_phase_bias->biases[1].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[1].bias == 8922);
    ck_assert(sbp_phase_bias->biases[1].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[1].code == 11);

    ck_assert(sbp_phase_bias->biases[2].discontinuity_counter == 14);
    ck_assert(sbp_phase_bias->biases[2].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[2].bias == 9542);
    ck_assert(sbp_phase_bias->biases[2].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[2].code == 14);
  }
}

void sbp_callback_glo_orbit_clock(uint16_t sender_id,
                                  sbp_msg_type_t msg_type,
                                  const sbp_msg_t *sbp_msg,
                                  void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrOrbitClock && !msg_checked) {
    const sbp_msg_ssr_orbit_clock_t *sbp_orbit_clock =
        &sbp_msg->ssr_orbit_clock;
    if (sbp_orbit_clock->sid.code != CODE_GLO_L1OF) {
      // This is not a GLO message, wait for first GLO message
      return;
    }
    msg_checked = true;
    glo_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 171680);
    ck_assert(sbp_orbit_clock->sid.sat == 1);
    ck_assert(sbp_orbit_clock->sid.code == 3);
    ck_assert(sbp_orbit_clock->update_interval == 2);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 11);
    ck_assert(sbp_orbit_clock->radial == 2170);
    ck_assert(sbp_orbit_clock->along == 1621);
    ck_assert(sbp_orbit_clock->cross == 1805);
    ck_assert(sbp_orbit_clock->dot_radial == -831);
    ck_assert(sbp_orbit_clock->dot_along == 96);
    ck_assert(sbp_orbit_clock->dot_cross == -19);
    ck_assert(sbp_orbit_clock->c0 == 20868);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_glo_separate_orbit_clock(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrOrbitClock && !msg_checked) {
    const sbp_msg_ssr_orbit_clock_t *sbp_orbit_clock =
        &sbp_msg->ssr_orbit_clock;
    if (sbp_orbit_clock->sid.code != CODE_GLO_L1OF) {
      // This is not a GLO message, wait for first GLO message
      return;
    }
    msg_checked = true;
    glo_separate_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 167896);
    ck_assert(sbp_orbit_clock->sid.sat == 3);
    ck_assert(sbp_orbit_clock->sid.code == 3);
    ck_assert(sbp_orbit_clock->update_interval == 0);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 7);
    ck_assert(sbp_orbit_clock->radial == -14969);
    ck_assert(sbp_orbit_clock->along == -25802);
    ck_assert(sbp_orbit_clock->cross == 30707);
    ck_assert(sbp_orbit_clock->dot_radial == 0);
    ck_assert(sbp_orbit_clock->dot_along == 0);
    ck_assert(sbp_orbit_clock->dot_cross == 0);
    ck_assert(sbp_orbit_clock->c0 == 2694);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_glo_code_bias(uint16_t sender_id,
                                sbp_msg_type_t msg_type,
                                const sbp_msg_t *sbp_msg,
                                void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrCodeBiases && !msg_checked) {
    const sbp_msg_ssr_code_biases_t *sbp_code_bias = &sbp_msg->ssr_code_biases;
    if (sbp_code_bias->sid.code != CODE_GLO_L1OF) {
      // This is not a GLO message, wait for first GLO message
      return;
    }
    msg_checked = true;
    glo_code_bias_processed = true;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 1);
    ck_assert(sbp_code_bias->sid.code == 3);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);

    ck_assert(sbp_code_bias->biases[0].value == -260);
    ck_assert(sbp_code_bias->biases[0].code == 0);
    ck_assert(sbp_code_bias->biases[1].value == -257);
    ck_assert(sbp_code_bias->biases[1].code == 1);
    ck_assert(sbp_code_bias->biases[2].value == -350);
    ck_assert(sbp_code_bias->biases[2].code == 2);
    ck_assert(sbp_code_bias->biases[3].value == -424);
    ck_assert(sbp_code_bias->biases[3].code == 3);
  }
}

void sbp_callback_gal_orbit_clock(uint16_t sender_id,
                                  sbp_msg_type_t msg_type,
                                  const sbp_msg_t *sbp_msg,
                                  void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrOrbitClock && !msg_checked) {
    const sbp_msg_ssr_orbit_clock_t *sbp_orbit_clock =
        &sbp_msg->ssr_orbit_clock;
    if (sbp_orbit_clock->sid.code != CODE_GAL_E1B) {
      // This is not a GAL message, wait for first GAL message
      return;
    }
    msg_checked = true;
    gal_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 171680);
    ck_assert(sbp_orbit_clock->sid.sat == 1);
    ck_assert(sbp_orbit_clock->sid.code == CODE_GAL_E1B);
    ck_assert(sbp_orbit_clock->update_interval == 2);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 26);
    ck_assert(sbp_orbit_clock->radial == -1887);
    ck_assert(sbp_orbit_clock->along == 943);
    ck_assert(sbp_orbit_clock->cross == 390);
    ck_assert(sbp_orbit_clock->dot_radial == -6);
    ck_assert(sbp_orbit_clock->dot_along == -22);
    ck_assert(sbp_orbit_clock->dot_cross == 4);
    ck_assert(sbp_orbit_clock->c0 == 714);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_gal_separate_orbit_clock(uint16_t sender_id,
                                           sbp_msg_type_t msg_type,
                                           const sbp_msg_t *sbp_msg,
                                           void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrOrbitClock && !msg_checked) {
    const sbp_msg_ssr_orbit_clock_t *sbp_orbit_clock =
        &sbp_msg->ssr_orbit_clock;
    if (sbp_orbit_clock->sid.code != CODE_GAL_E1B) {
      // This is not a GAL message, wait for first GAL message
      return;
    }
    msg_checked = true;
    gal_separate_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 513478);
    ck_assert(sbp_orbit_clock->sid.sat == 1);
    ck_assert(sbp_orbit_clock->sid.code == CODE_GAL_E1B);
    ck_assert(sbp_orbit_clock->update_interval == 0);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 86);
    ck_assert(sbp_orbit_clock->radial == 238457);
    ck_assert(sbp_orbit_clock->along == -29239);
    ck_assert(sbp_orbit_clock->cross == 33418);
    ck_assert(sbp_orbit_clock->dot_radial == 0);
    ck_assert(sbp_orbit_clock->dot_along == 0);
    ck_assert(sbp_orbit_clock->dot_cross == 0);
    ck_assert(sbp_orbit_clock->c0 == -233944);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_gal_code_bias(uint16_t sender_id,
                                sbp_msg_type_t msg_type,
                                const sbp_msg_t *sbp_msg,
                                void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrCodeBiases && !msg_checked) {
    const sbp_msg_ssr_code_biases_t *sbp_code_bias = &sbp_msg->ssr_code_biases;
    if (sbp_code_bias->sid.code != CODE_GAL_E1B) {
      // This is not a GAL message, wait for first GAL message
      return;
    }
    msg_checked = true;
    gal_code_bias_processed = true;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 1);
    ck_assert(sbp_code_bias->sid.code == CODE_GAL_E1B);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);

    ck_assert(sbp_code_bias->biases[0].value == 143);
    ck_assert(sbp_code_bias->biases[0].code == 3);
    ck_assert(sbp_code_bias->biases[1].value == 257);
    ck_assert(sbp_code_bias->biases[1].code == 7);
    ck_assert(sbp_code_bias->biases[2].value == 265);
    ck_assert(sbp_code_bias->biases[2].code == 10);
    ck_assert(sbp_code_bias->biases[3].value == 262);
    ck_assert(sbp_code_bias->biases[3].code == 13);
  }
}

void sbp_callback_gal_phase_bias(uint16_t sender_id,
                                 sbp_msg_type_t msg_type,
                                 const sbp_msg_t *sbp_msg,
                                 void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrPhaseBiases && !msg_checked) {
    const sbp_msg_ssr_phase_biases_t *sbp_phase_bias =
        &sbp_msg->ssr_phase_biases;
    if (sbp_phase_bias->sid.code != CODE_GAL_E1B) {
      // This is not a GAL message, wait for first GAL message
      return;
    }
    msg_checked = true;
    gal_phase_bias_processed = true;

    ck_assert(sbp_phase_bias->time.wn == 2013);
    ck_assert(sbp_phase_bias->time.tow == 171680);
    ck_assert(sbp_phase_bias->sid.sat == 1);
    ck_assert(sbp_phase_bias->sid.code == 14);
    ck_assert(sbp_phase_bias->update_interval == 2);
    ck_assert(sbp_phase_bias->iod_ssr == 0);
    ck_assert(sbp_phase_bias->dispersive_bias == 0);
    ck_assert(sbp_phase_bias->mw_consistency == 1);
    ck_assert(sbp_phase_bias->yaw == 461);
    ck_assert(sbp_phase_bias->yaw_rate == 0);

    ck_assert(sbp_phase_bias->biases[0].discontinuity_counter == 10);
    ck_assert(sbp_phase_bias->biases[0].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[0].bias == -8261);
    ck_assert(sbp_phase_bias->biases[0].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[0].code == 3);

    ck_assert(sbp_phase_bias->biases[1].discontinuity_counter == 10);
    ck_assert(sbp_phase_bias->biases[1].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[1].bias == -10853);
    ck_assert(sbp_phase_bias->biases[1].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[1].code == 7);

    ck_assert(sbp_phase_bias->biases[2].discontinuity_counter == 10);
    ck_assert(sbp_phase_bias->biases[2].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[2].bias == -10494);
    ck_assert(sbp_phase_bias->biases[2].integer_indicator == 1);
    ck_assert(sbp_phase_bias->biases[2].code == 10);
  }
}

void sbp_callback_bds_orbit_clock(uint16_t sender_id,
                                  sbp_msg_type_t msg_type,
                                  const sbp_msg_t *sbp_msg,
                                  void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrOrbitClock && !msg_checked) {
    const sbp_msg_ssr_orbit_clock_t *sbp_orbit_clock =
        &sbp_msg->ssr_orbit_clock;
    if (sbp_orbit_clock->sid.code != CODE_BDS2_B1) {
      // This is not a BDS message, wait for first BDS message
      return;
    }
    msg_checked = true;
    bds_orbit_clock_processed = true;

    ck_assert(sbp_orbit_clock->time.wn == 2013);
    ck_assert(sbp_orbit_clock->time.tow == 172805);
    ck_assert(sbp_orbit_clock->sid.sat == 2);
    ck_assert(sbp_orbit_clock->sid.code == CODE_BDS2_B1);
    ck_assert(sbp_orbit_clock->update_interval == 2);
    ck_assert(sbp_orbit_clock->iod_ssr == 0);
    ck_assert(sbp_orbit_clock->iod == 235);
    ck_assert(sbp_orbit_clock->radial == 47787);
    ck_assert(sbp_orbit_clock->along == -3652);
    ck_assert(sbp_orbit_clock->cross == -13030);
    ck_assert(sbp_orbit_clock->dot_radial == -58);
    ck_assert(sbp_orbit_clock->dot_along == -15);
    ck_assert(sbp_orbit_clock->dot_cross == -11);
    ck_assert(sbp_orbit_clock->c0 == -2083);
    ck_assert(sbp_orbit_clock->c1 == 0);
    ck_assert(sbp_orbit_clock->c2 == 0);
  }
}

void sbp_callback_bds_code_bias(uint16_t sender_id,
                                sbp_msg_type_t msg_type,
                                const sbp_msg_t *sbp_msg,
                                void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrCodeBiases && !msg_checked) {
    const sbp_msg_ssr_code_biases_t *sbp_code_bias = &sbp_msg->ssr_code_biases;
    if (sbp_code_bias->sid.code != CODE_BDS2_B1) {
      // This is not a BDS message, wait for first BDS message
      return;
    }
    msg_checked = true;
    bds_code_bias_processed = true;

    ck_assert(sbp_code_bias->time.wn == 2013);
    ck_assert(sbp_code_bias->time.tow == 171680);
    ck_assert(sbp_code_bias->sid.sat == 5);
    ck_assert(sbp_code_bias->sid.code == CODE_BDS2_B1);
    ck_assert(sbp_code_bias->update_interval == 2);
    ck_assert(sbp_code_bias->iod_ssr == 0);

    ck_assert(sbp_code_bias->biases[0].value == 32);
    ck_assert(sbp_code_bias->biases[0].code == 0);
    ck_assert(sbp_code_bias->biases[1].value == -103);
    ck_assert(sbp_code_bias->biases[1].code == 3);
    ck_assert(sbp_code_bias->biases[2].value == 53);
    ck_assert(sbp_code_bias->biases[2].code == 6);
  }
}

void sbp_callback_bds_phase_bias(uint16_t sender_id,
                                 sbp_msg_type_t msg_type,
                                 const sbp_msg_t *sbp_msg,
                                 void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  static bool msg_checked = false;
  if (msg_type == SbpMsgSsrPhaseBiases && !msg_checked) {
    const sbp_msg_ssr_phase_biases_t *sbp_phase_bias =
        &sbp_msg->ssr_phase_biases;
    if (sbp_phase_bias->sid.code != CODE_BDS2_B1) {
      // This is not a BDS message, wait for first BDS message
      return;
    }
    msg_checked = true;
    bds_phase_bias_processed = true;

    ck_assert(sbp_phase_bias->time.wn == 2013);
    ck_assert(sbp_phase_bias->time.tow == 171675);
    ck_assert(sbp_phase_bias->sid.sat == 5);
    ck_assert(sbp_phase_bias->sid.code == CODE_BDS2_B1);
    ck_assert(sbp_phase_bias->update_interval == 2);
    ck_assert(sbp_phase_bias->iod_ssr == 0);
    ck_assert(sbp_phase_bias->dispersive_bias == 0);
    ck_assert(sbp_phase_bias->mw_consistency == 1);
    ck_assert(sbp_phase_bias->yaw == 0);
    ck_assert(sbp_phase_bias->yaw_rate == 0);

    ck_assert(sbp_phase_bias->biases[0].discontinuity_counter == 0);
    ck_assert(sbp_phase_bias->biases[0].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[0].bias == 1382);
    ck_assert(sbp_phase_bias->biases[0].integer_indicator == 0);
    ck_assert(sbp_phase_bias->biases[0].code == 0);

    ck_assert(sbp_phase_bias->biases[1].discontinuity_counter == 0);
    ck_assert(sbp_phase_bias->biases[1].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[1].bias == 2311);
    ck_assert(sbp_phase_bias->biases[1].integer_indicator == 0);
    ck_assert(sbp_phase_bias->biases[1].code == 6);

    ck_assert(sbp_phase_bias->biases[2].discontinuity_counter == 0);
    ck_assert(sbp_phase_bias->biases[2].widelane_integer_indicator == 2);
    ck_assert(sbp_phase_bias->biases[2].bias == 1588);
    ck_assert(sbp_phase_bias->biases[2].integer_indicator == 0);
    ck_assert(sbp_phase_bias->biases[2].code == 3);
  }
}

START_TEST(test_ssr_gps_orbit_clock) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gps_orbit_clock,
             current_time);
  ck_assert(gps_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_gps_separate_orbit_clock) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/separate_clk_orbit.rtcm",
             sbp_callback_gps_separate_orbit_clock,
             current_time);
  ck_assert(gps_separate_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_gps_code_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gps_code_bias,
             current_time);
  ck_assert(gps_code_bias_processed);
}
END_TEST

START_TEST(test_ssr_gps_phase_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gps_phase_bias,
             current_time);
  ck_assert(gps_phase_bias_processed);
}
END_TEST

START_TEST(test_ssr_glo_orbit_clock) {
  current_time.wn = 2013;
  current_time.tow = 171680;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_glo_orbit_clock,
             current_time);
  ck_assert(glo_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_glo_separate_orbit_clock) {
  current_time.wn = 2013;
  current_time.tow = 171680;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/separate_clk_orbit.rtcm",
             sbp_callback_glo_separate_orbit_clock,
             current_time);
  ck_assert(glo_separate_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_glo_code_bias) {
  current_time.wn = 2013;
  current_time.tow = 171680;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_glo_code_bias,
             current_time);
  ck_assert(glo_code_bias_processed);
}
END_TEST

START_TEST(test_ssr_gal_orbit_clock) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gal_orbit_clock,
             current_time);
  ck_assert(gal_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_gal_separate_orbit_clock) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/separate_clk_orbit.rtcm",
             sbp_callback_gal_separate_orbit_clock,
             current_time);
  ck_assert(gal_separate_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_gal_code_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gal_code_bias,
             current_time);
  ck_assert(gal_code_bias_processed);
}
END_TEST

START_TEST(test_ssr_gal_phase_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_gal_phase_bias,
             current_time);
  ck_assert(gal_phase_bias_processed);
}
END_TEST

START_TEST(test_ssr_bds_orbit_clock) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/bds-clk.rtcm",
             sbp_callback_bds_orbit_clock,
             current_time);
  ck_assert(bds_orbit_clock_processed);
}
END_TEST

START_TEST(test_ssr_bds_code_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_bds_code_bias,
             current_time);
  ck_assert(bds_code_bias_processed);
}
END_TEST

START_TEST(test_ssr_bds_phase_bias) {
  current_time.wn = 2013;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/clk.rtcm",
             sbp_callback_bds_phase_bias,
             current_time);
  ck_assert(bds_phase_bias_processed);
}
END_TEST

typedef struct {
  u8 expected_sbp_message_lengths[5];
  size_t total_sbp_messages_sent;
} test_ssr_phase_bias_context_t;

void test_ssr_phase_bias_callback(uint16_t sender_id,
                                  sbp_msg_type_t msg_type,
                                  const sbp_msg_t *sbp_msg,
                                  void *context) {
  (void)msg_type;
  (void)sbp_msg;
  (void)sender_id;

  const sbp_msg_ssr_phase_biases_t *sbp_phase_bias = &sbp_msg->ssr_phase_biases;

  u8 length = sbp_phase_bias->n_biases;

  test_ssr_phase_bias_context_t *ctx = (test_ssr_phase_bias_context_t *)context;
  ck_assert_int_eq(
      ctx->expected_sbp_message_lengths[ctx->total_sbp_messages_sent++],
      length);
}

START_TEST(test_ssr_phase_bias) {
  const gps_time_t local_current_time = {.wn = 2013, .tow = 211190.0};

  const size_t max_bias_count =
      sizeof(((sbp_msg_ssr_phase_biases_t *)0)->biases) /
      sizeof(sbp_phase_biases_content_t);

  struct rtcm3_sbp_state state;
  test_ssr_phase_bias_context_t context;
  time_truth_t time_truth;

  time_truth_init(&time_truth);
  time_truth_update(&time_truth, TIME_TRUTH_EPH_GAL, local_current_time);

  rtcm2sbp_init(
      &state, &time_truth, test_ssr_phase_bias_callback, NULL, &context);
  state.time_from_input_data = local_current_time;

  rtcm_msg_phase_bias rtcm_msg = {
      .header.message_num = 1270,
      .header.epoch_time = 171661,
      .header.constellation = 3,
      .header.update_interval = 2,
      .header.multi_message = false,
      .header.sat_ref_datum = false,
      .header.iod_ssr = 0,
      .header.ssr_provider_id = 0,
      .header.ssr_solution_id = 0,
      .header.dispersive_bias_consistency = false,
      .header.melbourne_wubbena_consistency = true,
      .header.num_sats = 1,
  };

  /*********************************
   * TEST CASE #1: no phase biases *
   *********************************/

  rtcm_msg.sats[0].num_phase_biases = 0;

  context.total_sbp_messages_sent = 0;
  context.expected_sbp_message_lengths[0] = 0;

  rtcm3_ssr_phase_bias_to_sbp(&rtcm_msg, &state);
  ck_assert_int_eq(context.total_sbp_messages_sent, 1);

  /********************************************
   * TEST CASE #2: below max phase bias count *
   ********************************************/

  rtcm_msg.sats[0].num_phase_biases = max_bias_count - 1;

  context.total_sbp_messages_sent = 0;
  context.expected_sbp_message_lengths[0] = (max_bias_count - 1);

  rtcm3_ssr_phase_bias_to_sbp(&rtcm_msg, &state);
  ck_assert_int_eq(context.total_sbp_messages_sent, 1);

  /*****************************************
   * TEST CASE #3: at max phase bias count *
   *****************************************/

  rtcm_msg.sats[0].num_phase_biases = max_bias_count;

  context.total_sbp_messages_sent = 0;
  context.expected_sbp_message_lengths[0] = max_bias_count;

  rtcm3_ssr_phase_bias_to_sbp(&rtcm_msg, &state);
  ck_assert_int_eq(context.total_sbp_messages_sent, 1);

  /*********************************************
   * TEST CASE #4: beyond max phase bias count *
   *********************************************/

  rtcm_msg.sats[0].num_phase_biases = max_bias_count + 1;

  context.total_sbp_messages_sent = 0;
  context.expected_sbp_message_lengths[0] = max_bias_count;
  context.expected_sbp_message_lengths[1] = 1;

  rtcm3_ssr_phase_bias_to_sbp(&rtcm_msg, &state);
  ck_assert_int_eq(context.total_sbp_messages_sent, 2);
}
END_TEST

Suite *rtcm3_ssr_suite(void) {
  Suite *s = suite_create("RTCMv3_ssr");

  TCase *tc_ssr = tcase_create("SSR");
  tcase_add_checked_fixture(tc_ssr, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_ssr, test_ssr_gps_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_gps_separate_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_gps_code_bias);
  tcase_add_test(tc_ssr, test_ssr_gps_phase_bias);
  tcase_add_test(tc_ssr, test_ssr_glo_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_glo_separate_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_glo_code_bias);
  tcase_add_test(tc_ssr, test_ssr_gal_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_gal_separate_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_gal_code_bias);
  tcase_add_test(tc_ssr, test_ssr_gal_phase_bias);
  tcase_add_test(tc_ssr, test_ssr_bds_orbit_clock);
  tcase_add_test(tc_ssr, test_ssr_bds_code_bias);
  tcase_add_test(tc_ssr, test_ssr_bds_phase_bias);
  tcase_add_test(tc_ssr, test_ssr_phase_bias);
  suite_add_tcase(s, tc_ssr);

  return s;
}
