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

#include <gnss-converters-extra/sbp_conv.h>

#include <assert.h>
#include <libsbp/logging.h>
#include <libsbp/sbp.h>
#include <swiftnav/fifo_byte.h>
#include <swiftnav/gnss_time.h>
#include <time.h>

#include <gnss-converters/sbp_rtcm3.h>

struct sbp_conv_s {
  struct rtcm3_out_state state;
  fifo_t fifo;
  uint8_t buf[2048];
};

static s32 sbp_conv_cb(uint8_t *buf, uint16_t len, void *context) {
  fifo_t *fifo = context;
  assert(fifo != NULL);
  return fifo_write(fifo, buf, len);
}

sbp_conv_t sbp_conv_new() {
  sbp_conv_t conv = malloc(sizeof(struct sbp_conv_s));
  if (conv != NULL) {
    fifo_init(&conv->fifo, conv->buf, sizeof(conv->buf));
    sbp2rtcm_init(&conv->state, sbp_conv_cb, &conv->fifo);
    gps_time_t gps_time = time2gps_t(time(NULL));
    sbp2rtcm_set_leap_second((s8)rint(get_gps_utc_offset(&gps_time, NULL)),
                             &conv->state);
    sbp2rtcm_set_rcv_ant_descriptors(
        "NULL                ", "SWFT", &conv->state);
  }
  return conv;
}

void sbp_conv_delete(sbp_conv_t conv) { free(conv); }

size_t sbp_conv(sbp_conv_t conv,
                uint16_t sender,
                uint16_t type,
                uint8_t *rbuf,
                size_t rlen,
                uint8_t *wbuf,
                size_t wlen) {
  switch (type) {
    case SBP_MSG_BASE_POS_ECEF: {
      sbp2rtcm_base_pos_ecef_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_OBS: {
      sbp2rtcm_sbp_obs_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_OSR: {
      sbp2rtcm_sbp_osr_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_SSR_ORBIT_CLOCK: {
      sbp2rtcm_sbp_ssr_orbit_clock_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_SSR_PHASE_BIASES: {
      sbp2rtcm_sbp_ssr_phase_biases_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_SSR_CODE_BIASES: {
      sbp2rtcm_sbp_ssr_code_biases_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_SSR_GRIDDED_CORRECTION: {
      sbp2rtcm_sbp_ssr_gridded_correction_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_SSR_GRID_DEFINITION: {
      sbp2rtcm_sbp_ssr_grid_definition_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_SSR_STEC_CORRECTION: {
      sbp2rtcm_sbp_ssr_stec_correction_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_EPHEMERIS_GPS: {
      sbp2rtcm_sbp_gps_eph_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_EPHEMERIS_GLO: {
      sbp2rtcm_sbp_glo_eph_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_EPHEMERIS_BDS: {
      sbp2rtcm_sbp_bds_eph_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_EPHEMERIS_GAL: {
      sbp2rtcm_sbp_gal_eph_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    case SBP_MSG_LOG: {
      sbp2rtcm_sbp_log_cb(sender, rlen, rbuf, &conv->state);
      break;
    }
    default: { break; }
  }
  return fifo_read(&conv->fifo, wbuf, wlen);
}
