/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_UBX_SBP_INTERFACE_H
#define GNSS_CONVERTERS_UBX_SBP_INTERFACE_H

#include <libsbp/gnss.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <unistd.h>

#include "swiftnav/signal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UBX_BUFFER_SIZE 4096
#define UBX_FRAME_SIZE 4096

#define DEFAULT_UBX_SENDER_ID 61568

struct sat_data {
  union {
    struct subframe {
      u32 words[10];
    } sf[3];
    struct page {
      u32 words[8];
    } pg[5];
  };
  unsigned vmask;
};

/* Stores state for ESF-* messages */
struct ubx_esf_state {
  double time_since_startup_tow_offset;
  bool tow_offset_set;
  u32 last_sync_msss;
  u32 imu_raw_msgs_sent;
  double last_imu_temp;
};

struct ubx_sbp_state {
  u8 read_buffer[UBX_BUFFER_SIZE];
  size_t index;
  size_t bytes_in_buffer;
  u16 sender_id;
  int (*read_stream_func)(u8 *buf, size_t len, void *ctx);
  void (*cb_ubx_to_sbp)(
      u16 msg_id, u8 length, u8 *buf, u16 sender_id, void *ctx);
  void *context;
  bool use_hnr;
  struct ubx_esf_state esf_state;

  struct sat_data gps_sat[NUM_SATS_GPS];
  struct sat_data bds_sat[NUM_SATS_BDS];
  struct sat_data gal_sat[NUM_SATS_GAL];
};

double ubx_convert_msss_to_tow(u32 msss, const struct ubx_esf_state *state);

void ubx_sbp_init(struct ubx_sbp_state *state,
                  void (*cb_ubx_to_sbp)(u16 msg_id,
                                        u8 length,
                                        u8 *buff,
                                        u16 sender_id,
                                        void *context),
                  void *context);
void ubx_handle_frame(u8 *frame, struct ubx_sbp_state *state);
void ubx_set_sender_id(struct ubx_sbp_state *state, u16 sender_id);
void ubx_set_hnr_flag(struct ubx_sbp_state *state, bool use_hnr);
int ubx_sbp_process(struct ubx_sbp_state *state,
                    int (*read_stream_func)(u8 *buff, size_t len, void *ctx));

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_UBX_SBP_INTERFACE_H */
