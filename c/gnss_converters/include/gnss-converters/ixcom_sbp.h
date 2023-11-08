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

#ifndef GNSS_CONVERTERS_IXCOM_SBP_INTERFACE_H
#define GNSS_CONVERTERS_IXCOM_SBP_INTERFACE_H

#include <ixcom/messages.h>
#include <libsbp/sbp.h>
#include <stddef.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IXCOM_BUFFER_SIZE 4096
#define IXCOM_FRAME_SIZE 4096

#define DEFAULT_IXCOM_SENDER_ID 4711

struct ixcom_sbp_state {
  u8 read_buffer[IXCOM_BUFFER_SIZE];
  size_t index;
  u16 sender_id;
  int (*read_stream_func)(u8 *buf, size_t len, void *context);
  void (*cb_ixcom_to_sbp)(u16 sender_id,
                          sbp_msg_type_t msg_type,
                          const sbp_msg_t *msg,
                          void *context);
  void *context;
  u8 imu_raw_msgs_sent;
};

void ixcom_sbp_init(struct ixcom_sbp_state *state,
                    void (*cb_ixcom_to_sbp)(u16 sender_id,
                                            sbp_msg_type_t msg_type,
                                            const sbp_msg_t *msg,
                                            void *context),
                    void *context);
void ixcom_handle_frame(struct ixcom_sbp_state *state);
void ixcom_set_sender_id(struct ixcom_sbp_state *state, u16 sender_id);
int ixcom_sbp_process(struct ixcom_sbp_state *state,
                      int (*read_stream_func)(u8 *buff,
                                              size_t len,
                                              void *context));

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_IXCOM_SBP_INTERFACE_H */
