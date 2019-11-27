/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>

#include <gnss-converters/ubx_sbp.h>

sbp_state_t sbp_state;

static int read_stdin(uint8_t *buf, size_t len, void *context) {
  (void)context;
  return read(STDIN_FILENO, buf, len);
}

static s32 write_sbp_stdout(u8 *buff, u32 n, void *context) {
  (void)context;
  return write(STDOUT_FILENO, buff, sizeof(u8) * n);
}

static void sbp_write(
    u16 msg_id, u8 length, u8 *buf, u16 sender_id, void *ctx) {
  (void)ctx;
  sbp_send_message(
      &sbp_state, msg_id, sender_id, length, buf, write_sbp_stdout);
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  /* TODO(STAR-917) accept sender id as a cmdline argument */

  sbp_state_init(&sbp_state);

  struct ubx_sbp_state state;

  ubx_sbp_init(&state, &sbp_write, NULL);

  int ret;
  do {
    ret = ubx_sbp_process(&state, &read_stdin);
  } while (ret >= 0);

  return 0;
}
