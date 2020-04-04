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

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static void help(char *arg) {
  fprintf(stderr, "Usage: %s [options]\n", arg);
  fprintf(stderr, "  -h this message\n");
  fprintf(stderr, "\n");
  fprintf(
      stderr,
      "  -s, --sender_id use provided sender id. Can be hex format if prefixed "
      "with '0x'. Defaults to %d\n",
      DEFAULT_UBX_SENDER_ID);
  fprintf(stderr,
          "  --hnr output POS_LLH (522) on HNR-PVT instead of NAV-PVT. NAV-PVT "
          "messages are still necessary.\n");
}

int ubx2sbp_main(int argc, char **argv) {
  /* TODO(STAR-917) accept sender id as a cmdline argument */

  sbp_state_init(&sbp_state);

  struct ubx_sbp_state state;
  ubx_sbp_init(&state, &sbp_write, NULL);

  int opt;
  int option_index = 0;
  static struct option long_options[] = {
      {"hnr", no_argument, 0, 0},
      {"sender_id", required_argument, 0, 's'},
      {0, 0, 0, 0}};

  while ((opt = getopt_long(argc, argv, "hs:", long_options, &option_index)) !=
         -1) {
    switch (opt) {
      case 0:
        if (strcmp("hnr", long_options[option_index].name) == 0) {
          ubx_set_hnr_flag(&state, true);
        }
        break;

      case 's':
        ubx_set_sender_id(&state, strtol(optarg, NULL, 0));
        break;

      case 'h':
        help(argv[0]);
        return 0;

      default:
        break;
    }
  }

  int ret;
  do {
    ret = ubx_sbp_process(&state, &read_stdin);
  } while (ret >= 0);

  return 0;
}
