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

#include <gnss-converters/ixcom_sbp.h>

sbp_state_t sbp_state;

typedef int (*readfn_ptr)(uint8_t *, size_t, void *);
typedef int (*writefn_ptr)(uint8_t *, uint32_t, void *);

sbp_write_fn_t g_writefn;

static void sbp_write(u16 sender_id,
                      sbp_msg_type_t msg_type,
                      const sbp_msg_t *msg,
                      void *context) {
  (void)context;
  sbp_message_send(&sbp_state, msg_type, sender_id, msg, g_writefn);
}

static void help(char *arg, const char *additional_opts_help) {
  fprintf(stderr, "Usage: %s [options]%s\n", arg, additional_opts_help);
  fprintf(stderr, "  -h, --help this message\n");
  fprintf(stderr, "\n");
  fprintf(
      stderr,
      "  -s, --sender_id use provided sender id. Can be hex format if prefixed "
      "with '0x'. Defaults to %d\n",
      DEFAULT_IXCOM_SENDER_ID);
}

int ixcom2sbp_main(int argc,
                   char **argv,
                   const char *additional_opts_help,
                   readfn_ptr readfn,
                   writefn_ptr writefn,
                   void *context) {
  g_writefn = writefn;

  sbp_state_init(&sbp_state);
  sbp_state_set_io_context(&sbp_state, context);

  struct ixcom_sbp_state state;
  ixcom_sbp_init(&state, &sbp_write, context);

  int opt;
  int option_index = 0;
  static struct option long_options[] = {
      {"sender_id", required_argument, 0, 's'},
      {"help", no_argument, 0, 'h'},
      {0, 0, 0, 0}};

  while ((opt = getopt_long(argc, argv, "hs:", long_options, &option_index)) !=
         -1) {
    switch (opt) {
      case 's':
        ixcom_set_sender_id(&state, (u16)strtol(optarg, NULL, 0));
        break;

      case 'h':
        help(argv[0], additional_opts_help);
        return 0;

      default:
        break;
    }
  }

  int ret;
  do {
    ret = ixcom_sbp_process(&state, readfn);
  } while (ret > 0);

  return 0;
}
