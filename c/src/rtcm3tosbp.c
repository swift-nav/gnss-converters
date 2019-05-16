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

/* This is a stand-alone tool that takes RTCM3 on stdin and writes
   Swift Binary Protocol (SBP) on stdout.  It is designed to perform
   well in embedded and cloud environments; likewise for live
   vs. pre-recorded data.  Note that by default it sets the time to
   the current system time, which may not be suitable for pre-recorded
   data.  */
#include <assert.h>
#include <gnss-converters/rtcm3_sbp.h>
#include <libsbp/edc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <swiftnav/edc.h>
#include <swiftnav/gnss_time.h>
#include <time.h>
#include <unistd.h>

#define SBP_PREAMBLE 0x55

static struct rtcm3_sbp_state state;

static void update_obs_time(const msg_obs_t *msg) {
  gps_time_t obs_time;
  obs_time.tow = msg[0].header.t.tow / 1000.0; /* ms to sec */
  obs_time.wn = msg[0].header.t.wn;
  rtcm2sbp_set_gps_time(&obs_time, &state);
}

/* Write the SBP packet to STDOUT. In theory, I could use
   sbp_send_message(). */
static void cb_rtcm_to_sbp(uint16_t msg_id,
                           uint8_t length,
                           uint8_t *buffer,
                           uint16_t sender_id,
                           void *context) {
  if (msg_id == SBP_MSG_OBS) {
    update_obs_time((msg_obs_t *)buffer);
  }

  (void)(context); /* squash warning */
  /* SBP specifies little endian; this code should work on all hosts */
  uint8_t tmpbuf[6];
  tmpbuf[0] = SBP_PREAMBLE;
  tmpbuf[1] = (uint8_t)msg_id;
  tmpbuf[2] = (uint8_t)(msg_id >> 8);
  tmpbuf[3] = (uint8_t)sender_id;
  tmpbuf[4] = (uint8_t)(sender_id >> 8);
  tmpbuf[5] = length;
  ssize_t numwritten = write(STDOUT_FILENO, tmpbuf, sizeof(tmpbuf));
  if (numwritten < (ssize_t)sizeof(tmpbuf)) {
    fprintf(stderr, "Write failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }

  numwritten = write(STDOUT_FILENO, buffer, length);
  if (numwritten < length) {
    fprintf(stderr, "Write failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }

  /* CRC does not cover preamble */
  u16 crc = crc16_ccitt(tmpbuf + 1, sizeof(tmpbuf) - 1, 0);
  crc = crc16_ccitt(buffer, length, crc);
  uint8_t crcbuf[2];
  crcbuf[0] = (uint8_t)crc;
  crcbuf[1] = (uint8_t)(crc >> 8);
  numwritten = write(STDOUT_FILENO, crcbuf, sizeof(crcbuf));
  if (numwritten < (ssize_t)sizeof(crcbuf)) {
    fprintf(stderr, "Write failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }
}

static void cb_base_obs_invalid(const double timediff, void *context) {
  (void)context; /* squash warning */
  fprintf(stderr, "Invalid base observation! timediff: %lf\n", timediff);
}

static int read_stdin(uint8_t *buf, size_t len, void *context) {
  (void)context; /* squash warning */
  return read(STDIN_FILENO, buf, len);
}

int main(int argc, char **argv) {
  (void)(argc); /* todo: accept arguments */
  (void)(argv); /* todo: accept arguments */

  /* set time from systime, account for UTC<->GPS leap second difference */
  time_t ct_utc_unix = time(NULL);
  gps_time_t noleapsec = time2gps_t(ct_utc_unix);
  double gps_utc_offset = get_gps_utc_offset(&noleapsec, NULL);
  ct_utc_unix += (s8)rint(gps_utc_offset);
  gps_time_t withleapsec = time2gps_t(ct_utc_unix);
  gps_time_t current_time;
  current_time.tow = withleapsec.tow;
  current_time.wn = withleapsec.wn;

  rtcm2sbp_init(&state, cb_rtcm_to_sbp, cb_base_obs_invalid, NULL);
  rtcm2sbp_set_gps_time(&current_time, &state);
  rtcm2sbp_set_leap_second((s8)rint(gps_utc_offset), &state);

  /* todo: Do we want to return a non-zero value on an error? */
  rtcm2sbp_process_stream(&state, read_stdin);

  return 0;
}
