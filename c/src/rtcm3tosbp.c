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
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <gnss-converters/rtcm3_sbp.h>
#include <libsbp/edc.h>
#include <swiftnav/edc.h>
#include <swiftnav/fifo_byte.h>
#include <swiftnav/gnss_time.h>

/* You may reduce FIFO_SIZE if you need a lower memory footprint. */
#define FIFO_SIZE         (1 << 14)
#define BUFFER_SIZE       (FIFO_SIZE - RTCM3_MSG_OVERHEAD - RTCM3_MAX_MSG_LEN)
#define SBP_PREAMBLE      0x55

static struct rtcm3_sbp_state state;

static void update_obs_time(const msg_obs_t *msg) {
  gps_time_sec_t obs_time;
  obs_time.tow = msg[0].header.t.tow / 1000.0; /* ms to sec */
  obs_time.wn = msg[0].header.t.wn;
  rtcm2sbp_set_gps_time(&obs_time, &state);
}

/* Write the SBP packet to STDOUT. In theory, I could use
   sbp_send_message(). */
static void cb_rtcm_to_sbp(uint16_t msg_id, uint8_t length, uint8_t *buffer,
                           uint16_t sender_id, void *context)
{

  if (msg_id == SBP_MSG_OBS) {
    update_obs_time((msg_obs_t *)buffer);
  }

  (void)(context);              /* squash warning */
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
  u16 crc = crc16_ccitt(tmpbuf + 1 , sizeof(tmpbuf) - 1, 0);
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

static void cb_base_obs_invalid(const double timediff, void *context)
{
  (void)context;                /* squash warning */
  fprintf(stderr, "Invalid base observation! timediff: %lf\n", timediff);
}

static uint16_t extract_msg_len(uint8_t *buf)
{
  return (((uint16_t)buf[1] << 8) | buf[2]) & RTCM3_MAX_MSG_LEN;
}

/* buf_len is your total allocated space - can be much bigger than
   your actual message */
static bool verify_crc(uint8_t *buf, uint16_t buf_len) {
  uint16_t msg_len = extract_msg_len(buf);
  assert(buf_len >= msg_len + RTCM3_MSG_OVERHEAD);
  uint32_t computed_crc = crc24q(buf, 3 + msg_len, 0);
  uint32_t frame_crc = (buf[msg_len + 3] << 16) |
                       (buf[msg_len + 4] << 8) |
                       (buf[msg_len + 5] << 0);
  if (frame_crc != computed_crc) {
    fprintf(stderr, "CRC failure! frame: %08X computed: %08X\n", frame_crc,
            computed_crc);
  }
  return (frame_crc == computed_crc);
}

int main(int argc, char **argv)
{
  (void)(argc);                 /* todo: accept arguments */
  (void)(argv);                 /* todo: accept arguments */
  assert(FIFO_SIZE > RTCM3_MSG_OVERHEAD + RTCM3_MAX_MSG_LEN);

  /* set time from systime, account for UTC<->GPS leap second difference */
  time_t ct_utc_unix = time(NULL);
  gps_time_t noleapsec = time2gps_t(ct_utc_unix);
  ct_utc_unix += get_gps_utc_offset(&noleapsec, NULL);
  gps_time_t withleapsec = time2gps_t(ct_utc_unix);
  gps_time_sec_t current_time;
  current_time.tow = withleapsec.tow;
  current_time.wn = withleapsec.wn;

  rtcm2sbp_init(&state, cb_rtcm_to_sbp, cb_base_obs_invalid, NULL);
  rtcm2sbp_set_gps_time(&current_time, &state);
  // rtcm2sbp_set_leap_second(18, &state); /* todo: worry about GLONASS later */

  uint8_t fifo_buf[FIFO_SIZE] = {0};
  fifo_t fifo;
  fifo_init(&fifo, fifo_buf, sizeof(fifo_buf));

  uint8_t inbuf[BUFFER_SIZE];
  ssize_t numread;
  while ((numread = read(STDIN_FILENO, inbuf, BUFFER_SIZE)) > 0) {
    ssize_t numwritten = fifo_write(&fifo, inbuf, numread);
    assert(numwritten == numread);

    uint8_t buf[FIFO_SIZE] = {0};
    fifo_size_t bytes_avail = fifo_peek(&fifo, buf, FIFO_SIZE);
    uint32_t index = 0;

    while (index + RTCM3_MSG_OVERHEAD < bytes_avail) {
      if (RTCM3_PREAMBLE != buf[index]) {
        index++;
        continue;
      }

      uint16_t msg_len = extract_msg_len(&buf[index]);
      if ((msg_len == 0) || (msg_len > RTCM3_MAX_MSG_LEN)) {
        index++;
        continue;
      }
      if (index + RTCM3_MSG_OVERHEAD + msg_len > bytes_avail) {
        break;
      }

      uint8_t *rtcm_msg = &buf[index];
      if (!verify_crc(rtcm_msg, FIFO_SIZE - index)) {
        index++;
        continue;
      }

      rtcm2sbp_decode_frame(rtcm_msg, msg_len + RTCM3_MSG_OVERHEAD, &state);
      index += msg_len + RTCM3_MSG_OVERHEAD;
    }
    fifo_size_t numremoved = fifo_remove(&fifo, index);
    assert(numremoved == index);

  }
  return 0;
}
