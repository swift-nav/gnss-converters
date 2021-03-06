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
#include <getopt.h>
#include <gnss-converters/options.h>
#include <gnss-converters/rtcm3_sbp.h>
#include <libsbp/edc.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/edc.h>
#include <swiftnav/gnss_time.h>
#include <time.h>
#include <unistd.h>

#define SBP_PREAMBLE 0x55
#define STRCMP_EQ 0

static time_truth_t time_truth;

typedef int (*readfn_ptr)(uint8_t *, size_t, void *);
typedef int (*writefn_ptr)(const uint8_t *, size_t, void *);

writefn_ptr g_writefn;

static struct rtcm3_sbp_state state;
static void parse_biases(char *arg);
static void parse_glonass_code_biases(char *arg);
static void parse_glonass_phase_biases(char *arg);

static void update_obs_time(const msg_obs_t *msg) {
  gps_time_t obs_time;
  obs_time.tow = msg[0].header.t.tow / 1000.0; /* ms to sec */
  obs_time.wn = (s16)msg[0].header.t.wn;
  /* Some receivers output a TOW 0 whenever it's in a denied environment
   * (teseoV) This stops us updating that as a valid observation time */
  if (fabs(obs_time.tow) > FLOAT_EQUALITY_EPS) {
    // TODO(SSTM-28) -  update time truth with observations
    state.time_from_input_data = obs_time;
  }
}

/* Write the SBP packet to STDOUT. In theory, I could use
   sbp_send_message(). */
static void cb_rtcm_to_sbp(uint16_t msg_id,
                           uint8_t length,
                           uint8_t *buffer,
                           uint16_t sender_id,
                           void *context) {
  if (state.use_time_from_input_obs) {
    if (msg_id == SBP_MSG_OBS) {
      update_obs_time((msg_obs_t *)buffer);
    }
  } else {
    time_truth_update_from_sbp(&time_truth, msg_id, length, buffer);
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
  ssize_t numwritten = g_writefn(tmpbuf, sizeof(tmpbuf), context);
  if (numwritten < (ssize_t)sizeof(tmpbuf)) {
    fprintf(stderr, "Write failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }

  numwritten = g_writefn(buffer, length, context);
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
  numwritten = g_writefn(crcbuf, sizeof(crcbuf), context);
  if (numwritten < (ssize_t)sizeof(crcbuf)) {
    fprintf(stderr, "Write failure at %d, %s. Aborting!\n", __LINE__, __FILE__);
    exit(EXIT_FAILURE);
  }
}

static void cb_base_obs_invalid(const double timediff, void *context) {
  (void)timediff;
  (void)context; /* squash warning */
  // TODO(SSTM-30) reintroduce this message, also downgrade level to warning
  // fprintf(stderr, "Invalid base observation! timediff: %lf\n", timediff);
}

static void help(char *arg, const char *additional_opts_help) {
  fprintf(stderr, "Usage: %s [options]%s\n", arg, additional_opts_help);
  fprintf(stderr, "  -h this message\n");
  fprintf(stderr, "  -b CODE:BIAS adds `bias` to signals with code `code`\n");
  fprintf(stderr,
          "  -c CODE:BIAS applies a liner pseudorange bias to Glonass signals "
          "with code CODE_GLO_L1OF or CODE_GLO_L2OF\n");
  fprintf(stderr,
          "  -l CODE:BIAS applies a liner phase bias to Glonass signals with "
          "code CODE_GLO_L1OF or CODE_GLO_L2OF\n");
  fprintf(stderr,
          "  -w GPS WN:GPS TOW passes time to the converter, needs to be "
          "accurate to within a half "
          "week for GPS only and within a half day for glonass, TOW is in "
          "seconds\n");
  fprintf(stderr,
          "  -d YEAR:MONTH:DAY:HOUR passes time to the nearest hour of the "
          "beginning of the data to "
          "the converter, needs to be accurate to within a half week for GPS "
          "only and within a half day "
          "for glonass\n");
  fprintf(stderr, "  -o Use observations instead of ephemerides time source");
  fprintf(stderr, "  -[GREICJS] disables a constellation\n");
  fprintf(stderr, "  -v for stderr verbosity\n");
}

int rtcm3tosbp_main(int argc,
                    char **argv,
                    const char *additional_opts_help,
                    readfn_ptr readfn,
                    writefn_ptr writefn,
                    void *context) {
  /* initialize time from systime */
  time_t ct_utc_unix = 0;
  /* use observations instead of ephemerides as time source */
  bool use_obs_time = false;

  g_writefn = writefn;

  int opt;
  while ((opt = getopt(argc, argv, "hb:c:l:w:d:oGRECJS:v")) != -1) {
    if (optarg && *optarg == '=') {
      optarg++;
    }
    switch (opt) {
      case 'h':
        help(argv[0], additional_opts_help);
        return 0;
      case 'b':
        parse_biases(optarg);
        break;
      case 'c':
        parse_glonass_code_biases(optarg);
        break;
      case 'l':
        parse_glonass_phase_biases(optarg);
        break;
      case 'w': {
        s16 week_num;
        double time_of_week;
        int num_args = sscanf(optarg, "%hd:%lf", &week_num, &time_of_week);
        if (num_args != 2) {
          fprintf(stderr, "expecting GPS week number and time of week\n");
          help(argv[0], additional_opts_help);
          return -1;
        }
        if (gps_time_valid(&(gps_time_t){time_of_week, week_num})) {
          ct_utc_unix = gps2time(&(gps_time_t){time_of_week, week_num});
        }
        break;
      }
      case 'd': {
        int year, month, day, hour;
        int n = sscanf(optarg, "%d:%d:%d:%d", &year, &month, &day, &hour);
        if (n != 4) {
          fprintf(stderr, "expecting [year:month:day:hour]\n");
          help(argv[0], additional_opts_help);
          return -1;
        }
        gps_time_t gps_time = date2gps(year, month, day, hour, 0, 0);
        if (gps_time_valid(&gps_time)) {
          ct_utc_unix = gps2time(&gps_time);
        }

        break;
      }
      case 'o':
        use_obs_time = true;
        break;
      case 'G':
        constellation_mask[CONSTELLATION_GPS] = true;
        fprintf(stderr, "Disabling Navstar\n");
        break;
      case 'R':
        constellation_mask[CONSTELLATION_GLO] = true;
        fprintf(stderr, "Disabling Glonass\n");
        break;
      case 'E':
        constellation_mask[CONSTELLATION_GAL] = true;
        fprintf(stderr, "Disabling Galileo\n");
        break;
      case 'C':
        constellation_mask[CONSTELLATION_BDS] = true;
        fprintf(stderr, "Disabling Beidou\n");
        break;
      case 'J':
        constellation_mask[CONSTELLATION_QZS] = true;
        fprintf(stderr, "Disabling QZSS\n");
        break;
      case 'S':
        constellation_mask[CONSTELLATION_SBAS] = true;
        fprintf(stderr, "Disabling SBAS\n");
        break;
      case 'v':
        verbosity_level++;
        break;
      default:
        break;
    }
  }

  time_truth_init(&time_truth);

  if (ct_utc_unix > 0) {
    /* set time, account for UTC<->GPS leap second difference */
    gps_time_t noleapsec = time2gps_t(ct_utc_unix);
    double gps_utc_offset = get_gps_utc_offset(&noleapsec, NULL);
    ct_utc_unix += (s8)rint(gps_utc_offset);
    gps_time_t withleapsec = time2gps_t(ct_utc_unix);
    gps_time_t current_time;
    current_time.tow = withleapsec.tow;
    current_time.wn = withleapsec.wn;

    time_truth_update(&time_truth, TIME_TRUTH_EPH_GAL, current_time);
  }

  rtcm2sbp_init(
      &state, &time_truth, cb_rtcm_to_sbp, cb_base_obs_invalid, context);

  if (use_obs_time) {
    state.use_time_from_input_obs = true;
    time_truth_get(&time_truth, NULL, &state.time_from_input_data);
  }

  /* todo: Do we want to return a non-zero value on an error? */
  ssize_t ret;
  do {
    ret = rtcm2sbp_process(&state, readfn);
  } while (ret > 0);

  return 0;
}

static void parse_biases(char *arg) {
  int n, code;
  float bias;
  char *tok = strtok(arg, ",");
  while (NULL != tok) {
    n = sscanf(tok, "%d:%f", &code, &bias);
    if ((n != 2) || (code >= CODE_COUNT)) {
      break;
    }
    sbp_signal_biases[code] = bias;
    if (verbosity_level > VERB_NORMAL) {
      fprintf(stderr, "Applying %.1f m bias to code %02d\n", bias, code);
    }
    tok = strtok(NULL, ",");
  }
}

static void parse_glonass_code_biases(char *arg) {
  int n, code;
  float bias_fcn_ratio;
  char *tok = strtok(arg, ",");
  while (NULL != tok) {
    n = sscanf(tok, "%d:%f", &code, &bias_fcn_ratio);
    if (n != 2) {
      break;
    }
    if (code == CODE_GLO_L1OF) {
      sbp_glo_code_bias[0] = bias_fcn_ratio;
      if (verbosity_level > VERB_NORMAL) {
        fprintf(stderr, "Applying %.1f m/fcn to L1OF\n", bias_fcn_ratio);
      }
    } else if (code == CODE_GLO_L2OF) {
      sbp_glo_code_bias[1] = bias_fcn_ratio;
      if (verbosity_level > VERB_NORMAL) {
        fprintf(stderr, "Applying %.1f m/fcn to L2OF\n", bias_fcn_ratio);
      }
    }
    tok = strtok(NULL, ",");
  }
}

static void parse_glonass_phase_biases(char *arg) {
  int n, code;
  float bias_fcn_ratio;
  char *tok = strtok(arg, ",");
  while (NULL != tok) {
    n = sscanf(tok, "%d:%f", &code, &bias_fcn_ratio);
    if (n != 2) {
      break;
    }
    if (code == CODE_GLO_L1OF) {
      sbp_glo_phase_bias[0] = bias_fcn_ratio;
      if (verbosity_level > VERB_NORMAL) {
        fprintf(stderr, "Applying %.1f circles/fcn to L1OF\n", bias_fcn_ratio);
      }
    } else if (code == CODE_GLO_L2OF) {
      sbp_glo_phase_bias[1] = bias_fcn_ratio;
      if (verbosity_level > VERB_NORMAL) {
        fprintf(stderr, "Applying %.1f circles/fcn to L2OF\n", bias_fcn_ratio);
      }
    }
    tok = strtok(NULL, ",");
  }
}
