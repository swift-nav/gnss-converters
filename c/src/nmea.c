/*
 * Copyright (C) 2013-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gnss-converters/nmea.h>
#include <swiftnav/array_tools.h>
#include <swiftnav/constants.h>
#include <swiftnav/pvt_result.h>
#include <swiftnav/signal.h>
#include "sbp_nmea_internal.h"

/** \addtogroup io
 * \{ */

/** \defgroup nmea NMEA
 * Send messages in NMEA 2.30 format.
 * \{ */

/* SBAS NMEA SV IDs are from 33 to 54 */
#define NMEA_SV_ID_OFFSET_SBAS (-87)

/* GLO NMEA SV IDs are from 65 to 96 */
#define NMEA_SV_ID_OFFSET_GLO (64)

/* GAL NMEA SV IDs are from 301 to 336 */
#define NMEA_SV_ID_OFFSET_GAL (300)

/* BDS NMEA SV IDs are from 401 to 437 */
#define NMEA_SV_ID_OFFSET_BDS2 (400)

/* Max SVs reported per GSA message */
#define GSA_MAX_SV 12

/* Number of decimals in NMEA time stamp (valid values 1-4) */
#define NMEA_UTC_S_DECIMALS 2
#define NMEA_UTC_S_FRAC_DIVISOR pow(10, NMEA_UTC_S_DECIMALS)

/* Adequate until end of year 999999.
   Worst case: "%02d%02d%05.2f,%02d,%02d,%lu" */
#define NMEA_TS_MAX_LEN (21 + NMEA_UTC_S_DECIMALS)

/* Accuracy of Course Over Ground */
#define NMEA_COG_DECIMALS 1
#define NMEA_COG_FRAC_DIVISOR pow(10, NMEA_COG_DECIMALS)

/* Based on testing calculated Course Over Ground starts deviating noticeably
 * below this limit. */
#define NMEA_COG_STATIC_LIMIT_MS 0.1f
#define NMEA_COG_STATIC_LIMIT_KNOTS MS2KNOTS(NMEA_COG_STATIC_LIMIT_MS, 0, 0)
#define NMEA_COG_STATIC_LIMIT_KPH MS2KMHR(NMEA_COG_STATIC_LIMIT_MS, 0, 0)

typedef enum talker_id_e {
  TALKER_ID_INVALID = -1,
  TALKER_ID_GP = 0,
  TALKER_ID_GL = 1,
  TALKER_ID_GA = 2,
  TALKER_ID_GB = 3,
  TALKER_ID_COUNT = 4
} talker_id_t;

#define NMEA_SUFFIX_LEN                    \
  6 /* How much room to leave for the NMEA \
       checksum, CRLF + null termination,  \
       i.e. "*%02X\r\n\0" */

/** Some helper macros for functions generating NMEA sentences. */

/** NMEA_SENTENCE_START: declare a buffer and set up some pointers
 * max_len = max possible length of the body of the message
 * (not including suffix)
 */
#define NMEA_SENTENCE_START(max_len)            \
  char sentence_buf[max_len + NMEA_SUFFIX_LEN]; \
  char *sentence_bufp = sentence_buf;           \
  char *const sentence_buf_end = sentence_buf + max_len;

/** NMEA_SENTENCE_PRINTF: use like printf, can use multiple times
    within a sentence. */
#define NMEA_SENTENCE_PRINTF(fmt, ...)                                        \
  do {                                                                        \
    sentence_bufp += snprintf(                                                \
        sentence_bufp, sentence_buf_end - sentence_bufp, fmt, ##__VA_ARGS__); \
    if (sentence_bufp >= sentence_buf_end) sentence_bufp = sentence_buf_end;  \
  } while (0)

/** NMEA_SENTENCE_DONE: append checksum and dispatch.
 * \note According to section 5.3.1 of the NMEA 0183 spec, sentences are
 *       terminated with <CR><LF>. The sentence_buf is null_terminated.
 *       The call to nmea_output has been modified to remove the NULL.
 *       This will also affect all registered dispatchers
 */
#define NMEA_SENTENCE_DONE(state)                             \
  do {                                                        \
    nmea_append_checksum(sentence_buf, sizeof(sentence_buf)); \
    nmea_output(state, sentence_buf);                         \
  } while (0)

/** Output NMEA sentence.

 * \param s The NMEA sentence to output.
 * \param size This is the C-string size, not including the null character
 */
static void nmea_output(const struct sbp_nmea_state *state, char *sentence) {
  state->cb_sbp_to_nmea(sentence);
}

/** Calculate and append the checksum of an NMEA sentence.
 * Calculates the bitwise XOR of the characters in a string until the end of
 * the string or a `*` is encountered. If the first character is `$` then it
 * is skipped.
 *
 * \param s A null-terminated NMEA sentence, up to and optionally
 * including the '*'
 *
 * \param size Length of the buffer.
 *
 */
static void nmea_append_checksum(char *s, size_t size) {
  u8 sum = 0;
  char *p = s;

  /* '$' header not included in checksum calculation */
  if (*p == '$') {
    p++;
  }

  /* '*'  not included in checksum calculation */
  while (*p != '*' && *p && p + NMEA_SUFFIX_LEN < s + size) {
    sum ^= *p;
    p++;
  }

  sprintf(p, "*%02X\r\n", sum);
}

/** Wrapper function for vsnprintf to accommodate return value handling. Updates
 *  buf_ptr accordingly if there's no encoding error. If buffer is full, sets
 *  buf_ptr to buf_end.
 */
__attribute__((format(printf, 3, 4))) static void vsnprintf_wrap(
    char **buf_ptr, char *buf_end, const char *format, ...) {
  va_list args;
  va_start(args, format);

  int res = vsnprintf(*buf_ptr, buf_end - *buf_ptr, format, args);

  va_end(args);

  if (res < 0) {
    return;
  }

  *buf_ptr += res;

  if (*buf_ptr >= buf_end) {
    *buf_ptr = buf_end;
  }
}

/** Generate UTC date time string. Time field is before date field.
 *
 * \param[in] sbp_msg_time Time and date to create the str from.
 * \param[in] time Time field is to be added.
 * \param[in] date Date field is to be added.
 * \param[in] trunc_date Truncate date field. No effect if param date is false.
 * \param[in] t UTC time structure
 * \param[out] utc_str Created date time string.
 * \param[in] size utc_str size.
 *
 */
static void get_utc_time_string(bool time,
                                bool date,
                                bool trunc_date,
                                const msg_utc_time_t *sbp_utc_time,
                                char *utc_str,
                                u8 size) {
  char *buf_end = utc_str + size;

  if (time) {
    /* Time (UTC) */
    vsnprintf_wrap(
        &utc_str,
        buf_end,
        "%02u%02u%02u.%0*u,",
        sbp_utc_time->hours,
        sbp_utc_time->minutes,
        sbp_utc_time->seconds,
        NMEA_UTC_S_DECIMALS,
        (u16)roundf(NMEA_UTC_S_FRAC_DIVISOR * sbp_utc_time->ns * 1e-9));
  }

  if (date) {
    /* Date Stamp */
    if (trunc_date) {
      vsnprintf_wrap(&utc_str,
                     buf_end,
                     "%02u%02u%02u,",
                     sbp_utc_time->day,
                     sbp_utc_time->month,
                     (u8)(sbp_utc_time->year % 100));
    } else {
      vsnprintf_wrap(&utc_str,
                     buf_end,
                     "%02u,%02u,%" PRIu32 ",",
                     sbp_utc_time->day,
                     sbp_utc_time->month,
                     sbp_utc_time->year);
    }
  }
}

/* General note: the NMEA functions below mask the time source, position and
   velocity modes to ensure prevention of accidental bugs in the future.

   The SBP specification only specifies the first 3 bits to indicate the mode.
   Thus the RAIM repair flag or other future flags need to be masked out.

   It is needed in this code becasue we pass SBP formatted structures, not the
   raw positioning modes.
*/

/** Assemble a NMEA GPGGA message and send it out NMEA USARTs.
 * NMEA GPGGA message contains Global Positioning System Fix Data.
 *
 * \param state Current SBP2NMEA state
 */
void send_gpgga(const struct sbp_nmea_state *state) {
  /* GGA sentence is formed by splitting latitude and longitude
     into degrees and minutes parts and then printing them separately
     using printf. Before doing the split we want to take care of
     the proper rounding. Doing it after the split would lead to the need
     of handling the case of minutes part overflow separately. Otherwise,
     printf would do the rounding of the minutes part, which could result
     in printing 60 minutes value.
     E.g. doing this way lat = 15.9999999996 would be printed as
     $GPGGA,hhmmss.ss,1600.000000,...
     and NOT
     $GPGGA,hhmmss.ss,1560.000000,...
   */
  const msg_pos_llh_t *sbp_pos_llh = &state->sbp_pos_llh;
  const msg_utc_time_t *sbp_utc_time = &state->sbp_utc_time;
  const msg_age_corrections_t *sbp_age = &state->sbp_age_corr;
  const msg_dops_t *sbp_dops = &state->sbp_dops;

  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  u8 fix_type = NMEA_GGA_QI_INVALID;
  if ((sbp_pos_llh->flags & POSITION_MODE_MASK) != POSITION_MODE_NONE) {
    fix_type = get_nmea_quality_indicator(sbp_pos_llh->flags);
  }

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGGA,");

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      true, false, false, sbp_utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  if (fix_type != NMEA_GGA_QI_INVALID) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,",
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,");
  }
  NMEA_SENTENCE_PRINTF("%01d,", fix_type);

  if (fix_type != NMEA_GGA_QI_INVALID) {
    NMEA_SENTENCE_PRINTF("%02d,%.1f,%.2f,M,0.0,M,",
                         sbp_pos_llh->n_sats,
                         round(10 * sbp_dops->hdop * 0.01) / 10,
                         sbp_pos_llh->height);
  } else {
    NMEA_SENTENCE_PRINTF(",,,M,,M,");
  }

  if ((fix_type == NMEA_GGA_QI_DGPS) || (fix_type == NMEA_GGA_QI_FLOAT) ||
      (fix_type == NMEA_GGA_QI_RTK)) {
    NMEA_SENTENCE_PRINTF(
        "%.1f,%04d",
        sbp_age->age * 0.1,
        state->base_sender_id & 0x3FF); /* ID range is 0000 to 1023 */
  } else {
    NMEA_SENTENCE_PRINTF(",");
  }

  NMEA_SENTENCE_DONE(state);
}

gnss_signal_t sbp_to_gnss(sbp_gnss_signal_t sid) {
  gnss_signal_t other;
  other.sat = sid.sat;
  other.code = sid.code;
  return other;
}

int gsa_cmp(const void *a, const void *b) { return (*(u16 *)a - *(u16 *)b); }

static u16 nmea_get_id(const sbp_gnss_signal_t sid) {
  u16 id = -1;

  switch (sid_to_constellation(sbp_to_gnss(sid))) {
    case CONSTELLATION_BDS:
      id = NMEA_SV_ID_OFFSET_BDS2 + sid.sat;
      break;
    case CONSTELLATION_GAL:
      id = NMEA_SV_ID_OFFSET_GAL + sid.sat;
      break;
    case CONSTELLATION_GPS:
      id = sid.sat;
      break;
    case CONSTELLATION_GLO:
      id = NMEA_SV_ID_OFFSET_GLO + sid.sat;
      break;
    case CONSTELLATION_SBAS:
      id = NMEA_SV_ID_OFFSET_SBAS + sid.sat;
      break;
    case CONSTELLATION_QZS:
    case CONSTELLATION_COUNT:
    case CONSTELLATION_INVALID:
    default:
      log_error("NMEA: Unsupported constellation");
      break;
  }

  return id;
}

static const char *talker_id_to_str(const talker_id_t id) {
  switch (id) {
    case TALKER_ID_GA:
      return "GA";
    case TALKER_ID_GB:
      return "GB";
    case TALKER_ID_GL:
      return "GL";
    case TALKER_ID_GP:
      return "GP";
    case TALKER_ID_COUNT:
    case TALKER_ID_INVALID:
    default:
      log_debug("talker_id_to_str() error: invalid talker ID");
      return "";
  }
}

static talker_id_t sid_to_talker_id(const sbp_gnss_signal_t sid) {
  switch (sid_to_constellation(sbp_to_gnss(sid))) {
    case CONSTELLATION_GAL:
      return TALKER_ID_GA;
    case CONSTELLATION_BDS:
      return TALKER_ID_GB;
    case CONSTELLATION_GLO:
      return TALKER_ID_GL;
    case CONSTELLATION_GPS:
    case CONSTELLATION_QZS:
    case CONSTELLATION_SBAS:
      return TALKER_ID_GP;
    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    default:
      log_debug("sid_to_talker_id() error: unsupported constellation");
      return TALKER_ID_INVALID;
  }
}

/** Print a NMEA GSA string and send it out NMEA USARTs.
 * NMEA GSA message contains GNSS DOP and Active Satellites.
 *
 * \param prns         Array of PRNs to output.
 * \param num_prns     Number of valid PRNs in array.
 * \param sbp_pos_llh  Pos data pointer.
 * \param sbp_dops     Pointer to SBP MSG DOP struct (PDOP, HDOP, VDOP).
 * \param talker       Talker ID to use.
 */
void send_gsa_print(u16 *prns,
                    const u8 num_prns,
                    const msg_pos_llh_t *sbp_pos_llh,
                    const msg_dops_t *sbp_dops,
                    const char *talker,
                    const struct sbp_nmea_state *state) {
  assert(prns);
  assert(sbp_dops);
  assert(sbp_pos_llh);

  bool fix = POSITION_MODE_NONE != (sbp_pos_llh->flags & POSITION_MODE_MASK);

  /* Our fix is always 3D */
  char fix_mode = fix ? '3' : '1';

  NMEA_SENTENCE_START(120);
  /* Always automatic mode */
  NMEA_SENTENCE_PRINTF("$%sGSA,A,%c,", talker, fix_mode);

  qsort(prns, num_prns, sizeof(u16), gsa_cmp);

  for (u8 i = 0; i < GSA_MAX_SV; i++) {
    if (i < num_prns) {
      NMEA_SENTENCE_PRINTF("%02d,", prns[i]);
    } else {
      NMEA_SENTENCE_PRINTF(",");
    }
  }

  if (fix && (NULL != sbp_dops)) {
    NMEA_SENTENCE_PRINTF("%.1f,%.1f,%.1f",
                         round(sbp_dops->pdop * 0.1) / 10,
                         round(sbp_dops->hdop * 0.1) / 10,
                         round(sbp_dops->vdop * 0.1) / 10);
  } else {
    NMEA_SENTENCE_PRINTF(",,");
  }

  NMEA_SENTENCE_DONE(state);
}

/** Group measurements by constellation and forward information to GSA
 *  printing function.
 *
 * \note NMEA 0183 - Standard For Interfacing Marine Electronic Devices
 *       versions 2.30, 3.01 and 4.10 state following:
 *       If only GPS, GLONASS, Galileo etc. is used for the reported position
 *       solution the talker ID is GP, GL, etc. and the DOP values pertain to
 *       the individual system. If GPS, GLONASS, Galileo etc. are combined to
 *       obtain the reported position solution multiple GSA sentences are
 *       produced, one with the GPS satellites, another with the GLONASS
 *       satellites and another with Galileo satellites, etc. Each of these GSA
 *       sentences shall have talker ID GN, to indicate that the satellites are
 *       used in a combined solution and each shall have the PDOP, HDOP and VDOP
 *       for the combined satellites used in the position.
 *
 * \param sbp_pos      Pointer to position data
 * \param sbp_dops     Pointer to DOPS data
 * \param n_meas       Number of measurements
 * \param nav_meas     Array of navigation measurements
 */
void send_gsa(const struct sbp_nmea_state *state) {
  assert(state);
  const msg_pos_llh_t *sbp_pos = &state->sbp_pos_llh;
  const msg_dops_t *sbp_dops = &state->sbp_dops;
  const u8 n_obs = state->num_obs;
  const sbp_gnss_signal_t *nav_sids = state->nav_sids;

  u16 prns[TALKER_ID_COUNT][GSA_MAX_SV] = {{0}};
  u8 num_prns[TALKER_ID_COUNT] = {0};

  /* Assemble list of currently active SVs */
  for (u8 i = 0; i < n_obs; i++) {
    const sbp_gnss_signal_t sid = nav_sids[i];
    talker_id_t id = sid_to_talker_id(sid);

    if (TALKER_ID_INVALID == id) {
      /* Unsupported constellation */
      continue;
    }

    /* Check following:
     *   - constellation to group by correct talker ID
     *       * GPS, QZSS and SBAS use GP
     *       * GLO uses GL
     *       * BDS uses GB
     *       * GAL uses GA
     *   - maximum group size is GSA_MAX_SV
     *   - if SV is reported already by another signal (eg. GPS L1CA vs L2C)
     */
    if (num_prns[id] > GSA_MAX_SV) {
      /* Talker ID specific sentence already maxed out */
      continue;
    }

    if (is_value_in_array_u16(prns[id], num_prns[id], nmea_get_id(sid))) {
      /* SV already listed by another signal */
      continue;
    }

    /* All checks pass, add to list */
    prns[id][num_prns[id]++] = nmea_get_id(sid);
  }

  u8 constellations = 0;
  for (u8 i = 0; i < TALKER_ID_COUNT; ++i) {
    constellations += (0 != num_prns[i]) ? 1 : 0;
  }

  /* Check if no SVs identified */
  if (0 == constellations) {
    /* At bare minimum, print empty GPGSA and be done with it */
    send_gsa_print(prns[TALKER_ID_GP],
                   num_prns[TALKER_ID_GP],
                   sbp_pos,
                   sbp_dops,
                   "GP",
                   state);
    return;
  }

  /* If at least two constellations detected, use GN talker ID */
  bool use_gn = (constellations > 1);

  /* Print active SVs per talker ID */
  for (u8 i = 0; i < TALKER_ID_COUNT; ++i) {
    if (0 == num_prns[i]) {
      /* Empty */
      continue;
    }

    send_gsa_print(prns[i],
                   num_prns[i],
                   sbp_pos,
                   sbp_dops,
                   use_gn ? "GN" : talker_id_to_str(i),
                   state);
  }
}

/** Calculate Course and Speed Over Ground values.
 *
 * \param[in]  sbp_vel_ned  pointer to sbp vel ned struct
 * \param[out] cog          true course over ground [deg]
 * \param[out] sog_knots    speed over ground [knots]
 * \param[out] sog_kph      speed over ground [kph]
 */
static void calc_cog_sog(const msg_vel_ned_t *sbp_vel_ned,
                         double *cog,
                         double *sog_knots,
                         double *sog_kph) {
  double vel_north_ms = MM2M(sbp_vel_ned->n);
  double vel_east_ms = MM2M(sbp_vel_ned->e);

  *cog = R2D * atan2(vel_east_ms, vel_north_ms);

  /* Convert negative values to positive */
  if (*cog < 0.0) {
    *cog += FULL_CIRCLE_DEG;
  }

  /* Rounding to specified accuracy */
  *cog = roundf(*cog * NMEA_COG_FRAC_DIVISOR) / NMEA_COG_FRAC_DIVISOR;

  /* Avoid having duplicate values for same point (0 and 360) */
  if (fabs(FULL_CIRCLE_DEG - *cog) < 1 / NMEA_COG_FRAC_DIVISOR) {
    *cog = 0;
  }

  *sog_knots = MS2KNOTS(vel_north_ms, vel_east_ms, 0);
  *sog_kph = MS2KMHR(vel_north_ms, vel_east_ms, 0);
}

/** Assemble an NMEA GPRMC message and send it out NMEA USARTs.
 * NMEA RMC contains Recommended Minimum Specific GNSS Data.
 *
 * \param sbp_pos_llh  pointer to sbp pos llh struct
 * \param sbp_vel_ned  pointer to sbp vel ned struct
 * \param sbp_msg_time Pointer to sbp gps time struct
 * \param utc_time     Pointer to UTC time
 */
void send_gprmc(const struct sbp_nmea_state *state) {
  const msg_pos_llh_t *sbp_pos_llh = &state->sbp_pos_llh;
  const msg_vel_ned_t *sbp_vel_ned = &state->sbp_vel_ned;
  const msg_utc_time_t *sbp_utc_time = &state->sbp_utc_time;
  /* See the relevant comment for the similar code in nmea_gpgga() function
     for the reasoning behind (... * 1e8 / 1e8) trick */
  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);
  char status = get_nmea_status(sbp_pos_llh->flags);

  double cog, sog_knots, sog_kph;
  calc_cog_sog(sbp_vel_ned, &cog, &sog_knots, &sog_kph);

  NMEA_SENTENCE_START(140);
  NMEA_SENTENCE_PRINTF("$GPRMC,"); /* Command */

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      true, false, false, sbp_utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF("%c,", /* Status */
                       status);

  if ((sbp_pos_llh->flags & POSITION_MODE_MASK) != POSITION_MODE_NONE) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,", /* Lat/Lon */
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,"); /* Lat/Lon */
  }

  if ((sbp_pos_llh->flags & VELOCITY_MODE_MASK) != VELOCITY_MODE_NONE) {
    NMEA_SENTENCE_PRINTF("%.2f,", sog_knots); /* Speed */
    if (NMEA_COG_STATIC_LIMIT_KNOTS < sog_knots) {
      NMEA_SENTENCE_PRINTF("%.*f,", NMEA_COG_DECIMALS, cog); /* Course */
    } else {
      NMEA_SENTENCE_PRINTF(","); /* Course */
    }
  } else {
    NMEA_SENTENCE_PRINTF(",,"); /* Speed, Course */
  }

  char date[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      false, true, true, sbp_utc_time, date, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", date);

  NMEA_SENTENCE_PRINTF(
      ",,"  /* Magnetic Variation */
      "%c", /* Mode Indicator */
      mode);
  NMEA_SENTENCE_DONE(state);
}

/** Assemble an NMEA GPVTG message and send it out NMEA USARTs.
 * NMEA VTG contains Course Over Ground & Ground Speed.
 *
 * \param sbp_vel_ned Pointer to sbp vel ned struct.
 */
void send_gpvtg(const struct sbp_nmea_state *state) {
  const msg_vel_ned_t *sbp_vel_ned = &state->sbp_vel_ned;
  const msg_pos_llh_t *sbp_pos_llh = &state->sbp_pos_llh;

  double cog, sog_knots, sog_kph;
  calc_cog_sog(sbp_vel_ned, &cog, &sog_knots, &sog_kph);

  /* Position indicator is used based upon spec
     "Positioning system mode indicator" means we should
     see the same mode for pos and velocity messages
     in a particular epoch */

  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPVTG,"); /* Command */

  bool is_moving =
      (sbp_pos_llh->flags & VELOCITY_MODE_MASK) != VELOCITY_MODE_NONE;

  if (is_moving && NMEA_COG_STATIC_LIMIT_KNOTS < sog_knots) {
    NMEA_SENTENCE_PRINTF("%.*f,T,", NMEA_COG_DECIMALS, cog); /* Course */
  } else {
    NMEA_SENTENCE_PRINTF(",T,"); /* Course */
  }

  NMEA_SENTENCE_PRINTF(",M,"); /* Magnetic Course (omitted) */

  if (is_moving) {
    /* Speed (knots, km/hr) */
    NMEA_SENTENCE_PRINTF("%.2f,N,%.2f,K,", sog_knots, sog_kph);
  } else {
    /* Speed (knots, km/hr) */
    NMEA_SENTENCE_PRINTF(",N,,K,");
  }

  /* Mode (note this is position mode not velocity mode)*/
  NMEA_SENTENCE_PRINTF("%c", mode);
  NMEA_SENTENCE_DONE(state);
}

/** Assemble an NMEA GPHDT message and send it out NMEA USARTs.
 * NMEA HDT contains Heading.
 *
 */
void send_gphdt(const struct sbp_nmea_state *state) {
  const msg_baseline_heading_t *sbp_baseline_heading = &state->sbp_heading;
  NMEA_SENTENCE_START(40);
  NMEA_SENTENCE_PRINTF("$GPHDT,"); /* Command */
  if ((POSITION_MODE_MASK & sbp_baseline_heading->flags) ==
      POSITION_MODE_FIXED) {
    NMEA_SENTENCE_PRINTF(
        "%.1f,T",
        (float)sbp_baseline_heading->heading /
            MSG_HEADING_SCALE_FACTOR); /* Heading only valid when fixed */
  } else {
    NMEA_SENTENCE_PRINTF(",T"); /* Heading only valid when fixed */
  }
  NMEA_SENTENCE_DONE(state);
}

/** Assemble an NMEA GPGLL message and send it out NMEA USARTs.
 * NMEA GLL contains Geographic Position Latitude/Longitude.
 *
 * \param sbp_pos_llh  Pointer to sbp pos llh struct.
 * \param sbp_msg_time Pointer to sbp gps time struct.
 * \param sbp_utc_time Pointer to sbp UTC time.
 */
void send_gpgll(const struct sbp_nmea_state *state) {
  const msg_pos_llh_t *sbp_pos_llh = &state->sbp_pos_llh;
  const msg_utc_time_t *sbp_utc_time = &state->sbp_utc_time;
  /* See the relevant comment for the similar code in nmea_gpgga() function
     for the reasoning behind (... * 1e8 / 1e8) trick */
  double lat = fabs(round(sbp_pos_llh->lat * 1e8) / 1e8);
  double lon = fabs(round(sbp_pos_llh->lon * 1e8) / 1e8);

  char lat_dir = sbp_pos_llh->lat < 0.0 ? 'S' : 'N';
  assert(lat <= UINT16_MAX);
  u16 lat_deg = (u16)lat; /* truncation towards zero */
  double lat_min = (lat - (double)lat_deg) * 60.0;

  char lon_dir = sbp_pos_llh->lon < 0.0 ? 'W' : 'E';
  assert(lon <= UINT16_MAX);
  u16 lon_deg = (u16)lon; /* truncation towards zero */
  double lon_min = (lon - (double)lon_deg) * 60.0;

  char status = get_nmea_status(sbp_pos_llh->flags);
  char mode = get_nmea_mode_indicator(sbp_pos_llh->flags);

  NMEA_SENTENCE_START(120);
  NMEA_SENTENCE_PRINTF("$GPGLL,"); /* Command */

  if ((sbp_pos_llh->flags & POSITION_MODE_MASK) != POSITION_MODE_NONE) {
    NMEA_SENTENCE_PRINTF("%02u%010.7f,%c,%03u%010.7f,%c,", /* Lat/Lon */
                         lat_deg,
                         lat_min,
                         lat_dir,
                         lon_deg,
                         lon_min,
                         lon_dir);
  } else {
    NMEA_SENTENCE_PRINTF(",,,,"); /* Lat/Lon */
  }

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      true, false, false, sbp_utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF("%c,%c", /* Status, Mode */
                       status,
                       mode);
  NMEA_SENTENCE_DONE(state);
}

/** Assemble an NMEA GPZDA message and send it out NMEA USARTs.
 * NMEA ZDA contains UTC Time and Date.
 *
 * \param sbp_utc_time Pointer to sbp UTC time
 */
void send_gpzda(const struct sbp_nmea_state *state) {
  const msg_utc_time_t *sbp_utc_time = &state->sbp_utc_time;

  NMEA_SENTENCE_START(40);
  NMEA_SENTENCE_PRINTF("$GPZDA,"); /* Command */

  char utc[NMEA_TS_MAX_LEN];
  get_utc_time_string(
      true, true, false, sbp_utc_time, utc, NMEA_TS_MAX_LEN);
  NMEA_SENTENCE_PRINTF("%s", utc);

  NMEA_SENTENCE_PRINTF(","); /* Time zone */
  NMEA_SENTENCE_DONE(state);

} /* send_gpzda() */

bool check_nmea_rate(u32 rate, u32 gps_tow_ms, float soln_freq) {
  if (rate == 0) {
    return false;
  }
  /* If the modulo of latest gps time estimate time with configured
   * output period is less than 1/2 the solution period we should send the NMEA
   * message.
   * This way, we still send no_fix messages when receiver clock is drifting. */
  u32 soln_period_ms = (u32)(1.0 / soln_freq * 1e3);
  u32 output_period_ms = (u32)soln_period_ms * rate;
  if (((gps_tow_ms) % output_period_ms) < (soln_period_ms / 2)) {
    return true;
  }
  return false;
}

/** Convert the SBP status flag into NMEA Status field.
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_status(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case POSITION_MODE_NONE:
      return 'V';
    case POSITION_MODE_DEAD_RECKONING:
    case POSITION_MODE_SPP: /* autonomous mode */
    case POSITION_MODE_DGNSS:
    case POSITION_MODE_SBAS:
    case POSITION_MODE_FLOAT:
    case POSITION_MODE_FIXED:
      return 'A';
    default:
      assert(!"Unsupported position type indicator");
      return 'V';
  }
}

/** Convert the SBP status flag into NMEA Mode Indicator field:
 * Ref: NMEA-0183 version 2.30 pp.42,43
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
char get_nmea_mode_indicator(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case POSITION_MODE_NONE:
      return 'N';
    case POSITION_MODE_SPP: /* autonomous mode */
      return 'A';
    case POSITION_MODE_DGNSS: /* differential mode */
    case POSITION_MODE_SBAS:
    case POSITION_MODE_FLOAT:
    case POSITION_MODE_FIXED:
      return 'D';
    case POSITION_MODE_DEAD_RECKONING:
      return 'E';
    default:
      assert(!"Unsupported position type indicator");
      return 'N';
  }
}

/** Convert the SBP status flag into NMEA Quality Indicator field:
 * Ref: NMEA-0183 version 2.30 pp.30
 *
 * \param flags        u8 sbp_pos_llh->flags
 */
u8 get_nmea_quality_indicator(u8 flags) {
  switch (flags & POSITION_MODE_MASK) {
    case POSITION_MODE_NONE:
      return NMEA_GGA_QI_INVALID;
    case POSITION_MODE_SPP:
      return NMEA_GGA_QI_GPS;
    case POSITION_MODE_DGNSS:
    case POSITION_MODE_SBAS:
      return NMEA_GGA_QI_DGPS;
    case POSITION_MODE_FLOAT:
      return NMEA_GGA_QI_FLOAT;
    case POSITION_MODE_FIXED:
      return NMEA_GGA_QI_RTK;
    case POSITION_MODE_DEAD_RECKONING:
      return NMEA_GGA_QI_EST;
    default:
      assert(!"Unsupported position type indicator");
      return NMEA_GGA_QI_INVALID;
  }
}

/** \} */

/** \} */
