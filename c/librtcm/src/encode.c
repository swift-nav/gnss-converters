/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtcm3/encode.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "rtcm3/bits.h"
#include "rtcm3/constants.h"
#include "rtcm3/msm_utils.h"

/** Convert a lock time in seconds into 7-bit RTCMv3 Lock Time Indicator value.
 * See RTCM 10403.1, Table 3.4-2.
 *
 *
 * \param time Lock time in seconds.
 * \return Lock Time Indicator value.
 */
static uint8_t to_lock_ind(double time) {
  if (time < 24) {
    return (uint8_t)time;
  }
  if (time < 72) {
    return (uint8_t)((time + 24) / 2);
  }
  if (time < 168) {
    return (uint8_t)((time + 120) / 4);
  }
  if (time < 360) {
    return (uint8_t)((time + 408) / 8);
  }
  if (time < 744) {
    return (uint8_t)((time + 1176) / 16);
  }
  if (time < 937) {
    return (uint8_t)((time + 3096) / 32);
  }
  return 127;
}

/** Convert a lock time in seconds into a 4-bit RTCMv3 Lock Time Indicator DF402
 * See RTCM 10403.1, Table 3.5-74.
 *
 * \param time Lock time in seconds.
 * \return Lock Time Indicator value 0-15.
 */
uint8_t rtcm3_encode_lock_time(double time) {
  if (time < 0.032) {
    return 0;
  }
  if (time < 0.064) {
    return 1;
  }
  if (time < 0.128) {
    return 2;
  }
  if (time < 0.256) {
    return 3;
  }
  if (time < 0.512) {
    return 4;
  }
  if (time < 1.024) {
    return 5;
  }
  if (time < 2.048) {
    return 6;
  }
  if (time < 4.096) {
    return 7;
  }
  if (time < 8.192) {
    return 8;
  }
  if (time < 16.384) {
    return 9;
  }
  if (time < 32.768) {
    return 10;
  }
  if (time < 65.536) {
    return 11;
  }
  if (time < 131.072) {
    return 12;
  }
  if (time < 262.144) {
    return 13;
  }
  if (time < 524.288) {
    return 14;
  }
  return 15;
}

/* Encode PhaseRange – L1 Pseudorange (DF012, DF018 ,DF042, DF048) */
static int32_t encode_diff_phaserange(double cp_pr, double freq) {
  double phase_unit = (GPS_C / freq) / 0.0005;
  int32_t ppr = lround(cp_pr * phase_unit);

  /* From specification: "Certain ionospheric conditions might cause the GPS
   * L1 Phaserange – L1 Pseudorange to diverge over time across the range
   * limits defined. Under these circumstances the computed value needs to be
   * adjusted (rolled over) by the equivalent of 1500 cycles in order to bring
   * the value back within the range" */

  if (ppr <= -C_2P19) {
    /* add multiples of 1500 cycles */
    cp_pr += 1500 * ceil((-C_2P19 / phase_unit - cp_pr) / 1500);
    ppr = lround(cp_pr * phase_unit);
  } else if (ppr >= C_2P19) {
    /* substract multiples of 1500 cycles */
    cp_pr -= 1500 * ceil((cp_pr - C_2P19 / phase_unit) / 1500);
    ppr = lround(cp_pr * phase_unit);
  }
  return ppr;
}

static void encode_basic_freq_data(const rtcm_freq_data *freq_data,
                                   const freq_t freq_enum,
                                   const double *l1_pr,
                                   uint8_t buff[],
                                   uint16_t *bit) {
  /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF014). */
  uint8_t amb = (uint8_t)(*l1_pr / PRUNIT_GPS);

  /* Construct L1 pseudorange value as it would be transmitted (DF011). */
  uint32_t calc_l1_pr = (uint32_t)round((*l1_pr - amb * PRUNIT_GPS) / 0.02);

  /* Calculate GPS Pseudorange (DF011/DF016). */
  uint32_t pr =
      (uint32_t)round((freq_data->pseudorange - amb * PRUNIT_GPS) / 0.02);

  double l1_prc = calc_l1_pr * 0.02 + amb * PRUNIT_GPS;

  double freq = 0;
  if (L1_FREQ == freq_enum) {
    freq = GPS_L1_HZ;
    rtcm_setbitu(buff, *bit, 1, 0);
    *bit += 1;
    rtcm_setbitu(
        buff, *bit, 24, freq_data->flags.fields.valid_pr ? pr : PR_L1_INVALID);
    *bit += 24;
  } else {
    freq = GPS_L2_HZ;
    rtcm_setbitu(buff, *bit, 2, 0);
    *bit += 2;
    rtcm_setbits(buff,
                 *bit,
                 14,
                 freq_data->flags.fields.valid_pr
                     ? (int32_t)pr - (int32_t)calc_l1_pr
                     : (int32_t)PR_L2_INVALID);
    *bit += 14;
  }

  if (freq_data->flags.fields.valid_cp) {
    /* phaserange - L1 pseudorange */
    double cp_pr = freq_data->carrier_phase - l1_prc / (GPS_C / freq);
    /* encode PhaseRange – L1 Pseudorange (DF012/DF018) and roll over if
     * necessary */
    int32_t ppr = encode_diff_phaserange(cp_pr, freq);
    rtcm_setbits(buff, *bit, 20, ppr);
  } else {
    rtcm_setbits(buff, *bit, 20, CP_INVALID);
  }
  *bit += 20;
  rtcm_setbitu(
      buff,
      *bit,
      7,
      freq_data->flags.fields.valid_lock ? to_lock_ind(freq_data->lock) : 0);
  *bit += 7;
}

static void encode_basic_glo_freq_data(const rtcm_freq_data *freq_data,
                                       const freq_t freq_enum,
                                       const double *l1_pr,
                                       const uint8_t fcn,
                                       uint8_t buff[],
                                       uint16_t *bit) {
  /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF044). */
  uint8_t amb = (uint8_t)(*l1_pr / PRUNIT_GLO);

  /* Construct L1 pseudorange value as it would be transmitted (DF041). */
  uint32_t calc_l1_pr = (uint32_t)round((*l1_pr - amb * PRUNIT_GLO) / 0.02);

  /* Calculate GLO Pseudorange (DF041/DF046). */
  uint32_t pr =
      (uint32_t)round((freq_data->pseudorange - amb * PRUNIT_GLO) / 0.02);

  double l1_prc = calc_l1_pr * 0.02 + amb * PRUNIT_GLO;

  double glo_freq = 0.0;
  if (L1_FREQ == freq_enum) {
    glo_freq = GLO_L1_HZ + (fcn - MT1012_GLO_FCN_OFFSET) * GLO_L1_DELTA_HZ;

    rtcm_setbitu(buff, *bit, 1, 0);
    *bit += 1;
    rtcm_setbitu(buff, *bit, 5, fcn);
    *bit += 5;
    rtcm_setbitu(
        buff, *bit, 25, freq_data->flags.fields.valid_pr ? pr : PR_L1_INVALID);
    *bit += 25;
  } else {
    glo_freq = GLO_L2_HZ + (fcn - MT1012_GLO_FCN_OFFSET) * GLO_L2_DELTA_HZ;

    rtcm_setbitu(buff, *bit, 2, 0);
    *bit += 2;
    rtcm_setbits(buff,
                 *bit,
                 14,
                 freq_data->flags.fields.valid_pr
                     ? (int32_t)pr - (int32_t)calc_l1_pr
                     : (int32_t)PR_L2_INVALID);
    *bit += 14;
  }

  if (freq_data->flags.fields.valid_cp) {
    /* phaserange - L1 pseudorange */
    double cp_pr = freq_data->carrier_phase - l1_prc / (GPS_C / glo_freq);

    /* Calculate PhaseRange – L1 Pseudorange (DF042/DF048) and roll over if
     * necessary */
    int32_t ppr = encode_diff_phaserange(cp_pr, glo_freq);
    rtcm_setbits(buff, *bit, 20, ppr);
  } else {
    rtcm_setbits(buff, *bit, 20, CP_INVALID);
  }
  *bit += 20;
  rtcm_setbitu(
      buff,
      *bit,
      7,
      freq_data->flags.fields.valid_lock ? to_lock_ind(freq_data->lock) : 0);
  *bit += 7;
}

/** Write RTCM header for observation message types 1001..1004.
 *
 * The data message header will be written starting from byte zero of the
 * buffer. If the buffer also contains a frame header then be sure to pass a
 * pointer to the start of the data message rather than a pointer to the start
 * of the frame buffer. The RTCM observation header is 8 bytes (64 bits) long.
 *
 * If the Synchronous GNSS Message Flag is set to `0`, it means that no further
 * GNSS observables referenced to the same Epoch Time will be transmitted. This
 * enables the receiver to begin processing the data immediately after decoding
 * the message. If it is set to `1`, it means that the next message will
 * contain observables of another GNSS source referenced to the same Epoch
 * Time.
 *
 * Divergence-free Smoothing Indicator values:
 *
 * Indicator | Meaning
 * --------- | ----------------------------------
 *     0     | Divergence-free smoothing not used
 *     1     | Divergence-free smoothing used
 *
 * GPS Smoothing Interval indicator values are listed in RTCM 10403.1 Table
 * 3.4-4, reproduced here:
 *
 * Indicator | Smoothing Interval
 * --------- | ------------------
 *  000 (0)  |   No smoothing
 *  001 (1)  |   < 30 s
 *  010 (2)  |   30-60 s
 *  011 (3)  |   1-2 min
 *  100 (4)  |   2-4 min
 *  101 (5)  |   4-8 min
 *  110 (6)  |   >8 min
 *  111 (7)  |   Unlimited
 *
 * \param header pointer to the obs header to encode
 * \param num_sats number of satellites in message
 * \param buff A pointer to the RTCM data message buffer.
 */
static uint16_t rtcm3_write_header(const rtcm_obs_header *header,
                                   uint8_t num_sats,
                                   uint8_t buff[]) {
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, header->msg_num);
  bit += 12;
  rtcm_setbitu(buff, bit, 12, header->stn_id);
  bit += 12;
  rtcm_setbitu(buff, bit, 30, (uint32_t)round(header->tow_ms));
  bit += 30;
  rtcm_setbitu(buff, bit, 1, header->sync);
  bit += 1;
  rtcm_setbitu(buff, bit, 5, num_sats);
  bit += 5;
  rtcm_setbitu(buff, bit, 1, header->div_free);
  bit += 1;
  rtcm_setbitu(buff, bit, 3, header->smooth);
  bit += 3;
  return bit;
}

/** Write RTCM header for observation message types 1009..1012.
 *
 * The data message header will be written starting from byte zero of the
 * buffer. If the buffer also contains a frame header then be sure to pass a
 * pointer to the start of the data message rather than a pointer to the start
 * of the frame buffer. The RTCM observation header is 8 bytes (61 bits) long.
 *
 * If the Synchronous GNSS Message Flag is set to `0`, it means that no further
 * GNSS observables referenced to the same Epoch Time will be transmitted. This
 * enables the receiver to begin processing the data immediately after decoding
 * the message. If it is set to `1`, it means that the next message will
 * contain observables of another GNSS source referenced to the same Epoch
 * Time.
 *
 * Divergence-free Smoothing Indicator values:
 *
 * Indicator | Meaning
 * --------- | ----------------------------------
 *     0     | Divergence-free smoothing not used
 *     1     | Divergence-free smoothing used
 *
 * GLO Smoothing Interval indicator values are listed in RTCM 10403.1 Table
 * 3.4-4, reproduced here:
 *
 * Indicator | Smoothing Interval
 * --------- | ------------------
 *  000 (0)  |   No smoothing
 *  001 (1)  |   < 30 s
 *  010 (2)  |   30-60 s
 *  011 (3)  |   1-2 min
 *  100 (4)  |   2-4 min
 *  101 (5)  |   4-8 min
 *  110 (6)  |   >8 min
 *  111 (7)  |   Unlimited
 *
 * \param header pointer to the obs header to encode
 * \param num_sats number of satellites in message
 * \param buff A pointer to the RTCM data message buffer.
 */
static uint16_t rtcm3_write_glo_header(const rtcm_obs_header *header,
                                       uint8_t num_sats,
                                       uint8_t buff[]) {
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, header->msg_num);
  bit += 12;
  rtcm_setbitu(buff, bit, 12, header->stn_id);
  bit += 12;
  rtcm_setbitu(buff, bit, 27, (uint32_t)round(header->tow_ms));
  bit += 27;
  rtcm_setbitu(buff, bit, 1, header->sync);
  bit += 1;
  rtcm_setbitu(buff, bit, 5, num_sats);
  bit += 5;
  rtcm_setbitu(buff, bit, 1, header->div_free);
  bit += 1;
  rtcm_setbitu(buff, bit, 3, header->smooth);
  bit += 3;
  return bit;
}

uint16_t rtcm3_encode_1001(const rtcm_obs_message *msg_1001, uint8_t buff[]) {
  assert(msg_1001);
  uint16_t bit = 64; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1001->header.n_sat; i++) {
    if (msg_1001->sats[i].obs[L1_FREQ].flags.fields.valid_pr &&
        msg_1001->sats[i].obs[L1_FREQ].flags.fields.valid_cp) {
      rtcm_setbitu(buff, bit, 6, msg_1001->sats[i].svId);
      bit += 6;
      encode_basic_freq_data(&msg_1001->sats[i].obs[L1_FREQ],
                             L1_FREQ,
                             &msg_1001->sats[i].obs[L1_FREQ].pseudorange,
                             buff,
                             &bit);
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  /* Fill in the header */
  rtcm3_write_header(&msg_1001->header, num_sats, buff);

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

/** Encode an RTCMv3 message type 1002 (Extended L1-Only GPS RTK Observables)
 * Message type 1002 has length `64 + n_sat*74` bits. Returned message length
 * is rounded up to the nearest whole byte.
 *
 * \param buff A pointer to the RTCM data message buffer.
 * \param id Reference station ID (DF003).
 * \param t GPS time of epoch (DF004).
 * \param n_sat Number of GPS satellites included in the message (DF006).
 * \param nm Struct containing the observation.
 * \param sync Synchronous GNSS Flag (DF005).
 * \return The message length in bytes.
 */
uint16_t rtcm3_encode_1002(const rtcm_obs_message *msg_1002, uint8_t buff[]) {
  assert(msg_1002);
  uint16_t bit = 64; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1002->header.n_sat; i++) {
    if (msg_1002->sats[i].obs[L1_FREQ].flags.fields.valid_pr &&
        msg_1002->sats[i].obs[L1_FREQ].flags.fields.valid_cp) {
      rtcm_setbitu(buff, bit, 6, msg_1002->sats[i].svId);
      bit += 6;
      encode_basic_freq_data(&msg_1002->sats[i].obs[L1_FREQ],
                             L1_FREQ,
                             &msg_1002->sats[i].obs[L1_FREQ].pseudorange,
                             buff,
                             &bit);

      /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF014). */
      uint8_t amb =
          (uint8_t)(msg_1002->sats[i].obs[L1_FREQ].pseudorange / PRUNIT_GPS);

      rtcm_setbitu(buff, bit, 8, amb);
      bit += 8;
      rtcm_setbitu(buff,
                   bit,
                   8,
                   (uint8_t)round(msg_1002->sats[i].obs[L1_FREQ].cnr * 4.0));
      bit += 8;
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  rtcm3_write_header(&msg_1002->header, num_sats, buff);

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_1003(const rtcm_obs_message *msg_1003, uint8_t buff[]) {
  assert(msg_1003);
  uint16_t bit = 64; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1003->header.n_sat; i++) {
    flag_bf l1_flags = msg_1003->sats[i].obs[L1_FREQ].flags;
    flag_bf l2_flags = msg_1003->sats[i].obs[L2_FREQ].flags;
    if (l1_flags.fields.valid_pr && l1_flags.fields.valid_cp &&
        l2_flags.fields.valid_pr && l2_flags.fields.valid_cp) {
      rtcm_setbitu(buff, bit, 6, msg_1003->sats[i].svId);
      bit += 6;
      encode_basic_freq_data(&msg_1003->sats[i].obs[L1_FREQ],
                             L1_FREQ,
                             &msg_1003->sats[i].obs[L1_FREQ].pseudorange,
                             buff,
                             &bit);
      encode_basic_freq_data(&msg_1003->sats[i].obs[L2_FREQ],
                             L2_FREQ,
                             &msg_1003->sats[i].obs[L1_FREQ].pseudorange,
                             buff,
                             &bit);
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  rtcm3_write_header(&msg_1003->header, num_sats, buff);

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_1004(const rtcm_obs_message *msg_1004, uint8_t buff[]) {
  assert(msg_1004);
  uint16_t bit = 64; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1004->header.n_sat; i++) {
    flag_bf l1_flags = msg_1004->sats[i].obs[L1_FREQ].flags;
    if (l1_flags.fields.valid_pr && l1_flags.fields.valid_cp) {
      rtcm_setbitu(buff, bit, 6, msg_1004->sats[i].svId);
      bit += 6;
      encode_basic_freq_data(&msg_1004->sats[i].obs[L1_FREQ],
                             L1_FREQ,
                             &msg_1004->sats[i].obs[L1_FREQ].pseudorange,
                             buff,
                             &bit);

      /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF014). */
      uint8_t amb =
          (uint8_t)(msg_1004->sats[i].obs[L1_FREQ].pseudorange / PRUNIT_GPS);

      rtcm_setbitu(buff, bit, 8, amb);
      bit += 8;
      rtcm_setbitu(buff,
                   bit,
                   8,
                   (uint8_t)round(msg_1004->sats[i].obs[L1_FREQ].cnr * 4.0));
      bit += 8;

      encode_basic_freq_data(&msg_1004->sats[i].obs[L2_FREQ],
                             L2_FREQ,
                             &msg_1004->sats[i].obs[L1_FREQ].pseudorange,
                             buff,
                             &bit);
      rtcm_setbitu(buff,
                   bit,
                   8,
                   (uint8_t)round(msg_1004->sats[i].obs[L2_FREQ].cnr * 4.0));
      bit += 8;
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  rtcm3_write_header(&msg_1004->header, num_sats, buff);

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

static uint16_t rtcm3_encode_1005_base(const rtcm_msg_1005 *msg_1005,
                                       uint8_t buff[],
                                       uint16_t *bit) {
  rtcm_setbitu(buff, *bit, 12, msg_1005->stn_id);
  *bit += 12;
  rtcm_setbitu(buff, *bit, 6, msg_1005->ITRF);
  *bit += 6;
  rtcm_setbitu(buff, *bit, 1, msg_1005->GPS_ind);
  *bit += 1;
  rtcm_setbitu(buff, *bit, 1, msg_1005->GLO_ind);
  *bit += 1;
  rtcm_setbitu(buff, *bit, 1, msg_1005->GAL_ind);
  *bit += 1;
  rtcm_setbitu(buff, *bit, 1, msg_1005->ref_stn_ind);
  *bit += 1;
  rtcm_setbitsl(buff, *bit, 38, (int64_t)round(msg_1005->arp_x * 10000.0));
  *bit += 38;
  rtcm_setbitu(buff, *bit, 1, msg_1005->osc_ind);
  *bit += 1;
  rtcm_setbitu(buff, *bit, 1, 0);
  *bit += 1;
  rtcm_setbitsl(buff, *bit, 38, (int64_t)round(msg_1005->arp_y * 10000.0));
  *bit += 38;
  rtcm_setbitu(buff, *bit, 2, msg_1005->quart_cycle_ind);
  *bit += 2;
  rtcm_setbitsl(buff, *bit, 38, (int64_t)round(msg_1005->arp_z * 10000.0));
  *bit += 38;

  /* Round number of bits up to nearest whole byte. */
  return (*bit + 7) / 8;
}

uint16_t rtcm3_encode_1005(const rtcm_msg_1005 *msg_1005, uint8_t buff[]) {
  assert(msg_1005);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1005);
  bit += 12;
  return rtcm3_encode_1005_base(msg_1005, buff, &bit);
}

uint16_t rtcm3_encode_1006(const rtcm_msg_1006 *msg_1006, uint8_t buff[]) {
  assert(msg_1006);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1006);
  bit += 12;
  rtcm3_encode_1005_base(&msg_1006->msg_1005, buff, &bit);
  double ant_height = msg_1006->ant_height;
  if (ant_height < 0.0) {
    ant_height = 0.0;
  } else if (ant_height > RTCM_1006_MAX_ANTENNA_HEIGHT_M) {
    ant_height = RTCM_1006_MAX_ANTENNA_HEIGHT_M;
  }
  rtcm_setbitu(buff, bit, 16, (uint16_t)round(ant_height * 10000.0));
  bit += 16;

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

static uint16_t rtcm3_encode_1007_base(const rtcm_msg_1007 *msg_1007,
                                       uint8_t buff[],
                                       uint16_t *bit) {
  rtcm_setbitu(buff, *bit, 12, msg_1007->stn_id);
  *bit += 12;
  rtcm_setbitu(buff, *bit, 8, msg_1007->ant_descriptor_counter);
  *bit += 8;
  for (uint8_t i = 0; i < msg_1007->ant_descriptor_counter; ++i) {
    rtcm_setbitu(buff, *bit, 8, msg_1007->ant_descriptor[i]);
    *bit += 8;
  }
  rtcm_setbitu(buff, *bit, 8, msg_1007->ant_setup_id);
  *bit += 8;

  /* Round number of bits up to nearest whole byte. */
  return (*bit + 7) / 8;
}

uint16_t rtcm3_encode_1007(const rtcm_msg_1007 *msg_1007, uint8_t buff[]) {
  assert(msg_1007);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1007);
  bit += 12;
  return rtcm3_encode_1007_base(msg_1007, buff, &bit);
}

uint16_t rtcm3_encode_1008(const rtcm_msg_1008 *msg_1008, uint8_t buff[]) {
  assert(msg_1008);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1008);
  bit += 12;
  rtcm3_encode_1007_base(&msg_1008->msg_1007, buff, &bit);
  rtcm_setbitu(buff, bit, 8, msg_1008->ant_serial_num_counter);
  bit += 8;
  for (uint8_t i = 0; i < msg_1008->ant_serial_num_counter; ++i) {
    rtcm_setbitu(buff, bit, 8, msg_1008->ant_serial_num[i]);
    bit += 8;
  }

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_1010(const rtcm_obs_message *msg_1010, uint8_t buff[]) {
  assert(msg_1010);
  uint16_t bit = 61; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1010->header.n_sat; i++) {
    if (msg_1010->sats[i].obs[L1_FREQ].flags.fields.valid_pr &&
        msg_1010->sats[i].obs[L1_FREQ].flags.fields.valid_cp) {
      const rtcm_sat_data *sat_obs = &msg_1010->sats[i];
      rtcm_setbitu(buff, bit, 6, sat_obs->svId);
      bit += 6;
      encode_basic_glo_freq_data(&sat_obs->obs[L1_FREQ],
                                 L1_FREQ,
                                 &sat_obs->obs[L1_FREQ].pseudorange,
                                 sat_obs->fcn,
                                 buff,
                                 &bit);

      /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF014). */
      uint8_t amb = (uint8_t)(sat_obs->obs[L1_FREQ].pseudorange / PRUNIT_GLO);

      rtcm_setbitu(buff, bit, 7, amb);
      bit += 7;
      rtcm_setbitu(
          buff, bit, 8, (uint8_t)round(sat_obs->obs[L1_FREQ].cnr * 4.0));
      bit += 8;
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  rtcm3_write_glo_header(&msg_1010->header, num_sats, buff);

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_1012(const rtcm_obs_message *msg_1012, uint8_t buff[]) {
  assert(msg_1012);
  uint16_t bit = 61; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1012->header.n_sat; i++) {
    flag_bf l1_flags = msg_1012->sats[i].obs[L1_FREQ].flags;
    if (l1_flags.fields.valid_pr && l1_flags.fields.valid_cp) {
      const rtcm_sat_data *sat_obs = &msg_1012->sats[i];
      rtcm_setbitu(buff, bit, 6, sat_obs->svId);
      bit += 6;
      encode_basic_glo_freq_data(&sat_obs->obs[L1_FREQ],
                                 L1_FREQ,
                                 &sat_obs->obs[L1_FREQ].pseudorange,
                                 sat_obs->fcn,
                                 buff,
                                 &bit);

      /* Calculate GLO Integer L1 Pseudorange Modulus Ambiguity (DF014). */
      uint8_t amb = (uint8_t)(sat_obs->obs[L1_FREQ].pseudorange / PRUNIT_GLO);

      rtcm_setbitu(buff, bit, 7, amb);
      bit += 7;
      rtcm_setbitu(
          buff, bit, 8, (uint8_t)round(sat_obs->obs[L1_FREQ].cnr * 4.0));
      bit += 8;

      encode_basic_glo_freq_data(&sat_obs->obs[L2_FREQ],
                                 L2_FREQ,
                                 &sat_obs->obs[L1_FREQ].pseudorange,
                                 sat_obs->fcn,
                                 buff,
                                 &bit);
      rtcm_setbitu(
          buff, bit, 8, (uint8_t)round(sat_obs->obs[L2_FREQ].cnr * 4.0));
      bit += 8;
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  rtcm3_write_glo_header(&msg_1012->header, num_sats, buff);

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_1029(const rtcm_msg_1029 *msg_1029, uint8_t buff[]) {
  assert(msg_1029);
  uint16_t bit = 0;

  rtcm_setbitu(buff, bit, 12, 1029);
  bit += 12;

  rtcm_setbitu(buff, bit, 12, msg_1029->stn_id);
  bit += 12;

  rtcm_setbitu(buff, bit, 16, msg_1029->mjd_num);
  bit += 16;

  rtcm_setbitu(buff, bit, 17, msg_1029->utc_sec_of_day);
  bit += 17;

  rtcm_setbitu(buff, bit, 7, msg_1029->unicode_chars);
  bit += 7;

  rtcm_setbitu(buff, bit, 8, msg_1029->utf8_code_units_n);
  bit += 8;
  for (uint8_t i = 0; i < msg_1029->utf8_code_units_n; i++) {
    rtcm_setbitu(buff, bit, 8, msg_1029->utf8_code_units[i]);
    bit += 8;
  }

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_1033(const rtcm_msg_1033 *msg_1033, uint8_t buff[]) {
  assert(msg_1033);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1033);
  bit += 12;

  rtcm_setbitu(buff, bit, 12, msg_1033->stn_id);
  bit += 12;

  rtcm_setbitu(buff, bit, 8, msg_1033->ant_descriptor_counter);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->ant_descriptor_counter; ++i) {
    rtcm_setbitu(buff, bit, 8, msg_1033->ant_descriptor[i]);
    bit += 8;
  }

  rtcm_setbits(buff, bit, 8, msg_1033->ant_setup_id);
  bit += 8;

  rtcm_setbitu(buff, bit, 8, msg_1033->ant_serial_num_counter);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->ant_serial_num_counter; ++i) {
    rtcm_setbitu(buff, bit, 8, msg_1033->ant_serial_num[i]);
    bit += 8;
  }

  rtcm_setbitu(buff, bit, 8, msg_1033->rcv_descriptor_counter);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->rcv_descriptor_counter; ++i) {
    rtcm_setbitu(buff, bit, 8, msg_1033->rcv_descriptor[i]);
    bit += 8;
  }

  rtcm_setbitu(buff, bit, 8, msg_1033->rcv_fw_version_counter);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->rcv_fw_version_counter; ++i) {
    rtcm_setbitu(buff, bit, 8, msg_1033->rcv_fw_version[i]);
    bit += 8;
  }

  rtcm_setbitu(buff, bit, 8, msg_1033->rcv_serial_num_counter);
  bit += 8;
  for (uint8_t i = 0; i < msg_1033->rcv_serial_num_counter; ++i) {
    rtcm_setbitu(buff, bit, 8, msg_1033->rcv_serial_num[i]);
    bit += 8;
  }
  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

uint16_t rtcm3_encode_1230(const rtcm_msg_1230 *msg_1230, uint8_t buff[]) {
  assert(msg_1230);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 1230);
  bit += 12;
  rtcm_setbitu(buff, bit, 12, msg_1230->stn_id);
  bit += 12;
  rtcm_setbitu(buff, bit, 8, msg_1230->bias_indicator);
  bit += 1;
  /* 3 reserved bits */
  rtcm_setbitu(buff, bit, 3, 0);
  bit += 3;
  rtcm_setbitu(buff, bit, 4, msg_1230->fdma_signal_mask);
  bit += 4;
  if (msg_1230->fdma_signal_mask & 0x08) {
    int16_t bias = (int16_t)round(msg_1230->L1_CA_cpb_meter * 50);
    rtcm_setbits(buff, bit, 16, bias);
    bit += 16;
  }
  if (msg_1230->fdma_signal_mask & 0x04) {
    int16_t bias = (int16_t)round(msg_1230->L1_P_cpb_meter * 50);
    rtcm_setbits(buff, bit, 16, bias);
    bit += 16;
  }
  if (msg_1230->fdma_signal_mask & 0x02) {
    int16_t bias = (int16_t)round(msg_1230->L2_CA_cpb_meter * 50);
    rtcm_setbits(buff, bit, 16, bias);
    bit += 16;
  }
  if (msg_1230->fdma_signal_mask & 0x01) {
    int16_t bias = (int16_t)round(msg_1230->L2_P_cpb_meter * 50);
    rtcm_setbits(buff, bit, 16, bias);
    bit += 16;
  }

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

static uint16_t rtcm3_encode_msm_header(const rtcm_msm_header *header,
                                        const rtcm_constellation_t cons,
                                        uint8_t buff[]) {
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, header->msg_num);
  bit += 12;
  rtcm_setbitu(buff, bit, 12, header->stn_id);
  bit += 12;
  if (RTCM_CONSTELLATION_GLO == cons) {
    /* day of the week */
    uint8_t dow = (uint16_t)(header->tow_ms / (24 * 3600 * 1000));
    rtcm_setbitu(buff, bit, 3, dow);
    bit += 3;
    /* time of the day */
    uint32_t tod_ms = header->tow_ms - dow * 24 * 3600 * 1000;
    rtcm_setbitu(buff, bit, 27, tod_ms);
    bit += 27;
  } else {
    /* for other systems, epoch time is the time of week in ms */
    rtcm_setbitu(buff, bit, 30, header->tow_ms);
    bit += 30;
  }
  rtcm_setbitu(buff, bit, 1, header->multiple);
  bit += 1;
  rtcm_setbitu(buff, bit, 3, header->iods);
  bit += 3;
  rtcm_setbitu(buff, bit, 7, header->reserved);
  bit += 7;
  rtcm_setbitu(buff, bit, 2, header->steering);
  bit += 2;
  rtcm_setbitu(buff, bit, 2, header->ext_clock);
  bit += 2;
  rtcm_setbitu(buff, bit, 1, header->div_free);
  bit += 1;
  rtcm_setbitu(buff, bit, 3, header->smooth);
  bit += 3;

  for (uint8_t i = 0; i < MSM_SATELLITE_MASK_SIZE; i++) {
    rtcm_setbitu(buff, bit, 1, header->satellite_mask[i]);
    bit++;
  }
  for (uint8_t i = 0; i < MSM_SIGNAL_MASK_SIZE; i++) {
    rtcm_setbitu(buff, bit, 1, header->signal_mask[i]);
    bit++;
  }
  uint8_t num_sats =
      count_mask_values(MSM_SATELLITE_MASK_SIZE, header->satellite_mask);
  uint8_t num_sigs =
      count_mask_values(MSM_SIGNAL_MASK_SIZE, header->signal_mask);
  uint8_t cell_mask_size = num_sats * num_sigs;

  for (uint8_t i = 0; i < cell_mask_size; i++) {
    rtcm_setbitu(buff, bit, 1, header->cell_mask[i]);
    bit++;
  }

  return bit;
}

static void encode_msm_sat_data(const rtcm_msm_message *msg,
                                const uint8_t num_sats,
                                const msm_enum msm_type,
                                double rough_range_ms[num_sats],
                                double rough_rate_m_s[num_sats],
                                uint8_t buff[],
                                uint16_t *bit) {
  /* number of integer milliseconds, DF397 */
  uint8_t integer_ms[num_sats];
  for (uint8_t i = 0; i < num_sats; i++) {
    integer_ms[i] = (uint8_t)floor(msg->sats[i].rough_range_ms);
    rtcm_setbitu(buff, *bit, 8, integer_ms[i]);
    *bit += 8;
  }
  if (MSM5 == msm_type) {
    for (uint8_t i = 0; i < num_sats; i++) {
      rtcm_setbitu(buff, *bit, 4, msg->sats[i].glo_fcn);
      *bit += 4;
    }
  }

  /* rough range modulo 1 ms, DF398 */
  for (uint8_t i = 0; i < num_sats; i++) {
    double pr = msg->sats[i].rough_range_ms;
    /* remove integer ms part */
    double range_modulo_ms = pr - integer_ms[i];
    uint16_t range_modulo_encoded = (uint16_t)round(1024 * range_modulo_ms);
    rtcm_setbitu(buff, *bit, 10, range_modulo_encoded);
    *bit += 10;
    rough_range_ms[i] = integer_ms[i] + (double)range_modulo_encoded / 1024;
  }

  /* range rate, m/s, DF399*/
  if (MSM5 == msm_type) {
    for (uint8_t i = 0; i < num_sats; i++) {
      int16_t range_rate = (int16_t)msg->sats[i].rough_range_rate_m_s;
      rtcm_setbits(buff, *bit, 14, range_rate);
      *bit += 14;
      rough_rate_m_s[i] = range_rate;
    }
  }
}

static void encode_msm_fine_pseudoranges(const uint8_t num_cells,
                                         const double fine_pr_ms[],
                                         const flag_bf flags[],
                                         uint8_t buff[],
                                         uint16_t *bit) {
  /* DF400 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_pr && fabs(fine_pr_ms[i]) < C_1_2P10) {
      rtcm_setbits(buff, *bit, 15, (int16_t)round(fine_pr_ms[i] / C_1_2P24));
    } else {
      rtcm_setbits(buff, *bit, 15, MSM_PR_INVALID);
    }
    *bit += 15;
  }
}

static void encode_msm_fine_phaseranges(const uint8_t num_cells,
                                        const double fine_cp_ms[],
                                        const flag_bf flags[],
                                        uint8_t buff[],
                                        uint16_t *bit) {
  /* DF401 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_cp && fabs(fine_cp_ms[i]) < C_1_2P8) {
      rtcm_setbits(buff, *bit, 22, (int32_t)round(fine_cp_ms[i] / C_1_2P29));
    } else {
      rtcm_setbits(buff, *bit, 22, MSM_CP_INVALID);
    }
    *bit += 22;
  }
}

static void encode_msm_lock_times(const uint8_t num_cells,
                                  const double lock_time[],
                                  const flag_bf flags[],
                                  uint8_t buff[],
                                  uint16_t *bit) {
  /* DF402 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_lock) {
      rtcm_setbitu(buff, *bit, 4, rtcm3_encode_lock_time(lock_time[i]));
    } else {
      rtcm_setbitu(buff, *bit, 4, 0);
    }
    *bit += 4;
  }
}

static void encode_msm_hca_indicators(const uint8_t num_cells,
                                      const bool hca_indicator[],
                                      uint8_t buff[],
                                      uint16_t *bit) {
  /* DF420 */
  for (uint16_t i = 0; i < num_cells; i++) {
    rtcm_setbitu(buff, *bit, 1, hca_indicator[i]);
    *bit += 1;
  }
}

static void encode_msm_cnrs(const uint8_t num_cells,
                            const double cnr[],
                            const flag_bf flags[],
                            uint8_t buff[],
                            uint16_t *bit) {
  /* DF403 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_cnr) {
      rtcm_setbitu(buff, *bit, 6, (uint8_t)round(cnr[i]));
    } else {
      rtcm_setbitu(buff, *bit, 6, 0);
    }
    *bit += 6;
  }
}

static void encode_msm_fine_phaserangerates(const uint8_t num_cells,
                                            const double fine_range_rate_m_s[],
                                            const flag_bf flags[],
                                            uint8_t buff[],
                                            uint16_t *bit) {
  /* DF404 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_dop &&
        fabs(fine_range_rate_m_s[i]) < 0.0001 * C_2P14) {
      rtcm_setbits(
          buff, *bit, 15, (int16_t)round(fine_range_rate_m_s[i] / 0.0001));
    } else {
      rtcm_setbits(buff, *bit, 15, MSM_DOP_INVALID);
    }
    *bit += 15;
  }
}

/** MSM4/5 encoder
 *
 * \param msg The input RTCM message struct
 * \param buff Data buffer large enough to hold the message (at worst 742 bytes)
 *             (see RTCM 10403.3 Table 3.5-71)
 * \return Number of bytes written or 0 on failure
 */

static uint16_t rtcm3_encode_msm_internal(const rtcm_msm_message *msg,
                                          uint8_t buff[]) {
  const rtcm_msm_header *header = &msg->header;

  msm_enum msm_type = to_msm_type(header->msg_num);
  if (MSM4 != msm_type && MSM5 != msm_type) {
    /* Unexpected message type. */
    return 0;
  }

  rtcm_constellation_t cons = to_constellation(header->msg_num);

  if (RTCM_CONSTELLATION_INVALID == cons) {
    /* Invalid or unsupported constellation */
    return 0;
  }

  uint8_t num_sats =
      count_mask_values(MSM_SATELLITE_MASK_SIZE, header->satellite_mask);
  uint8_t num_sigs =
      count_mask_values(MSM_SIGNAL_MASK_SIZE, header->signal_mask);
  uint8_t cell_mask_size = num_sats * num_sigs;
  uint8_t num_cells = count_mask_values(cell_mask_size, header->cell_mask);

  if (num_sats * num_sigs > MSM_MAX_CELLS) {
    /* Too large cell mask, should already have been handled by caller */
    return 0;
  }

  /* Header */
  uint16_t bit = rtcm3_encode_msm_header(header, cons, buff);

  /* Satellite Data */

  double rough_range_ms[num_sats];
  double rough_rate_m_s[num_sats];

  /* Satellite data */
  encode_msm_sat_data(
      msg, num_sats, msm_type, rough_range_ms, rough_rate_m_s, buff, &bit);

  /* Signal Data */

  double fine_pr_ms[num_cells];
  double fine_cp_ms[num_cells];
  double lock_time[num_cells];
  bool hca_indicator[num_cells];
  double cnr[num_cells];
  double fine_range_rate_m_s[num_cells];
  flag_bf flags[num_cells];

  memset(fine_pr_ms, 0, sizeof(fine_pr_ms));
  memset(fine_cp_ms, 0, sizeof(fine_cp_ms));
  memset(lock_time, 0, sizeof(lock_time));
  memset(hca_indicator, 0, sizeof(hca_indicator));
  memset(cnr, 0, sizeof(cnr));
  memset(fine_range_rate_m_s, 0, sizeof(fine_range_rate_m_s));
  memset(flags, 0, sizeof(flags));

  uint8_t i = 0;
  for (uint8_t sat = 0; sat < num_sats; sat++) {
    for (uint8_t sig = 0; sig < num_sigs; sig++) {
      if (header->cell_mask[sat * num_sigs + sig]) {
        flags[i] = msg->signals[i].flags;
        if (flags[i].fields.valid_pr) {
          fine_pr_ms[i] = msg->signals[i].pseudorange_ms - rough_range_ms[sat];
        }
        if (flags[i].fields.valid_cp) {
          fine_cp_ms[i] =
              msg->signals[i].carrier_phase_ms - rough_range_ms[sat];
        }
        if (flags[i].fields.valid_lock) {
          lock_time[i] = msg->signals[i].lock_time_s;
        }
        hca_indicator[i] = msg->signals[i].hca_indicator;
        if (flags[i].fields.valid_cnr) {
          cnr[i] = msg->signals[i].cnr;
        } else {
          cnr[i] = 0;
        }
        if (MSM5 == msm_type && flags[i].fields.valid_dop) {
          fine_range_rate_m_s[i] =
              msg->signals[i].range_rate_m_s - rough_rate_m_s[sat];
        }
        i++;
      }
    }
  }

  encode_msm_fine_pseudoranges(num_cells, fine_pr_ms, flags, buff, &bit);
  encode_msm_fine_phaseranges(num_cells, fine_cp_ms, flags, buff, &bit);
  encode_msm_lock_times(num_cells, lock_time, flags, buff, &bit);
  encode_msm_hca_indicators(num_cells, hca_indicator, buff, &bit);
  encode_msm_cnrs(num_cells, cnr, flags, buff, &bit);
  if (MSM5 == msm_type) {
    encode_msm_fine_phaserangerates(
        num_cells, fine_range_rate_m_s, flags, buff, &bit);
  }

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}

/** MSM4 encoder
 *
 * \param msg The input RTCM message struct
 * \param buff Data buffer large enough to hold the message (at worst 742 bytes)
 *             (see RTCM 10403.3 Table 3.5-71)
 * \return Number of bytes written or 0 on failure
 */

uint16_t rtcm3_encode_msm4(const rtcm_msm_message *msg_msm4, uint8_t buff[]) {
  assert(msg_msm4);
  if (MSM4 != to_msm_type(msg_msm4->header.msg_num)) {
    return 0;
  }

  return rtcm3_encode_msm_internal(msg_msm4, buff);
}

/** MSM5 encoder
 *
 * \param msg The input RTCM message struct
 * \param buff Data buffer large enough to hold the message (at worst 742 bytes)
 *             (see RTCM 10403.3 Table 3.5-71)
 * \return Number of bytes written or 0 on failure
 */

uint16_t rtcm3_encode_msm5(const rtcm_msm_message *msg_msm5, uint8_t buff[]) {
  assert(msg_msm5);
  if (MSM5 != to_msm_type(msg_msm5->header.msg_num)) {
    return 0;
  }

  return rtcm3_encode_msm_internal(msg_msm5, buff);
}

/** Encode the Swift Proprietary Message
 *
 * \param msg The input RTCM message struct
 * \param buff Data buffer large enough to hold the message
 * \return Number of bytes written or 0 on failure
 */

uint16_t rtcm3_encode_4062(const rtcm_msg_swift_proprietary *msg,
                           uint8_t buff[]) {
  assert(msg);
  uint16_t bit = 0;
  rtcm_setbitu(buff, bit, 12, 4062);
  bit += 12;
  /* These 4 bits are currently reserved, and should always be 0 */
  rtcm_setbitu(buff, bit, 4, 0);
  bit += 4;
  rtcm_setbitu(buff, bit, 16, msg->msg_type);
  bit += 16;
  rtcm_setbitu(buff, bit, 16, msg->sender_id);
  bit += 16;
  rtcm_setbitu(buff, bit, 8, msg->len);
  bit += 8;

  for (uint8_t i = 0; i < msg->len; ++i) {
    rtcm_setbitu(buff, bit, 8, msg->data[i]);
    bit += 8;
  }

  /* Round number of bits up to nearest whole byte. */
  return (bit + 7) / 8;
}
