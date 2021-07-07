/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtcm3/ssr_decode.h"
#include <assert.h>
#include <stdio.h>
#include "rtcm3/bits.h"
#include "rtcm3/msm_utils.h"
#include "swiftnav/bitstream.h"

#include "decode_helpers.h"

/** Get the numbers of bits for the  Epoch Time 1s field
 * \param constellation Message constellation
 * \return Number of bits
 */
rtcm3_rc get_number_of_bits_for_epoch_time(rtcm_constellation_t constellation,
                                           uint8_t *num_bit) {
  switch (constellation) {
    case RTCM_CONSTELLATION_GPS:
    case RTCM_CONSTELLATION_GAL:
    case RTCM_CONSTELLATION_BDS:
    case RTCM_CONSTELLATION_QZS:
    case RTCM_CONSTELLATION_SBAS: {
      *num_bit = 20;
      return RC_OK;
    }
    case RTCM_CONSTELLATION_GLO: {
      *num_bit = 17;
      return RC_OK;
    }
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      return RC_INVALID_MESSAGE;
  }
}

/** Get the numbers of bits for the sat ID field
 * \param constellation Message constellation
 * \return Number of bits
 */
rtcm3_rc get_number_of_bits_for_sat_id(rtcm_constellation_t constellation,
                                       uint8_t *num_bit) {
  switch (constellation) {
    case RTCM_CONSTELLATION_GPS:
    case RTCM_CONSTELLATION_GAL:
    case RTCM_CONSTELLATION_BDS:
    case RTCM_CONSTELLATION_SBAS: {
      *num_bit = 6;
      return RC_OK;
    }
    case RTCM_CONSTELLATION_GLO: {
      *num_bit = 5;
      return RC_OK;
    }
    case RTCM_CONSTELLATION_QZS: {
      *num_bit = 4;
      return RC_OK;
    }
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      return RC_INVALID_MESSAGE;
  }
}

/** Get the numbers of bits for the IODE field
 * \param constellation Message constellation
 * \return Number of bits
 */
enum rtcm3_rc_e get_number_of_bits_for_iode(
    const rtcm_constellation_t constellation, uint8_t *num_bit) {
  switch (constellation) {
    case RTCM_CONSTELLATION_GPS:
    case RTCM_CONSTELLATION_GLO:
    case RTCM_CONSTELLATION_QZS: {
      *num_bit = 8;
      return RC_OK;
    }
    case RTCM_CONSTELLATION_GAL:
    case RTCM_CONSTELLATION_BDS: {
      *num_bit = 10;
      return RC_OK;
    }
    case RTCM_CONSTELLATION_SBAS: {
      *num_bit = 9;
      return RC_OK;
    }
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      return RC_INVALID_MESSAGE;
  }
}

/*
 *            TYPE       GPS     GLOASS    GALILEO    QZSS     BEIDOU     SBAS
 *         ----------------------------------------------------------------------
 *
 *          SSR OBT   : 1057      1063      1240*     1246*     1258*       -
 *              CLK   : 1058      1064      1241*     1247*     1259*       -
 *              BIAS  : 1059      1065      1242*     1248*     1260*       -
 *              OBTCLK: 1060      1066      1243*     1249*     1261*       -
 *              URA   : 1061      1067      1244*     1250*     1262*       -
 *              HRCLK : 1062      1068      1245*     1251*     1263*       -
 *              PHBIAS: 1265*     1266*     1267*     1268*     1270*      1269*
 *                    (* means that these RTCM messages are still draft )
 */

bool is_ssr_orbit_clock_message(const uint16_t message_num) {
  return message_num == 1057 || message_num == 1060 || message_num == 1063 ||
         message_num == 1066 || message_num == 1240 || message_num == 1243 ||
         message_num == 1246 || message_num == 1249 || message_num == 1258 ||
         message_num == 1261;
}

bool is_ssr_orbit_message(const uint16_t message_num) {
  return message_num == 1057 || message_num == 1063 || message_num == 1240 ||
         message_num == 1246 || message_num == 1258;
}

bool is_ssr_clock_message(const uint16_t message_num) {
  return message_num == 1058 || message_num == 1064 || message_num == 1241 ||
         message_num == 1247 || message_num == 1259;
}

bool is_ssr_code_biases_message(const uint16_t message_num) {
  return message_num == 1059 || message_num == 1065 || message_num == 1242 ||
         message_num == 1248 || message_num == 1260;
}

bool is_ssr_phase_biases_message(const uint16_t message_num) {
  return message_num >= 1265 && message_num <= 1270;
}

enum rtcm3_rc_e decode_ssr_header(swiftnav_bitstream_t *buff,
                                  rtcm_msg_ssr_header *msg_header) {
  assert(msg_header);
  BITSTREAM_DECODE_U16(buff, msg_header->message_num, 12);
  uint8_t number_of_bits_for_epoch_time;
  if (!(RC_OK == get_number_of_bits_for_epoch_time(
                     to_constellation(msg_header->message_num),
                     &number_of_bits_for_epoch_time))) {
    return RC_INVALID_MESSAGE;
  }
  BITSTREAM_DECODE_U32(
      buff, msg_header->epoch_time, number_of_bits_for_epoch_time);
  msg_header->constellation = to_constellation(msg_header->message_num);

  BITSTREAM_DECODE_U8(buff, msg_header->update_interval, 4);
  BITSTREAM_DECODE_BOOL(buff, msg_header->multi_message, 1);
  if (is_ssr_orbit_clock_message(msg_header->message_num)) {
    BITSTREAM_DECODE_BOOL(buff, msg_header->sat_ref_datum, 1);
  }
  BITSTREAM_DECODE_U8(buff, msg_header->iod_ssr, 4);
  BITSTREAM_DECODE_U16(buff, msg_header->ssr_provider_id, 16);
  BITSTREAM_DECODE_U16(buff, msg_header->ssr_solution_id, 4);
  if (is_ssr_phase_biases_message(msg_header->message_num)) {
    BITSTREAM_DECODE_BOOL(buff, msg_header->dispersive_bias_consistency, 1);
    BITSTREAM_DECODE_BOOL(buff, msg_header->melbourne_wubbena_consistency, 1);
  }
  BITSTREAM_DECODE_U8(buff, msg_header->num_sats, 6);
  return RC_OK;
}

static rtcm3_rc decode_ssr_orbit(swiftnav_bitstream_t *buff,
                                 uint8_t constellation,
                                 rtcm_msg_ssr_orbit_corr *orbit) {
  uint8_t number_of_bits_for_iode;
  if (!(RC_OK ==
        get_number_of_bits_for_iode(constellation, &number_of_bits_for_iode))) {
    return RC_INVALID_MESSAGE;
  }

  BITSTREAM_DECODE_U16(buff, orbit->iode, number_of_bits_for_iode);
  // In ssr_1_gal_qzss_sbas_bds_v08u.pdf there are two IODE fields for BDS.
  // The first one is a 10 bit "BDS toe Modulo" "toe modulo 8192" which we put
  // into orbit->iode above. The second one is "BDS IOD" "IOD=mod(toe/720,240)"
  // which is the definition starling expects in orbit->iode so we overwrite
  // the previous value below.
  if (constellation == RTCM_CONSTELLATION_BDS) {
    BITSTREAM_DECODE_U16(buff, orbit->iode, 8);
    // In ssr_1_gal_qzss_sbas_bds_v08u.pdf there are two IODE fields for SBAS.
    // The first one is a 9 bit "SBAS t0 Modulo" "toe modulo 8192" which we put
    // into orbit->iode above. The second one is "SBAS IOD CRC"
    // "IOD=mod(toe/720,240)" which is put into orbit->iodcrc below.
  } else if (constellation == RTCM_CONSTELLATION_SBAS) {
    BITSTREAM_DECODE_U32(buff, orbit->iodcrc, 24);
  }

  BITSTREAM_DECODE_S32(buff, orbit->radial, 22);
  BITSTREAM_DECODE_S32(buff, orbit->along_track, 20);
  BITSTREAM_DECODE_S32(buff, orbit->cross_track, 20);
  BITSTREAM_DECODE_S32(buff, orbit->dot_radial, 21);
  BITSTREAM_DECODE_S32(buff, orbit->dot_along_track, 19);
  BITSTREAM_DECODE_S32(buff, orbit->dot_cross_track, 19);

  return RC_OK;
}

static rtcm3_rc decode_ssr_clock(swiftnav_bitstream_t *buff,
                                 rtcm_msg_ssr_clock_corr *clock) {
  BITSTREAM_DECODE_S32(buff, clock->c0, 22);
  BITSTREAM_DECODE_S32(buff, clock->c1, 21);
  BITSTREAM_DECODE_S32(buff, clock->c2, 27);
  return RC_OK;
}

static rtcm3_rc decode_satellite_id(swiftnav_bitstream_t *buff,
                                    uint8_t constellation,
                                    uint8_t *sat_id) {
  uint8_t number_of_bits_for_sat_id;
  if (!(RC_OK == get_number_of_bits_for_sat_id(constellation,
                                               &number_of_bits_for_sat_id))) {
    return RC_INVALID_MESSAGE;
  }

  BITSTREAM_DECODE_U8(buff, *sat_id, number_of_bits_for_sat_id);

  return RC_OK;
}

rtcm3_rc rtcm3_decode_orbit_bitstream(swiftnav_bitstream_t *buff,
                                      rtcm_msg_orbit *msg_orbit) {
  assert(msg_orbit);
  if (!(RC_OK == decode_ssr_header(buff, &msg_orbit->header))) {
    return RC_INVALID_MESSAGE;
  }

  if (!is_ssr_orbit_message(msg_orbit->header.message_num)) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  for (int sat_count = 0; sat_count < msg_orbit->header.num_sats; sat_count++) {
    rtcm_msg_ssr_orbit_corr *orbit = &msg_orbit->orbit[sat_count];

    uint8_t sat_id;
    if (!(RC_OK == decode_satellite_id(
                       buff, msg_orbit->header.constellation, &sat_id))) {
      return RC_INVALID_MESSAGE;
    }

    orbit->sat_id = sat_id;

    if (!(RC_OK ==
          decode_ssr_orbit(buff, msg_orbit->header.constellation, orbit))) {
      return RC_INVALID_MESSAGE;
    }
  }
  return RC_OK;
}

rtcm3_rc rtcm3_decode_clock_bitstream(swiftnav_bitstream_t *buff,
                                      rtcm_msg_clock *msg_clock) {
  assert(msg_clock);
  if (!(RC_OK == decode_ssr_header(buff, &msg_clock->header))) {
    return RC_INVALID_MESSAGE;
  }

  if (!is_ssr_clock_message(msg_clock->header.message_num)) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  for (int sat_count = 0; sat_count < msg_clock->header.num_sats; sat_count++) {
    rtcm_msg_ssr_clock_corr *clock = &msg_clock->clock[sat_count];

    uint8_t sat_id;
    if (!(RC_OK == decode_satellite_id(
                       buff, msg_clock->header.constellation, &sat_id))) {
      return RC_INVALID_MESSAGE;
    }

    clock->sat_id = sat_id;
    decode_ssr_clock(buff, clock);
  }
  return RC_OK;
}

/** Decode an RTCMv3 Combined SSR Orbit and Clock message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Unknown constellation
 */
rtcm3_rc rtcm3_decode_orbit_clock_bitstream(
    swiftnav_bitstream_t *buff, rtcm_msg_orbit_clock *msg_orbit_clock) {
  assert(msg_orbit_clock);
  if (!(RC_OK == decode_ssr_header(buff, &msg_orbit_clock->header))) {
    return RC_INVALID_MESSAGE;
  }

  if (!is_ssr_orbit_clock_message(msg_orbit_clock->header.message_num)) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  for (int sat_count = 0; sat_count < msg_orbit_clock->header.num_sats;
       sat_count++) {
    rtcm_msg_ssr_orbit_corr *orbit = &msg_orbit_clock->orbit[sat_count];
    rtcm_msg_ssr_clock_corr *clock = &msg_orbit_clock->clock[sat_count];

    uint8_t sat_id;
    if (!(RC_OK == decode_satellite_id(
                       buff, msg_orbit_clock->header.constellation, &sat_id))) {
      return RC_INVALID_MESSAGE;
    }

    orbit->sat_id = sat_id;
    clock->sat_id = sat_id;

    if (!(RC_OK == decode_ssr_orbit(
                       buff, msg_orbit_clock->header.constellation, orbit))) {
      return RC_INVALID_MESSAGE;
    }
    decode_ssr_clock(buff, clock);
  }
  return RC_OK;
}

/** Decode an RTCMv3 Code bias message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Unknown constellation
 */
rtcm3_rc rtcm3_decode_code_bias_bitstream(swiftnav_bitstream_t *buff,
                                          rtcm_msg_code_bias *msg_code_bias) {
  assert(msg_code_bias);
  if (!(RC_OK == decode_ssr_header(buff, &msg_code_bias->header))) {
    return RC_INVALID_MESSAGE;
  }

  if (!is_ssr_code_biases_message(msg_code_bias->header.message_num)) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  for (int i = 0; i < msg_code_bias->header.num_sats; i++) {
    rtcm_msg_ssr_code_bias_sat *sat = &msg_code_bias->sats[i];

    uint8_t number_of_bits_for_sat_id;
    if (!(RC_OK ==
          get_number_of_bits_for_sat_id(msg_code_bias->header.constellation,
                                        &number_of_bits_for_sat_id))) {
      return RC_INVALID_MESSAGE;
    }

    BITSTREAM_DECODE_U8(buff, sat->sat_id, number_of_bits_for_sat_id);

    BITSTREAM_DECODE_U8(buff, sat->num_code_biases, 5);

    for (int j = 0; j < sat->num_code_biases; j++) {
      BITSTREAM_DECODE_U8(buff, sat->signals[j].signal_id, 5);
      BITSTREAM_DECODE_S16(buff, sat->signals[j].code_bias, 14);
    }
  }
  return RC_OK;
}

/** Decode an RTCMv3 Phase bias message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Unknown constellation
 */
rtcm3_rc rtcm3_decode_phase_bias_bitstream(
    swiftnav_bitstream_t *buff, rtcm_msg_phase_bias *msg_phase_bias) {
  assert(msg_phase_bias);
  if (!(RC_OK == decode_ssr_header(buff, &msg_phase_bias->header))) {
    return RC_INVALID_MESSAGE;
  }

  if (!is_ssr_phase_biases_message(msg_phase_bias->header.message_num)) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  for (int i = 0; i < msg_phase_bias->header.num_sats; i++) {
    rtcm_msg_ssr_phase_bias_sat *sat = &msg_phase_bias->sats[i];

    uint8_t number_of_bits_for_sat_id;
    if (!(RC_OK ==
          get_number_of_bits_for_sat_id(msg_phase_bias->header.constellation,
                                        &number_of_bits_for_sat_id))) {
      return RC_INVALID_MESSAGE;
    }

    BITSTREAM_DECODE_U8(buff, sat->sat_id, number_of_bits_for_sat_id);

    BITSTREAM_DECODE_U8(buff, sat->num_phase_biases, 5);
    BITSTREAM_DECODE_U16(buff, sat->yaw_angle, 9);
    BITSTREAM_DECODE_S8(buff, sat->yaw_rate, 8);

    for (int j = 0; j < sat->num_phase_biases; j++) {
      BITSTREAM_DECODE_U8(buff, sat->signals[j].signal_id, 5);
      BITSTREAM_DECODE_BOOL(buff, sat->signals[j].integer_indicator, 1);
      BITSTREAM_DECODE_U8(buff, sat->signals[j].widelane_indicator, 2);
      BITSTREAM_DECODE_U8(buff, sat->signals[j].discontinuity_indicator, 4);
      BITSTREAM_DECODE_S32(buff, sat->signals[j].phase_bias, 20);
    }
  }
  return RC_OK;
}
