/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* These encoder functions are provided to allow for easier unit testing however
 * they are not robust to be used in production. Before using with real data, we
 * would need to handle ambiguity rollover and code carrier divergance at least
 */

#include <string.h>

#include <ubx/encode.h>

#include <swiftnav/bits.h>

// UBX protocol is little-endian.

/** Set bit field in buffer from an unsigned integer.
 * Packs `len` bits into bit position `pos` from the start of the buffer.
 * Maximum bit field length is 64 bits, i.e. `len <= 64`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \param data Unsigned integer to be packed into bit field.
 */
void ubx_setbitul(uint8_t *buff, uint32_t pos, uint32_t len, uint64_t data) {
  uint64_t mask = ((uint64_t)1) << (len - 1);

  if (len <= 0 || 64 < len) {
    return;
  }

  for (uint32_t i = pos; i < pos + len; i++, mask >>= 1) {
    if (data & mask) {
      buff[i / 8] |= ((uint64_t)1) << (7 - i % 8);
    } else {
      buff[i / 8] &= ~(((uint64_t)1) << (7 - i % 8));
    }
  }
}

/** Set bit field in buffer from a signed integer.
 * Packs `len` bits into bit position `pos` from the start of the buffer.
 * Maximum bit field length is 32 bits, i.e. `len <= 32`.
 *
 * \param buff
 * \param pos Position in buffer of start of bit field in bits.
 * \param len Length of bit field in bits.
 * \param data Signed integer to be packed into bit field.
 */
void ubx_setbitsl(uint8_t *buff, uint32_t pos, uint32_t len, int64_t data) {
  ubx_setbitul(buff, pos, len, (uint64_t)data);
}

/** Serialize the ubx_hnr_pvt message
 *
 * \param buff outgoing data buffer
 * \param msg_hnr_pvt UBX hnr pvt message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_hnr_pvt(const ubx_hnr_pvt *msg_hnr_pvt, uint8_t buff[]) {
  assert(msg_hnr_pvt);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_hnr_pvt->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_hnr_pvt->i_tow, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->year, 2);
  index += 2;
  memcpy(&buff[index], &msg_hnr_pvt->month, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->day, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->hour, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->min, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->sec, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->valid, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->nano, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->fix_type, 1);
  index += 1;
  memcpy(&buff[index], &msg_hnr_pvt->flags, 1);
  index += 1;
  /* reserved */
  for (int i = 0; i < 2; i++) {
    memcpy(&buff[index], &msg_hnr_pvt->reserved1[i], 1);
    index += 1;
  }
  memcpy(&buff[index], &msg_hnr_pvt->lon, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->lat, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->height, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->height_mean_sea_level, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->ground_speed, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->speed, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->heading_of_motion, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->heading_vehicle, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->horizontal_accuracy, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->vertical_accuracy, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->speed_acc, 4);
  index += 4;
  memcpy(&buff[index], &msg_hnr_pvt->heading_acc, 4);
  index += 4;
  /* reserved */
  for (int i = 0; i < 4; i++) {
    memcpy(&buff[index], &msg_hnr_pvt->reserved2[i], 1);
    index += 1;
  }
  return index;
}

/** Serialize the ubx_rxm_rawx message
 *
 * \param buff outgoing data buffer
 * \param msg_rawx UBX rawx message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_rawx(const ubx_rxm_rawx *msg_rawx, uint8_t buff[]) {
  assert(msg_rawx);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_rawx->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_rawx->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_rawx->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_rawx->rcv_tow, 8);
  index += 8;
  memcpy(&buff[index], &msg_rawx->rcv_wn, 2);
  index += 2;
  memcpy(&buff[index], &msg_rawx->leap_second, 1);
  index += 1;
  memcpy(&buff[index], &msg_rawx->num_meas, 1);
  index += 1;
  memcpy(&buff[index], &msg_rawx->rec_status, 1);
  index += 1;
  memcpy(&buff[index], &msg_rawx->version, 1);
  index += 1;
  /* reserved */
  for (int i = 0; i < 2; i++) {
    memcpy(&buff[index], &msg_rawx->reserved1[i], 1);
    index += 1;
  }

  for (int i = 0; i < msg_rawx->num_meas; i++) {
    memcpy(&buff[index], &msg_rawx->pseudorange_m[i], 8);
    index += 8;
    memcpy(&buff[index], &msg_rawx->carrier_phase_cycles[i], 8);
    index += 8;
    memcpy(&buff[index], &msg_rawx->doppler_hz[i], 4);
    index += 4;
    memcpy(&buff[index], &msg_rawx->gnss_id[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->sat_id[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->sig_id[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->freq_id[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->lock_time[i], 2);
    index += 2;
    memcpy(&buff[index], &msg_rawx->cno_dbhz[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->pr_std_m[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->cp_std_cycles[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->doppler_std_hz[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->track_state[i], 1);
    index += 1;
    memcpy(&buff[index], &msg_rawx->reserved2[i], 1);
    index += 1;
  }
  return index;
}

/** Serialize the ubx_nav_att message
 *
 * \param buff outgoing data buffer
 * \param msg_nav_att UBX nav att message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_nav_att(const ubx_nav_att *msg_nav_att, uint8_t buff[]) {
  assert(msg_nav_att);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_nav_att->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_att->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_att->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_nav_att->i_tow, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_att->version, 1);
  index += 1;
  /* reserved */
  for (int i = 0; i < 3; i++) {
    memcpy(&buff[index], &msg_nav_att->reserved[i], 1);
    index += 1;
  }
  memcpy(&buff[index], &msg_nav_att->roll, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_att->pitch, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_att->heading, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_att->acc_roll, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_att->acc_pitch, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_att->acc_heading, 4);
  index += 4;

  return index;
}

/** Serialize the ubx_nav_att message
 *
 * \param buff outgoing data buffer
 * \param msg_nav_clock UBX nav clock message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_nav_clock(const ubx_nav_clock *msg_nav_clock,
                              uint8_t buff[]) {
  assert(msg_nav_clock);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_nav_clock->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_clock->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_clock->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_nav_clock->i_tow, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_clock->clk_bias, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_clock->clk_drift, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_clock->time_acc, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_clock->freq_acc, 4);
  index += 4;

  return index;
}

/** Serialize the ubx_nav_pvt message
 *
 * \param buff outgoing data buffer
 * \param msg_nav_pvt UBX nav pvt message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_nav_pvt(const ubx_nav_pvt *msg_nav_pvt, uint8_t buff[]) {
  assert(msg_nav_pvt);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_nav_pvt->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_nav_pvt->i_tow, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->year, 2);
  index += 2;
  memcpy(&buff[index], &msg_nav_pvt->month, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->day, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->hour, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->min, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->sec, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->valid, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->time_acc, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->nano, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->fix_type, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->flags, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->flags2, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->num_sats, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_pvt->lon, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->lat, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->height, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->height_mean_sea_level, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->horizontal_accuracy, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->vertical_accuracy, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->vel_north, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->vel_east, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->vel_down, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->ground_speed, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->heading_of_motion, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->speed_acc, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->heading_acc, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->PDOP, 2);
  index += 2;
  memcpy(&buff[index], &msg_nav_pvt->flags3, 1);
  index += 1;
  /* reserved */
  for (int i = 0; i < 5; i++) {
    memcpy(&buff[index], &msg_nav_pvt->reserved1[i], 1);
    index += 1;
  }
  memcpy(&buff[index], &msg_nav_pvt->heading_vehicle, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_pvt->magnetic_declination, 2);
  index += 2;
  memcpy(&buff[index], &msg_nav_pvt->magnetic_declination_accuracy, 2);
  index += 2;
  return index;
}

/** Serialize the ubx_nav_velecef message
 *
 * \param buff outgoing data buffer
 * \param msg_nav_velecef UBX nav velecef message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_nav_velecef(const ubx_nav_velecef *msg_nav_velecef,
                                uint8_t buff[]) {
  assert(msg_nav_velecef);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_nav_velecef->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_velecef->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_velecef->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_nav_velecef->i_tow, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_velecef->ecefVX, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_velecef->ecefVY, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_velecef->ecefVZ, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_velecef->speed_acc, 4);
  index += 4;

  return index;
}

/** Serialize the ubx_nav_sat message
 *
 * \param buff outgoing data buffer
 * \param msg_nav_sat UBX nav sat message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_nav_sat(const ubx_nav_sat *msg_nav_sat, uint8_t buff[]) {
  assert(msg_nav_sat);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_nav_sat->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_sat->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_sat->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_nav_sat->i_tow, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_sat->version, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_sat->num_svs, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_sat->reserved1, 2);
  index += 2;

  for (int i = 0; i < msg_nav_sat->num_svs; i++) {
    if (i >= NAV_DATA_MAX_COUNT) {
      break;
    }

    const ubx_nav_sat_data *data = &msg_nav_sat->data[i];

    memcpy(&buff[index], &data->gnss_id, 1);
    index += 1;
    memcpy(&buff[index], &data->sv_id, 1);
    index += 1;
    memcpy(&buff[index], &data->cno, 1);
    index += 1;
    memcpy(&buff[index], &data->elev, 1);
    index += 1;
    memcpy(&buff[index], &data->azim, 2);
    index += 2;
    memcpy(&buff[index], &data->pr_res, 2);
    index += 2;
    memcpy(&buff[index], &data->flags, 4);
    index += 4;
  }

  return index;
}

/** Serialize the ubx_nav_status message
 *
 * \param buff outgoing data buffer
 * \param msg_nav_status UBX nav status message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_nav_status(const ubx_nav_status *msg_nav_status,
                               uint8_t buff[]) {
  assert(msg_nav_status);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_nav_status->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_status->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_status->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_nav_status->i_tow, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_status->fix_type, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_status->status_flags, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_status->fix_status, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_status->status_flags_ext, 1);
  index += 1;
  memcpy(&buff[index], &msg_nav_status->ttff_ms, 4);
  index += 4;
  memcpy(&buff[index], &msg_nav_status->msss, 4);
  index += 4;

  return index;
}

/** Serialize the ubx_mga_gps_eph message
 *
 * \param buff outgoing data buffer
 * \param msg_mga_gps_eph UBX gps eph message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_mga_gps_eph(const ubx_mga_gps_eph *msg_mga_gps_eph,
                                uint8_t buff[]) {
  assert(msg_mga_gps_eph);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_mga_gps_eph->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->length, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->msg_type, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->version, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->sat_id, 1);
  index += 1;
  /* reserved */
  memcpy(&buff[index], &msg_mga_gps_eph->reserved1, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->fit_interval, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->ura_index, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->sat_health, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->tgd, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->iodc, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->toc, 2);
  index += 2;
  /* reserved */
  memcpy(&buff[index], &msg_mga_gps_eph->reserved2, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->af2, 1);
  index += 1;
  memcpy(&buff[index], &msg_mga_gps_eph->af1, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->af0, 4);
  index += 4;
  memcpy(&buff[index], &msg_mga_gps_eph->crs, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->delta_N, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->m0, 4);
  index += 4;
  memcpy(&buff[index], &msg_mga_gps_eph->cuc, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->cus, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->e, 4);
  index += 4;
  memcpy(&buff[index], &msg_mga_gps_eph->sqrt_A, 4);
  index += 4;
  memcpy(&buff[index], &msg_mga_gps_eph->toe, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->cic, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->omega0, 4);
  index += 4;
  memcpy(&buff[index], &msg_mga_gps_eph->cis, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->crc, 2);
  index += 2;
  memcpy(&buff[index], &msg_mga_gps_eph->i0, 4);
  index += 4;
  memcpy(&buff[index], &msg_mga_gps_eph->omega, 4);
  index += 4;
  memcpy(&buff[index], &msg_mga_gps_eph->omega_dot, 4);
  index += 4;
  memcpy(&buff[index], &msg_mga_gps_eph->i_dot, 2);
  index += 2;
  /* reserved */
  for (int i = 0; i < 2; i++) {
    memcpy(&buff[index], &msg_mga_gps_eph->reserved3[i], 1);
    index += 1;
  }

  return index;
}

/** Serialize the ubx_rxm_sfrbx message
 *
 * \param buff outgoing data buffer
 * \param msg_rxm_sfrbx UBX rxm sfrbx message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_rxm_sfrbx(const ubx_rxm_sfrbx *msg_rxm_sfrbx,
                              uint8_t buff[]) {
  assert(msg_rxm_sfrbx);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_rxm_sfrbx->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_rxm_sfrbx->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_rxm_sfrbx->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_rxm_sfrbx->gnss_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_rxm_sfrbx->sat_id, 1);
  index += 1;
  /* reserved */
  memcpy(&buff[index], &msg_rxm_sfrbx->reserved1, 1);
  index += 1;
  memcpy(&buff[index], &msg_rxm_sfrbx->freq_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_rxm_sfrbx->num_words, 1);
  index += 1;
  memcpy(&buff[index], &msg_rxm_sfrbx->channel, 1);
  index += 1;
  memcpy(&buff[index], &msg_rxm_sfrbx->version, 1);
  index += 1;
  /* reserved */
  memcpy(&buff[index], &msg_rxm_sfrbx->reserved2, 1);
  index += 1;
  memset(&buff[index], 0, sizeof(msg_rxm_sfrbx->data_words));
  for (int i = 0; i < msg_rxm_sfrbx->num_words; i++) {
    memcpy(&buff[index], &msg_rxm_sfrbx->data_words[i], 4);
    index += 4;
  }

  return index;
}

/** Serialize the ubx_esf_ins message
 *
 * \param buff outgoing data buffer
 * \param msg_esf_ins UBX esf ins message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_esf_ins(const ubx_esf_ins *msg_esf_ins, uint8_t buff[]) {
  assert(msg_esf_ins);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_esf_ins->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_esf_ins->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_esf_ins->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_esf_ins->bitfield0, 4);
  index += 4;

  /* reserved */
  for (int i = 0; i < 4; i++) {
    memcpy(&buff[index], &msg_esf_ins->reserved1[i], 1);
    index += 1;
  }

  memcpy(&buff[index], &msg_esf_ins->i_tow, 4);
  index += 4;
  memcpy(&buff[index], &msg_esf_ins->x_ang_rate, 4);
  index += 4;
  memcpy(&buff[index], &msg_esf_ins->y_ang_rate, 4);
  index += 4;
  memcpy(&buff[index], &msg_esf_ins->z_ang_rate, 4);
  index += 4;
  memcpy(&buff[index], &msg_esf_ins->x_accel, 4);
  index += 4;
  memcpy(&buff[index], &msg_esf_ins->y_accel, 4);
  index += 4;
  memcpy(&buff[index], &msg_esf_ins->z_accel, 4);
  index += 4;

  return index;
}

/** Serialize the ubx_esf_meas message
 *
 * \param buff outgoing data buffer
 * \param msg_esf_meas UBX esf meas message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_esf_meas(const ubx_esf_meas *msg_esf_meas, uint8_t buff[]) {
  assert(msg_esf_meas);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_esf_meas->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_esf_meas->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_esf_meas->length, 2);
  index += 2;

  memcpy(&buff[index], &msg_esf_meas->time_tag, 4);
  index += 4;
  memcpy(&buff[index], &msg_esf_meas->flags, 2);
  index += 2;
  memcpy(&buff[index], &msg_esf_meas->id, 2);
  index += 2;

  u8 num_meas = (msg_esf_meas->flags >> 11) & 0x1F;
  for (int i = 0; i < num_meas; i++) {
    memcpy(&buff[index], &msg_esf_meas->data[i], 4);
    index += 4;
  }

  bool has_calib = msg_esf_meas->flags & 0x8;
  if (has_calib) {
    memcpy(&buff[index], &msg_esf_meas->calib_tag, 4);
    index += 4;
  }

  return index;
}

/** Serialize the ubx_esf_raw message
 *
 * \param buff outgoing data buffer
 * \param msg_esf_raw UBX esf raw message to serialize
 * \return number of bytes serialized
 */
uint16_t ubx_encode_esf_raw(const ubx_esf_raw *msg_esf_raw, uint8_t buff[]) {
  assert(msg_esf_raw);

  uint16_t index = 0;
  memcpy(&buff[index], &msg_esf_raw->class_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_esf_raw->msg_id, 1);
  index += 1;
  memcpy(&buff[index], &msg_esf_raw->length, 2);
  index += 2;

  /* reserved */
  memcpy(&buff[index], &msg_esf_raw->msss, 4);
  index += 4;

  for (int i = 0; i < (msg_esf_raw->length - 4) / 8; i++) {
    memcpy(&buff[index], &msg_esf_raw->data[i], 4);
    index += 4;
    memcpy(&buff[index], &msg_esf_raw->sensor_time_tag[i], 4);
    index += 4;
  }
  return index;
}
