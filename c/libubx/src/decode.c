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

#include <swiftnav/bits.h>
#include <ubx/decode.h>

#include "decode_helpers.h"

// UBX protocol is little-endian. This file assumes that the host system is also
// little endian.
// TODO(yizhe) write endian-independent code

/** Writes checksum over `length` bytes of `buf` into `CK_A` and `CK_B`. The
 * `length` includes class id, msg id, length bytes, and payload. Uses the 8-Bit
 * Fletcher Algorithm.
 *
 * \param buff Buffer containing a ubx message.
 * \param length Length of the ubx message
 * \param CK_A first byte of checksum
 * \param CK_B second byte of checksum
 */
void ubx_checksum(const uint8_t buff[], size_t length, uint8_t *checksum) {
  uint8_t *CK_A = checksum;
  uint8_t *CK_B = checksum + 1;
  *CK_A = 0;
  *CK_B = 0;
  for (size_t i = 0; i < length; i++) {
    *CK_A = *CK_A + buff[i];
    *CK_B = *CK_B + *CK_A;
  }
}

/** Deserialize the ubx_hnr_pvt message
 *
 * \param buff incoming data buffer
 * \param msg_hnr_pvt UBX hnr pvt message
 * \return UBX return code
 */
ubx_rc ubx_decode_hnr_pvt_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_hnr_pvt *msg_hnr_pvt) {
  assert(msg_hnr_pvt);

  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->msg_id, 1);

  if (msg_hnr_pvt->class_id != UBX_CLASS_HNR) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_hnr_pvt->msg_id != UBX_MSG_HNR_PVT) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->i_tow, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->year, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->month, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->day, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->hour, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->min, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->sec, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->valid, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->nano, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->fix_type, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->flags, 1);
  /* reserved */
  for (int i = 0; i < 2; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->reserved1[i], 1);
  }
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->lon, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->lat, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->height, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->height_mean_sea_level, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->ground_speed, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->speed, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->heading_of_motion, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->heading_vehicle, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->horizontal_accuracy, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->vertical_accuracy, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->speed_acc, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->heading_acc, 4);
  /* reserved */
  for (int i = 0; i < 4; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_hnr_pvt->reserved2[i], 1);
  }

  return RC_OK;
}

/** Deserialize the ubx_rxm_rawx message
 *
 * \param buff incoming data buffer
 * \param msg_rawx UBX rawx message
 * \return UBX return code
 */
ubx_rc ubx_decode_rxm_rawx_bytestream(swiftnav_bytestream_t *buff,
                                      ubx_rxm_rawx *msg_rawx) {
  assert(msg_rawx);
  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->msg_id, 1);

  if (msg_rawx->class_id != 0x02) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_rawx->msg_id != 0x15) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->rcv_tow, 8);
  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->rcv_wn, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->leap_second, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->num_meas, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->rec_status, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rawx->version, 1);
  for (int i = 0; i < 2; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->reserved1[i], 1);
  }

  for (int i = 0; i < msg_rawx->num_meas; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->pseudorange_m[i], 8);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->carrier_phase_cycles[i], 8);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->doppler_hz[i], 4);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->gnss_id[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->sat_id[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->sig_id[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->freq_id[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->lock_time[i], 2);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->cno_dbhz[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->pr_std_m[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->cp_std_cycles[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->doppler_std_hz[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->track_state[i], 1);
    BYTESTREAM_DECODE_BYTES(buff, msg_rawx->reserved2[i], 1);
  }
  return RC_OK;
}

/** Deserialize the ubx_nav_att message
 *
 * \param buff incoming data buffer
 * \param msg_nav_att UBX nav att message
 * \return UBX return code
 */
ubx_rc ubx_decode_nav_att_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_nav_att *msg_nav_att) {
  assert(msg_nav_att);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->msg_id, 1);

  if (msg_nav_att->class_id != UBX_CLASS_NAV) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_nav_att->msg_id != UBX_MSG_NAV_ATT) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->i_tow, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->version, 1);
  /* reserved */
  for (int i = 0; i < 3; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->reserved[i], 1);
  }
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->roll, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->pitch, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->heading, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->acc_roll, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->acc_pitch, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_att->acc_heading, 4);

  return RC_OK;
}

/** Deserialize the ubx_nav_clock message
 *
 * \param buff incoming data buffer
 * \param msg_nav_clock UBX nav clock message
 * \return UBX return code
 */
ubx_rc ubx_decode_nav_clock_bytestream(swiftnav_bytestream_t *buff,
                                       ubx_nav_clock *msg_nav_clock) {
  assert(msg_nav_clock);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_clock->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_clock->msg_id, 1);

  if (msg_nav_clock->class_id != UBX_CLASS_NAV) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_nav_clock->msg_id != UBX_MSG_NAV_CLOCK) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_clock->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_clock->i_tow, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_clock->clk_bias, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_clock->clk_drift, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_clock->time_acc, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_clock->freq_acc, 4);

  return RC_OK;
}

/** Deserialize the ubx_nav_pvt message
 *
 * \param buff incoming data buffer
 * \param msg_nav_pvt UBX nav pvt message
 * \return UBX return code
 */
ubx_rc ubx_decode_nav_pvt_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_nav_pvt *msg_nav_pvt) {
  assert(msg_nav_pvt);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->msg_id, 1);

  if (msg_nav_pvt->class_id != UBX_CLASS_NAV) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_nav_pvt->msg_id != UBX_MSG_NAV_PVT) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->i_tow, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->year, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->month, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->day, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->hour, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->min, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->sec, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->valid, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->time_acc, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->nano, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->fix_type, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->flags, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->flags2, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->num_sats, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->lon, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->lat, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->height, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->height_mean_sea_level, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->horizontal_accuracy, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->vertical_accuracy, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->vel_north, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->vel_east, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->vel_down, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->ground_speed, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->heading_of_motion, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->speed_acc, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->heading_acc, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->PDOP, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->flags3, 1);
  /* reserved */
  for (int i = 0; i < 5; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->reserved1[i], 1);
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->heading_vehicle, 4);
  // if (msg_nav_pvt->flags & 0x20)
  {
    BYTESTREAM_DECODE_BYTES(buff, msg_nav_pvt->magnetic_declination, 2);
    BYTESTREAM_DECODE_BYTES(
        buff, msg_nav_pvt->magnetic_declination_accuracy, 2);
  }
  return RC_OK;
}

/** Deserialize the ubx_nav_velecef message
 *
 * \param buff incoming data buffer
 * \param msg_nav_velecef UBX nav velecef message
 * \return UBX return code
 */
ubx_rc ubx_decode_nav_velecef_bytestream(swiftnav_bytestream_t *buff,
                                         ubx_nav_velecef *msg_nav_velecef) {
  assert(msg_nav_velecef);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_velecef->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_velecef->msg_id, 1);

  if (msg_nav_velecef->class_id != UBX_CLASS_NAV) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_nav_velecef->msg_id != UBX_MSG_NAV_VELECEF) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_velecef->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_velecef->i_tow, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_velecef->ecefVX, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_velecef->ecefVY, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_velecef->ecefVZ, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_velecef->speed_acc, 4);

  return RC_OK;
}

/** Deserialize the ubx_nav_sat message
 *
 * \param buff incoming data buffer
 * \param msg_nav_sat UBX nav sat message
 * \return UBX return code
 */
ubx_rc ubx_decode_nav_sat_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_nav_sat *msg_nav_sat) {
  assert(msg_nav_sat);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_sat->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_sat->msg_id, 1);

  if (msg_nav_sat->class_id != UBX_CLASS_NAV) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_nav_sat->msg_id != UBX_MSG_NAV_SAT) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_sat->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_sat->i_tow, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_sat->version, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_sat->num_svs, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_sat->reserved1, 2);

  for (int i = 0; i < msg_nav_sat->num_svs; i++) {
    if (i >= NAV_DATA_MAX_COUNT) {
      break;
    }

    ubx_nav_sat_data *data = &msg_nav_sat->data[i];

    BYTESTREAM_DECODE_BYTES(buff, data->gnss_id, 1);
    BYTESTREAM_DECODE_BYTES(buff, data->sv_id, 1);
    BYTESTREAM_DECODE_BYTES(buff, data->cno, 1);
    BYTESTREAM_DECODE_BYTES(buff, data->elev, 1);
    BYTESTREAM_DECODE_BYTES(buff, data->azim, 2);
    BYTESTREAM_DECODE_BYTES(buff, data->pr_res, 2);
    BYTESTREAM_DECODE_BYTES(buff, data->flags, 4);
  }

  return RC_OK;
}

/** Deserialize the ubx_nav_status message
 *
 * \param buff incoming data buffer
 * \param msg_nav_status UBX nav status message
 * \return UBX return code
 */
ubx_rc ubx_decode_nav_status_bytestream(swiftnav_bytestream_t *buff,
                                        ubx_nav_status *msg_nav_status) {
  assert(msg_nav_status);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->msg_id, 1);

  if (msg_nav_status->class_id != UBX_CLASS_NAV) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_nav_status->msg_id != UBX_MSG_NAV_STATUS) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->i_tow, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->fix_type, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->status_flags, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->fix_status, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->status_flags_ext, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->ttff_ms, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_nav_status->msss, 4);

  return RC_OK;
}

/** Deserialize the ubx_mga_gps_eph message
 *
 * \param buff incoming data buffer
 * \param ubx_mga_gps_eph UBX mga gps eph message
 * \return UBX return code
 */
ubx_rc ubx_decode_mga_gps_eph_bytestream(swiftnav_bytestream_t *buff,
                                         ubx_mga_gps_eph *msg_mga_gps_eph) {
  assert(msg_mga_gps_eph);

  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->msg_id, 1);

  if (msg_mga_gps_eph->class_id != 0x13) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_mga_gps_eph->msg_id != 0x00) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->msg_type, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->version, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->sat_id, 1);
  /* reserved */
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->reserved1, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->fit_interval, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->ura_index, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->sat_health, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->tgd, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->iodc, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->toc, 2);
  /* reserved */
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->reserved2, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->af2, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->af1, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->af0, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->crs, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->delta_N, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->m0, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->cuc, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->cus, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->e, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->sqrt_A, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->toe, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->cic, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->omega0, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->cis, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->crc, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->i0, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->omega, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->omega_dot, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->i_dot, 2);
  /* reserved */
  for (int i = 0; i < 2; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_mga_gps_eph->reserved3[i], 1);
  }

  return RC_OK;
}

/** Deserialize the ubx_rxm_sfrbx message
 *
 * \param buff incoming data buffer
 * \param ubx_rxm_sfrbx UBX rxm sfrbx message
 * \return UBX return code
 */
ubx_rc ubx_decode_rxm_sfrbx_bytestream(swiftnav_bytestream_t *buff,
                                       ubx_rxm_sfrbx *msg_rxm_sfrbx) {
  assert(msg_rxm_sfrbx);

  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->msg_id, 1);

  if (msg_rxm_sfrbx->class_id != 0x02) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_rxm_sfrbx->msg_id != 0x13) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->gnss_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->sat_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->reserved1, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->freq_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->num_words, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->channel, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->version, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->reserved2, 1);

  if (msg_rxm_sfrbx->num_words > UBX_RXM_SFRBX_MAX_DATA_WORDS) {
    return RC_INVALID_MESSAGE;
  }

  for (int i = 0; i < msg_rxm_sfrbx->num_words; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_rxm_sfrbx->data_words[i], 4);
  }

  return RC_OK;
}

/** Deserialize the ubx_esf_ins message
 *
 * \param buff incoming data buffer
 * \param msg_esf_ins UBX esf ins message
 * \return UBX return code
 */
ubx_rc ubx_decode_esf_ins_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_esf_ins *msg_esf_ins) {
  assert(msg_esf_ins);

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->msg_id, 1);

  if (msg_esf_ins->class_id != UBX_CLASS_ESF) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_esf_ins->msg_id != UBX_MSG_ESF_INS) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->bitfield0, 4);

  for (int i = 0; i < 4; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->reserved1[i], 1);
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->i_tow, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->x_ang_rate, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->y_ang_rate, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->z_ang_rate, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->x_accel, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->y_accel, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_ins->z_accel, 4);

  return RC_OK;
}

/** Deserialize the ubx_esf_meas message
 *
 * \param buff incoming data buffer
 * \param msg_esf_meas UBX esf meas message
 * \return UBX return code
 */
ubx_rc ubx_decode_esf_meas_bytestream(swiftnav_bytestream_t *buff,
                                      ubx_esf_meas *msg_esf_meas) {
  assert(msg_esf_meas);

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_meas->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_meas->msg_id, 1);

  if (msg_esf_meas->class_id != UBX_CLASS_ESF) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_esf_meas->msg_id != UBX_MSG_ESF_MEAS) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_meas->length, 2);

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_meas->time_tag, 4);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_meas->flags, 2);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_meas->id, 2);

  u8 num_meas = (msg_esf_meas->flags >> 11) & 0x1F;
  for (int i = 0; i < num_meas; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_esf_meas->data[i], 4);
  }

  bool has_calib = msg_esf_meas->flags & 0x8;
  if (has_calib) {
    BYTESTREAM_DECODE_BYTES(buff, msg_esf_meas->calib_tag, 4);
  }

  return RC_OK;
}

/** Deserialize the ubx_esf_raw message
 *
 * \param buff incoming data buffer
 * \param msg_esf_raw UBX esf raw message
 * \return UBX return code
 */
ubx_rc ubx_decode_esf_raw_bytestream(swiftnav_bytestream_t *buff,
                                     ubx_esf_raw *msg_esf_raw) {
  assert(msg_esf_raw);

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_raw->class_id, 1);
  BYTESTREAM_DECODE_BYTES(buff, msg_esf_raw->msg_id, 1);

  if (msg_esf_raw->class_id != UBX_CLASS_ESF) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_esf_raw->msg_id != UBX_MSG_ESF_RAW) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_raw->length, 2);
  int num_measurements = (msg_esf_raw->length - 4) / 8;
  if (num_measurements > ESF_DATA_MAX_COUNT) {
    return RC_INVALID_MESSAGE;
  }

  BYTESTREAM_DECODE_BYTES(buff, msg_esf_raw->msss, 4);

  for (int i = 0; i < num_measurements; i++) {
    BYTESTREAM_DECODE_BYTES(buff, msg_esf_raw->data[i], 4);

    BYTESTREAM_DECODE_BYTES(buff, msg_esf_raw->sensor_time_tag[i], 4);
  }

  return RC_OK;
}
