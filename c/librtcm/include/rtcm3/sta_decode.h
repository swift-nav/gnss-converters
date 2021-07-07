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

#ifndef SWIFTNAV_DECODE_STA_H
#define SWIFTNAV_DECODE_STA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rtcm3/messages.h>

#define RF_STATUS_CW_MASK 0x1
#define RF_STATUS_AGC_MASK 0x2
#define RF_STATUS_IFM_MASK 0x4
#define RF_STATUS_NOISE_FLOOR_MASK 0x8
#define RF_STATUS_ANTENNA_SENSE_MASK 0x10

/* see ST AN GNSS Proprietary Interface (fw version 5.8.11) */
typedef struct {
  uint16_t msg_num;   /* Msg Num DF002 uint16 12*/
  uint8_t subtype_id; /* Msg Num DF002p uint8 8 */
  uint32_t gnss_epoch_time;
  uint8_t status_mask;
  uint8_t cw_detect_mask;
  uint8_t cw_notch_mask;
  uint32_t cw_constellation_mask;
  uint8_t cw_candidate_mask;
  uint32_t cw_power_array[8];
  uint8_t agc_mask;
  uint16_t agc_array[4];
  uint8_t ifm_mask;
  uint32_t ifm_constellation_mask;
  uint32_t noise_floor_ddc_array[6];
  uint8_t antenna_sense_power_switch;
  uint8_t antenna_sense_op_mode;
  uint8_t antenna_sense_rf_path;
  uint8_t antenna_sense_status;
} sta_rf_status;

typedef struct {
  uint16_t msg_num;   /* Msg Num DF002 uint16 12*/
  uint8_t subtype_id; /* Msg Num DF002p uint8 8 */
  uint16_t ref_id;
  uint8_t ITRF_year;
  uint8_t fix_mode;
  uint8_t n_sats_used;
  uint8_t n_sats_view;
  uint8_t hdop;
  uint8_t vdop;
  uint8_t pdop;
  uint16_t geoid_sep;
  uint32_t age_corrections;
  uint16_t ref_station_id;
  uint8_t gnss_id;
  uint32_t gnss_epoch_time;
  uint16_t week_number;
  uint8_t leap_seconds;
  double pos_ECEF_x;
  double pos_ECEF_y;
  double pos_ECEF_z;
  uint32_t vel_ECEF_x;
  uint32_t vel_ECEF_y;
  uint32_t vel_ECEF_z;
} sta_pvt;

#include <rtcm3/messages.h>

rtcm3_rc sta_decode_fwver(const uint8_t buff[], char *fw_ver, uint8_t len);
rtcm3_rc sta_decode_rfstatus(const uint8_t buff[],
                             sta_rf_status *rf_status,
                             uint16_t len);
rtcm3_rc sta_decode_rcc_config(const uint8_t buff[],
                               char *sta_config_buffer,
                               uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_DECODE_STA_H */
