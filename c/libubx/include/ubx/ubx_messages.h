/*
 * Copyright (C) 2019,2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_UBX_MESSAGES_H
#define SWIFTNAV_UBX_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>
#include <ubx/constants.h>

/* return codes for the decoders */
typedef enum ubx_rc_e {
  RC_OK = 0,
  RC_MESSAGE_TYPE_MISMATCH = -1,
  RC_INVALID_MESSAGE = -2
} ubx_rc;

#define UBX_CLASS_HNR 0x28
#define UBX_MSG_HNR_PVT 0x00

#define UBX_CLASS_NAV 0x01
#define UBX_MSG_NAV_ATT 0x05
#define UBX_MSG_NAV_CLOCK 0x22
#define UBX_MSG_NAV_PVT 0x07
#define UBX_MSG_NAV_VELECEF 0x11
#define UBX_MSG_NAV_SAT 0x35
#define UBX_MSG_NAV_STATUS 0x03
/* Arbitrarily defined, accomodates 100 satellites */
#define NAV_DATA_MAX_COUNT 100

#define UBX_CLASS_RXM 0x02
#define UBX_MSG_RXM_RAWX 0x15
#define UBX_MSG_RXM_SFRBX 0x13 /* broadcast navigation data subframe */

#define UBX_CLASS_ESF 0x10
#define UBX_MSG_ESF_INS 0x15
#define UBX_MSG_ESF_MEAS 0x02
#define UBX_MSG_ESF_RAW 0x03
/* Arbitrarily defined, spec leaves this unbounded */
#define ESF_DATA_MAX_COUNT 128

/* For forwarding only, no decoding */
#define UBX_CLASS_MON 0x0A
#define UBX_MSG_MON_HW 0x09

/* Max number of data words in RXM-SFRBX */
#define UBX_RXM_SFRBX_MAX_DATA_WORDS 10

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t i_tow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  int32_t nano;
  uint8_t fix_type;
  uint8_t flags;
  uint8_t reserved1[2];
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t height_mean_sea_level;
  int32_t ground_speed;
  int32_t speed;
  int32_t heading_of_motion;
  int32_t heading_vehicle;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
  uint32_t speed_acc;
  uint32_t heading_acc;
  uint8_t reserved2[4];
} ubx_hnr_pvt;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  double rcv_tow;
  uint16_t rcv_wn;
  int8_t leap_second;
  uint8_t num_meas;
  uint8_t rec_status;
  uint8_t version;
  uint8_t reserved1[2];
  double pseudorange_m[UBX_MAX_NUM_OBS];
  double carrier_phase_cycles[UBX_MAX_NUM_OBS];
  float doppler_hz[UBX_MAX_NUM_OBS];
  uint8_t gnss_id[UBX_MAX_NUM_OBS];
  uint8_t sat_id[UBX_MAX_NUM_OBS];
  uint8_t sig_id[UBX_MAX_NUM_OBS];
  uint8_t freq_id[UBX_MAX_NUM_OBS];
  uint16_t lock_time[UBX_MAX_NUM_OBS];
  uint8_t cno_dbhz[UBX_MAX_NUM_OBS];
  uint8_t pr_std_m[UBX_MAX_NUM_OBS];
  uint8_t cp_std_cycles[UBX_MAX_NUM_OBS];
  uint8_t doppler_std_hz[UBX_MAX_NUM_OBS];
  uint8_t track_state[UBX_MAX_NUM_OBS];
  uint8_t reserved2[UBX_MAX_NUM_OBS];
} ubx_rxm_rawx;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint8_t msg_type;
  uint8_t version;
  uint8_t sat_id;
  uint8_t reserved1;
  uint8_t fit_interval;
  uint8_t ura_index;
  uint8_t sat_health;
  int8_t tgd;
  uint16_t iodc;
  uint16_t toc;
  uint8_t reserved2;
  int8_t af2;
  int16_t af1;
  int32_t af0;
  int16_t crs;
  int16_t delta_N;
  int32_t m0;
  int16_t cuc;
  int16_t cus;
  uint32_t e;
  uint32_t sqrt_A;
  uint16_t toe;
  int16_t cic;
  int32_t omega0;
  int16_t cis;
  int16_t crc;
  int32_t i0;
  int32_t omega;
  int32_t omega_dot;
  int16_t i_dot;
  uint8_t reserved3[2];
} ubx_mga_gps_eph;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t i_tow;
  uint8_t version;
  uint8_t reserved[3];
  int32_t roll;
  int32_t pitch;
  int32_t heading;
  uint32_t acc_roll;
  uint32_t acc_pitch;
  uint32_t acc_heading;
} ubx_nav_att;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t i_tow;
  int32_t clk_bias;
  int32_t clk_drift;
  uint32_t time_acc;
  uint32_t freq_acc;
} ubx_nav_clock;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t i_tow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t time_acc;
  int32_t nano;
  uint8_t fix_type;
  uint8_t flags;
  uint8_t flags2;
  uint8_t num_sats;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t height_mean_sea_level;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
  int32_t vel_north;
  int32_t vel_east;
  int32_t vel_down;
  int32_t ground_speed;
  int32_t heading_of_motion;
  uint32_t speed_acc;
  uint32_t heading_acc;
  uint16_t PDOP;
  uint8_t flags3;
  uint8_t reserved1[5];
  int32_t heading_vehicle;
  int16_t magnetic_declination;
  uint16_t magnetic_declination_accuracy;
} ubx_nav_pvt;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t i_tow;
  int32_t ecefVX;
  int32_t ecefVY;
  int32_t ecefVZ;
  uint32_t speed_acc;
} ubx_nav_velecef;

typedef struct {
  uint8_t gnss_id;
  uint8_t sv_id;
  uint8_t cno;
  int8_t elev;
  int16_t azim;
  int16_t pr_res;
  uint32_t flags;
} ubx_nav_sat_data;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t i_tow;
  uint8_t version;
  uint8_t num_svs;
  uint16_t reserved1;
  ubx_nav_sat_data data[NAV_DATA_MAX_COUNT];
} ubx_nav_sat;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t i_tow;
  uint8_t fix_type;
  uint8_t status_flags;
  uint8_t fix_status;
  uint8_t status_flags_ext;
  uint32_t ttff_ms;
  uint32_t msss;
} ubx_nav_status;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint8_t gnss_id;
  uint8_t sat_id;
  uint8_t reserved1;
  uint8_t freq_id;
  uint8_t num_words;
  uint8_t channel;
  uint8_t version;
  uint8_t reserved2;
  uint32_t data_words[UBX_RXM_SFRBX_MAX_DATA_WORDS];
} ubx_rxm_sfrbx;

typedef enum ESF_DATA_TYPE {
  ESF_NONE = 0,
  ESF_Z_AXIS_GYRO_ANG_RATE = 5,
  ESF_FRONT_LEFT_WHEEL_TICKS = 6,
  ESF_FRONT_RIGHT_WHEEL_TICKS = 7,
  ESF_REAR_LEFT_WHEEL_TICKS = 8,
  ESF_REAR_RIGHT_WHEEL_TICKS = 9,
  ESF_SINGLE_TICK = 10,
  ESF_SPEED = 11,
  ESF_GYRO_TEMP = 12,
  ESF_Y_AXIS_GYRO_ANG_RATE = 13,
  ESF_X_AXIS_GYRO_ANG_RATE = 14,
  ESF_X_AXIS_ACCEL_SPECIFIC_FORCE = 16,
  ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE = 17,
  ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE = 18,
} ESF_DATA_TYPE;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t bitfield0;
  uint8_t reserved1[4];
  uint32_t i_tow;
  int32_t x_ang_rate;
  int32_t y_ang_rate;
  int32_t z_ang_rate;
  int32_t x_accel;
  int32_t y_accel;
  int32_t z_accel;
} ubx_esf_ins;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t time_tag;
  uint16_t flags;
  uint16_t id;
  uint32_t data[ESF_DATA_MAX_COUNT];
  uint32_t calib_tag;
} ubx_esf_meas;

typedef struct {
  uint8_t class_id;
  uint8_t msg_id;
  uint16_t length;
  uint32_t msss;
  uint32_t data[ESF_DATA_MAX_COUNT];
  uint32_t sensor_time_tag[ESF_DATA_MAX_COUNT];
} ubx_esf_raw;

#endif /* SWIFTNAV_UBX_MESSAGES_H */
