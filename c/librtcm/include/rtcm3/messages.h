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

#ifndef SWIFTNAV_RTCM3_MESSAGES_H
#define SWIFTNAV_RTCM3_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

#include "rtcm3/constants.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum freq_e { L1_FREQ, L2_FREQ, NUM_FREQS } freq_t;

typedef enum msm_type_e {
  MSM_UNKNOWN = 0,
  MSM1,
  MSM2,
  MSM3,
  MSM4,
  MSM5,
  MSM6,
  MSM7
} msm_enum;

/** Constellation identifier (aligns with constellation_t in libswiftnav) */
typedef enum rtcm_constellation_e {
  RTCM_CONSTELLATION_INVALID = -1,
  RTCM_CONSTELLATION_GPS,
  RTCM_CONSTELLATION_SBAS,
  RTCM_CONSTELLATION_GLO,
  RTCM_CONSTELLATION_BDS,
  RTCM_CONSTELLATION_QZS,
  RTCM_CONSTELLATION_GAL,
  RTCM_CONSTELLATION_COUNT,
} rtcm_constellation_t;

/* return codes for the decoders */
typedef enum rtcm3_rc_e {
  RC_OK = 0,
  RC_MESSAGE_TYPE_MISMATCH = -1,
  RC_INVALID_MESSAGE = -2
} rtcm3_rc;

typedef struct {
  uint16_t msg_num; /* Msg Num DF002 uint16 12*/
  uint16_t stn_id;  /* Station Id DF003 uint16 12*/
  uint32_t tow_ms;  /* GPS/GLO time of week DF004/DF034 uint32 30/27 */
  uint8_t sync;     /* Syncronous flag DF005 bit(1) 1 */
  uint8_t n_sat;    /* Number of satellites DF006 uint8 5 */
  uint8_t div_free; /* Divergance free flag DF007 bit(1) 1 */
  uint8_t smooth;   /* GPS Smoothing Interval DF008 bit(3) 3 */
} rtcm_obs_header;

typedef struct {
  uint16_t msg_num;  /* Msg Num DF002 uint16 12*/
  uint16_t stn_id;   /* Station Id DF003 uint16 12*/
  uint32_t tow_ms;   /* System-specific epoch time uint32 30 */
  uint8_t multiple;  /* Multiple Message Bit DF393 bit(1) 1 */
  uint8_t iods;      /* Issue of Data Station DF409 uint8 3 */
  uint8_t reserved;  /* Reserved DF001 bit(7) 7 */
  uint8_t steering;  /* Clock Steering Indicator DF411 uint2 2 */
  uint8_t ext_clock; /* External Clock Indicator DF412 uint2 2 */
  uint8_t div_free;  /* Divergance free flag DF417 bit(1) 1 */
  uint8_t smooth;    /* GPS Smoothing Interval DF418 bit(3) 3 */
  /* GNSS Satellite Mask DF394 bit(64) 64 */
  bool satellite_mask[MSM_SATELLITE_MASK_SIZE];
  /* GNSS Signal Mask DF395 bit(32) 32 */
  bool signal_mask[MSM_SIGNAL_MASK_SIZE];
  /* GNSS Cell Mask DF396 bit(X) (X<=64) */
  bool cell_mask[MSM_MAX_CELLS];
} rtcm_msm_header;

typedef union {
  uint8_t data;
  struct {
    uint8_t valid_pr : 1;
    uint8_t valid_cp : 1;
    uint8_t valid_cnr : 1;
    uint8_t valid_lock : 1;
    uint8_t valid_dop : 1;
  } fields;
} flag_bf;

typedef struct {
  uint8_t code;
  double pseudorange;
  double carrier_phase;
  double lock;
  double cnr;
  flag_bf flags;

} rtcm_freq_data;

typedef struct {
  uint8_t svId;
  uint8_t fcn;
  rtcm_freq_data obs[NUM_FREQS];
} rtcm_sat_data;

typedef struct {
  uint8_t glo_fcn;
  double rough_range_ms;
  double rough_range_rate_m_s;
} rtcm_msm_sat_data;

typedef struct {
  double pseudorange_ms;
  double carrier_phase_ms;
  double lock_time_s;
  bool hca_indicator;
  double cnr;
  flag_bf flags;
  double range_rate_m_s;
} rtcm_msm_signal_data;

typedef struct {
  rtcm_obs_header header;
  rtcm_sat_data sats[RTCM_MAX_SATS];
} rtcm_obs_message;

typedef struct {
  rtcm_msm_header header;
  rtcm_msm_sat_data sats[RTCM_MAX_SATS];
  rtcm_msm_signal_data signals[MSM_MAX_CELLS];
} rtcm_msm_message;

typedef struct {
  uint16_t stn_id;
  uint8_t ITRF;        /* Reserved for ITRF Realization Year DF021 uint6 6 */
  uint8_t GPS_ind;     /* GPS Indicator DF022 bit(1) 1 */
  uint8_t GLO_ind;     /* GLONASS Indicator DF023 bit(1) 1 */
  uint8_t GAL_ind;     /* Reserved for Galileo Indicator DF024 bit(1) 1 */
  uint8_t ref_stn_ind; /* Reference-Station Indicator DF141 bit(1) 1 */
  double arp_x;        /* Antenna Reference Point ECEF-X DF025 int38 38 */
  uint8_t osc_ind;     /* Single Receiver Oscillator Indicator DF142 bit(1) 1 */
  uint8_t reserved;    /* Reserved DF001 bit(1) 1 */
  double arp_y;        /* Antenna Reference Point ECEF-Y DF026 int38 38 */
  uint8_t quart_cycle_ind; /* Quarter Cycle Indicator DF364 bit(2) 2 */
  double arp_z;            /* Antenna Reference Point ECEF-Z DF027 int38 38 */
} rtcm_msg_1005;

typedef struct {
  rtcm_msg_1005 msg_1005;
  double ant_height; /* Antenna Height DF028 uint16 16 */
} rtcm_msg_1006;

typedef struct {
  uint16_t stn_id;                /* Reference Station ID DF003 uint12 12 */
  uint8_t ant_descriptor_counter; /* Descriptor Counter N DF029 uint8 8 */
  char ant_descriptor[RTCM_MAX_STRING_LEN + 1]; /* Antenna Descriptor DF030
                                                 char8(N) 8*N N <= 31 */
  uint8_t ant_setup_id; /* Antenna Setup ID DF031 uint8 8 */
} rtcm_msg_1007;

typedef struct {
  rtcm_msg_1007 msg_1007;
  uint8_t ant_serial_num_counter; /* Serial Number Counter M DF032 uint8 8 */
  char ant_serial_num[RTCM_MAX_STRING_LEN + 1]; /* Antenna Serial Number DF033
                                                 char8(M) 8*M M <= 31*/
} rtcm_msg_1008;

#define RTCM_1029_MAX_CODE_UNITS (255u)
typedef struct {
  uint16_t stn_id;
  uint16_t mjd_num;
  uint32_t utc_sec_of_day;
  uint8_t unicode_chars;
  uint8_t utf8_code_units_n;
  uint8_t utf8_code_units[RTCM_1029_MAX_CODE_UNITS];
} rtcm_msg_1029;

typedef struct {
  uint16_t stn_id;                /* Reference Station ID DF003 uint12 12 */
  uint8_t ant_descriptor_counter; /* Antenna Descriptor Counter N DF029 */
  char ant_descriptor[RTCM_MAX_STRING_LEN + 1]; /* Antenna Descriptor DF030 */
  uint8_t ant_setup_id;                         /* Antenna Setup ID DF031 */
  uint8_t ant_serial_num_counter; /* Antenna Serial Number Counter M DF032 */
  char
      ant_serial_num[RTCM_MAX_STRING_LEN + 1]; /* Antenna Serial Number DF033 */
  uint8_t rcv_descriptor_counter; /* Receiver Type Descriptor Counter I DF227 */
  char rcv_descriptor[RTCM_MAX_STRING_LEN +
                      1];         /* Receiver Type Descriptor DF228 */
  uint8_t rcv_fw_version_counter; /* Receiver Firmware Version Counter DF229 */
  char rcv_fw_version[RTCM_MAX_STRING_LEN +
                      1];         /* Receiver Firmware Version DF230*/
  uint8_t rcv_serial_num_counter; /* Receiver Serial Number Counter K DF231 */
  char rcv_serial_num[RTCM_MAX_STRING_LEN +
                      1]; /* Receiver Serial Number DF232 */
} rtcm_msg_1033;

typedef struct {
  uint16_t stn_id;
  uint8_t bias_indicator;
  uint8_t fdma_signal_mask;
  double L1_CA_cpb_meter;
  double L1_P_cpb_meter;
  double L2_CA_cpb_meter;
  double L2_P_cpb_meter;
} rtcm_msg_1230;

/** Structure containing the ephemeris for one satellite.
 *  NOTE: this size has to fit parameters for all constellations
 */
typedef struct {
  union {
    int32_t gps_s;
    int32_t qzss_s;
    int32_t bds_s[2];
    int32_t gal_s[2];
  } tgd;
  int32_t crc;
  int32_t crs;
  int32_t cuc;
  int32_t cus;
  int32_t cic;
  int32_t cis;
  int16_t dn;
  int32_t m0;
  uint32_t ecc;
  uint32_t sqrta;
  int32_t omega0;
  int32_t omegadot;
  int32_t w;
  int32_t inc;
  int16_t inc_dot;
  int32_t af0;
  int32_t af1;
  int16_t af2;
  uint32_t toc;
  uint16_t iodc;
  uint16_t iode;
  uint8_t codeL2;
  bool L2_data_bit;
} ephemeris_kepler_raw_rtcm_t;

/** Structure containing the SBAS ephemeris for one satellite. */
typedef struct {
  double pos[3];
  double vel[3];
  double acc[3];
  double a_gf0;
  double a_gf1;
} ephemeris_xyz_rtcm_t;

/** Structure containing the GLONASS ephemeris for one satellite. */
typedef struct {
  int16_t gamma;
  int32_t tau;
  int8_t d_tau;
  uint8_t t_b;
  int32_t pos[3];
  int32_t vel[3];
  int32_t acc[3];

  uint8_t fcn;
  uint8_t iod;
} ephemeris_glo_raw_rtcm_t;

/** Structure containing the ephemeris for one satellite. */
typedef struct {
  uint8_t sat_id;
  rtcm_constellation_t constellation;
  uint16_t wn;
  uint32_t toe;
  uint16_t ura;
  uint32_t fit_interval;
  uint8_t health_bits;
  union {
    ephemeris_kepler_raw_rtcm_t kepler;
    ephemeris_xyz_rtcm_t xyz;
    ephemeris_glo_raw_rtcm_t glo;
  } data;
} rtcm_msg_eph;

#define MAX_SSR_SATELLITES 64
#define MAX_SSR_SIGNALS 32

typedef struct {
  uint16_t message_num;
  uint32_t epoch_time;
  uint8_t constellation;
  uint8_t update_interval;
  bool multi_message;
  bool sat_ref_datum;
  uint8_t iod_ssr;
  uint16_t ssr_provider_id;
  uint16_t ssr_solution_id;
  bool dispersive_bias_consistency;
  bool melbourne_wubbena_consistency;
  uint8_t num_sats;
} rtcm_msg_ssr_header;

typedef struct {
  uint8_t sat_id;
  int32_t c0;
  int32_t c1;
  int32_t c2;
} rtcm_msg_ssr_clock_corr;

typedef struct {
  uint8_t sat_id;
  uint16_t iode;
  uint32_t iodcrc;
  int32_t radial;
  int32_t along_track;
  int32_t cross_track;
  int32_t dot_radial;
  int32_t dot_along_track;
  int32_t dot_cross_track;
} rtcm_msg_ssr_orbit_corr;

typedef struct {
  uint8_t signal_id;
  int16_t code_bias;
} rtcm_msg_ssr_code_bias_sig;

typedef struct {
  uint8_t sat_id;
  uint8_t num_code_biases;
  rtcm_msg_ssr_code_bias_sig signals[MAX_SSR_SIGNALS];
} rtcm_msg_ssr_code_bias_sat;

typedef struct {
  uint8_t signal_id;
  bool integer_indicator;
  uint8_t widelane_indicator;
  uint8_t discontinuity_indicator;
  int32_t phase_bias;
} rtcm_msg_ssr_phase_bias_sig;

typedef struct {
  uint8_t sat_id;
  uint8_t num_phase_biases;
  uint16_t yaw_angle;
  int8_t yaw_rate;
  rtcm_msg_ssr_phase_bias_sig signals[MAX_SSR_SIGNALS];
} rtcm_msg_ssr_phase_bias_sat;

typedef struct {
  rtcm_msg_ssr_header header;
  rtcm_msg_ssr_orbit_corr orbit[MAX_SSR_SATELLITES];
} rtcm_msg_orbit;

typedef struct {
  rtcm_msg_ssr_header header;
  rtcm_msg_ssr_clock_corr clock[MAX_SSR_SATELLITES];
} rtcm_msg_clock;

typedef struct {
  rtcm_msg_ssr_header header;
  rtcm_msg_ssr_orbit_corr orbit[MAX_SSR_SATELLITES];
  rtcm_msg_ssr_clock_corr clock[MAX_SSR_SATELLITES];
} rtcm_msg_orbit_clock;

typedef struct {
  rtcm_msg_ssr_header header;
  rtcm_msg_ssr_code_bias_sat sats[MAX_SSR_SATELLITES];
} rtcm_msg_code_bias;

typedef struct {
  rtcm_msg_ssr_header header;
  rtcm_msg_ssr_phase_bias_sat sats[MAX_SSR_SATELLITES];
} rtcm_msg_phase_bias;

/* Encodes a SBP message */
typedef struct {
  uint16_t msg_type;
  uint16_t sender_id;
  uint8_t len;
  uint8_t data[255];
} rtcm_msg_swift_proprietary;

#define NDF_SYS_GPS 0
#define NDF_SYS_GLO 1
#define NDF_SYS_GAL 2
#define NDF_SYS_SBAS 3
#define NDF_SYS_QZS 4
#define NDF_SYS_BDS 5

typedef struct {
  uint8_t sat_sys;      /* Satellite System uint8 4 */
  uint8_t sat_num;      /* Satellite Number uint8 6 */
                        /* index from MSM satellite mask bit field, see DF394 */
  uint8_t ext_sat_info; /* Extended Satellite Information uint8 4 */
                        /* Frequency Number + 7 for GLONASS */
  uint8_t sig_type;     /* Signal Type uint8 5 */
                        /* index from MSM signal mask bit field, see DF395 */
  uint32_t epoch_time;  /* Epoch Time uint32 30 */
  /* in milliseconds, equal to MSM */
  bool continuous_tracking;      /* Continuous Tracking bool 1 */
  uint16_t frame_data_size_bits; /* Frame Data Size in bits uint16 12 */
  uint32_t frame_data[MAX_NDF_FRAME_SIZE_WORDS]; /* Frame Data uint32 */
} rtcm_ndf_frame;

/* Navigation Data Frame: encodes raw navigation data bits */
/* TODO add reference to def */
typedef struct {
  uint16_t msg_type;   /* Msg Num DF002 uint16 12 */
  uint16_t stn_id;     /* Reference Station ID uint16 12 */
  uint8_t frame_count; /* Frame Count uint8 6 */
  rtcm_ndf_frame frames[MAX_NDF_FRAMES];
} rtcm_msg_ndf;

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_RTCM3_MESSAGES_H */
