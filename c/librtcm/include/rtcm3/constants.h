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

#ifndef SWIFTNAV_RTCM3_CONSTANTS_H
#define SWIFTNAV_RTCM3_CONSTANTS_H

#define PRUNIT_GPS 299792.458 /**< RTCM v3 Unit of GPS Pseudorange (m) */
#define PRUNIT_GLO 599584.916 /**< RTCM v3 Unit of GLO Pseudorange (m) */
#define RTCM_MAX_SATS 32
#define CP_INVALID 0xFFF80000      /* Unsigned bit pattern 0x80000 */
#define PR_L1_INVALID 0xFFF80000   /* Unsigned bit pattern 0x80000 */
#define PR_L2_INVALID 0xFFFFE000   /* Unsigned bit pattern 0x20000 */
#define MSM_MAX_CELLS 64           /* Maximum number of cells in MSM message */
#define MSM_SATELLITE_MASK_SIZE 64 /* Maximum size of MSM satellite mask */
#define MSM_SIGNAL_MASK_SIZE 32    /* Maximum size of MSM signal mask */
#define MSM_ROUGH_RANGE_INVALID 0xFF   /* Unsigned bit pattern 0xFF */
#define MSM_ROUGH_RATE_INVALID (-8192) /* Signed bit pattern 0x2000 */
#define MSM_PR_INVALID (-16384)        /* Signed bit pattern 0x4000 */
#define MSM_PR_EXT_INVALID (-524288)   /* Signed bit pattern 0x80000 */
#define MSM_CP_INVALID (-2097152)      /* Signed bit pattern 0x200000 */
#define MSM_CP_EXT_INVALID (-8388608)  /* Signed bit pattern 0x800000 */
#define MSM_DOP_INVALID (-16384)       /* Signed bit pattern 0x4000 */

#define RTCM_MAX_STRING_LEN 32 /* Max length of strings in 1008, 1033, etc */

#define MSM_GLO_FCN_OFFSET 7 /* Offset for FCN coding in sat_info */
#define MSM_GLO_MAX_FCN 13
#define MSM_GLO_FCN_UNKNOWN 255

#define MT1012_GLO_FCN_OFFSET 7
#define MT1012_GLO_MAX_FCN 20

/* maximum value for time-of-week in integer milliseconds */
#define RTCM_MAX_TOW_MS (7 * 24 * 3600 * 1000 - 1)
/* maximum value for time-of-day in integer milliseconds */
#define RTCM_GLO_MAX_TOW_MS (24 * 3600 * 1000 - 1)

/* maximum value for antenna height DF028 */
#define RTCM_1006_MAX_ANTENNA_HEIGHT_M 6.5535

#define MAX_NDF_FRAMES 63
#define MAX_NDF_FRAME_SIZE_BITS 4096 /* max uint12 */
#define MAX_NDF_FRAME_SIZE_WORDS (MAX_NDF_FRAME_SIZE_BITS / (8 * 4))

/** 2^-4 */
#define C_1_2P4 0.0625
/** 2^-8 */
#define C_1_2P8 0.00390625
/** 2^-10 */
#define C_1_2P10 0.0009765625
/** 2^-24 */
#define C_1_2P24 5.960464477539063e-08
/** 2^-29 */
#define C_1_2P29 1.862645149230957e-09
/** 2^-31 */
#define C_1_2P31 4.656612873077393e-10
/** 2^14 */
#define C_2P14 16384
/** 2^19 */
#define C_2P19 524288
/** 2^30 */
#define C_2P30 1073741824

/** Constant difference of Beidou time from GPS time */
#define BDS_SECOND_TO_GPS_SECOND 14

/** The official GPS value of the speed of light in m / s.
 * \note This is the exact value of the speed of light in vacuum (by the
 * definition of meters). */
#define GPS_C 299792458.0

/** The GPS L1 center frequency in Hz. */
#define GPS_L1_HZ 1.57542e9

/** The GPS L2 center frequency in Hz. */
#define GPS_L2_HZ 1.22760e9

/** The GLO L1 center frequency in Hz. */
#define GLO_L1_HZ 1.602e9

/** The GLO L2 center frequency in Hz. */
#define GLO_L2_HZ 1.246e9

/** Frequency range between two adjacent GLO channel in Hz for L1 band*/
#define GLO_L1_DELTA_HZ 5.625e5

/** Frequency range between two adjacent GLO channel in Hz for L2 band */
#define GLO_L2_DELTA_HZ 4.375e5

#define PRN_INVALID 0

#endif /* SWIFTNAV_RTCM3_CONSTANTS_H */
