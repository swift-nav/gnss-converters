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

/* To illustrate the relationship between PRN, MSM satellite ID, satellite mask,
 * and the 0-based satellite index, used to index the satellite data array,
 * take for example QZS with measurements from PRNs 193, 194 and 196:
 *
 *                       PRN  |  193 |  194 |   193 |  196 |   197 |   198 | ...
 * MSM sat ID (Table 3.5-104) |    1 |    2 |     3 |    4 |     5 |     6 | ...
 *            satellite mask  | true | true | false | true | false | false | ...
 *            satellite index |    0 |    1 |       |    2 |       |       |
 *
 */

#include "rtcm3_utils.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include <rtcm3/constants.h>
#include <rtcm3/msm_utils.h>
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

/** Define the PRN ranges for each constellation. */
typedef struct {
  u16 first_prn;
  u16 num_sats;
} prn_table_element_t;

static const prn_table_element_t prn_table[RTCM_CONSTELLATION_COUNT] = {
    [RTCM_CONSTELLATION_GPS] = {GPS_FIRST_PRN, NUM_SATS_GPS},
    [RTCM_CONSTELLATION_SBAS] = {SBAS_FIRST_PRN, NUM_SATS_SBAS},
    [RTCM_CONSTELLATION_GLO] = {GLO_FIRST_PRN, NUM_SATS_GLO},
    [RTCM_CONSTELLATION_BDS] = {BDS_FIRST_PRN, NUM_SATS_BDS},
    [RTCM_CONSTELLATION_QZS] = {QZS_FIRST_PRN, NUM_SATS_QZS},
    [RTCM_CONSTELLATION_GAL] = {GAL_FIRST_PRN, NUM_SATS_GAL},
};

static code_t get_msm_gps_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-91 */
  switch (signal_id) {
    case 2: /* 1C */
      return CODE_GPS_L1CA;
    case 3: /* 1P */
      return CODE_GPS_L1P;
    case 4: /* 1W */
      return CODE_GPS_L1P;
    /* case 8: 2C */
    case 9: /* 2P */
      return CODE_GPS_L2P;
    case 10: /* 2W */
      return CODE_GPS_L2P;
    case 15: /* 2S */
      return CODE_GPS_L2CM;
    case 16: /* 2L */
      return CODE_GPS_L2CL;
    case 17: /* 2X */
      return CODE_GPS_L2CX;
    case 22: /* 5I */
      return CODE_GPS_L5I;
    case 23: /* 5Q */
      return CODE_GPS_L5Q;
    case 24: /* 5X */
      return CODE_GPS_L5X;
    case 30: /* 1S */
      return CODE_GPS_L1CI;
    case 31: /* 1L */
      return CODE_GPS_L1CQ;
    case 32: /* 1X */
      return CODE_GPS_L1CX;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_glo_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-96 */
  switch (signal_id) {
    case 3: /* 1P */
      return CODE_GLO_L1P;
    case 2: /* 1C */
      return CODE_GLO_L1OF;
    case 9: /* 2P */
      return CODE_GLO_L2P;
    case 8: /* 2C */
      return CODE_GLO_L2OF;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_gal_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-99 */
  switch (signal_id) {
    case 2: /* 1C */
      return CODE_GAL_E1C;
    /* case 3: 1A */
    case 4: /* 1B */
      return CODE_GAL_E1B;
    case 5: /* 1X */
      return CODE_GAL_E1X;
    /* case 6: 1Z */
    case 8: /* 6C */
      return CODE_GAL_E6C;
    /* case 9: 6A */
    case 10: /* 6B */
      return CODE_GAL_E6B;
    case 11: /* 6X */
      return CODE_GAL_E6X;
    /* case 12: 6Z */
    case 14: /* 7I */
      return CODE_GAL_E7I;
    case 15: /* 7Q */
      return CODE_GAL_E7Q;
    case 16: /* 7X */
      return CODE_GAL_E7X;
    case 18: /* 8I */
      return CODE_GAL_E8I;
    case 19: /* 8Q */
      return CODE_GAL_E8Q;
    case 20: /* 8X */
      return CODE_GAL_E8X;
    case 22: /* 5I */
      return CODE_GAL_E5I;
    case 23: /* 5Q */
      return CODE_GAL_E5Q;
    case 24: /* 5X */
      return CODE_GAL_E5X;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_sbas_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-102 */
  switch (signal_id) {
    case 2: /* 1C */
      return CODE_SBAS_L1CA;
    case 22: /* 5I */
      return CODE_SBAS_L5I;
    case 23: /* 5Q */
      return CODE_SBAS_L5Q;
    case 24: /* 5X */
      return CODE_SBAS_L5X;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_qzs_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-105 */
  switch (signal_id) {
    case 2: /* 1C */
      return CODE_QZS_L1CA;
    /* case 9:  6S */
    /* case 10: 6L */
    /* case 11: 6X */
    case 15: /* 2S */
      return CODE_QZS_L2CM;
    case 16: /* 2L */
      return CODE_QZS_L2CL;
    case 17: /* 2X */
      return CODE_QZS_L2CX;
    case 22: /* 5I */
      return CODE_QZS_L5I;
    case 23: /* 5Q */
      return CODE_QZS_L5Q;
    case 24: /* 5X */
      return CODE_QZS_L5X;
    case 30: /* 1S */
      return CODE_QZS_L1CI;
    case 31: /* 1L */
      return CODE_QZS_L1CQ;
    case 32: /* 1X */
      return CODE_QZS_L1CX;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_bds_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-108 */
  switch (signal_id) {
    case 2: /* 2I */
      return CODE_BDS2_B1;
    /* case 3:  2Q */
    /* case 4:  2X */
    /* case 5:  Reserved */
    /* case 6:  Reserved */
    /* case 7:  Reserved */
    case 8: /* 6I */
      return CODE_BDS3_B3I;
    case 9: /* 6Q */
      return CODE_BDS3_B3Q;
    case 10: /* 6X */
      return CODE_BDS3_B3X;
    /* case 11:  Reserved */
    /* case 12:  Reserved */
    /* case 13:  Reserved */
    case 14: /* 7I */
      return CODE_BDS2_B2;
    /* case 15:  7Q */
    /* case 16:  7X */
    /* case 17:  Reserved */
    /* case 18:  Reserved */
    /* case 19:  Reserved */
    /* case 20:  Reserved */
    /* case 21:  Reserved */
    case 22: /* B2aI */
      return CODE_BDS3_B5I;
    case 23: /* B2aQ */
      return CODE_BDS3_B5Q;
    case 24: /* B2aX */
      return CODE_BDS3_B5X;
    case 25: /* B2bI */
      return CODE_BDS3_B7I;
    /* case 26:  Reserved */
    /* case 27:  Reserved */
    /* case 28:  Reserved */
    /* case 29:  Reserved */
    case 30: /* B1C */
      return CODE_BDS3_B1CI;
    case 31: /* B1P */
      return CODE_BDS3_B1CQ;
    case 32: /* B1X */
      return CODE_BDS3_B1CX;
    default:
      return CODE_INVALID;
  }
}

/** Get the code enum of an MSM signal
 *
 * \param header Pointer to message header
 * \param signal_index 0-based index into the signal mask
 * \return code enum (CODE_INVALID for unsupported codes/constellations)
 */
code_t msm_signal_to_code(const rtcm_msm_header *header, u8 signal_index) {
  rtcm_constellation_t cons = to_constellation(header->msg_num);
  assert(signal_index <= MSM_SIGNAL_MASK_SIZE);
  u8 code_index =
      find_nth_mask_value(
          MSM_SIGNAL_MASK_SIZE, header->signal_mask, signal_index + 1) +
      1;

  switch (cons) {
    case RTCM_CONSTELLATION_GPS:
      return get_msm_gps_code(code_index);
    case RTCM_CONSTELLATION_SBAS:
      return get_msm_sbas_code(code_index);
    case RTCM_CONSTELLATION_GLO:
      return get_msm_glo_code(code_index);
    case RTCM_CONSTELLATION_BDS:
      return get_msm_bds_code(code_index);
    case RTCM_CONSTELLATION_QZS:
      return get_msm_qzs_code(code_index);
    case RTCM_CONSTELLATION_GAL:
      return get_msm_gal_code(code_index);
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      return CODE_INVALID;
  }
}

/** Get the index to MSM signal mask corresponding to a code enum
 *
 * \param header Pointer to message header
 * \param code code enum
 * \return signal_index 0-based index into the signal mask
 */
u8 code_to_msm_signal_index(const rtcm_msm_header *header, code_t code) {
  rtcm_constellation_t cons = to_constellation(header->msg_num);
  u8 signal_id = code_to_msm_signal_id(code, cons);
  assert(signal_id <= MSM_SIGNAL_MASK_SIZE);
  return count_mask_values(signal_id, header->signal_mask);
}

/** Get the MSM signal id from code enum
 *
 * \param code enum
 * \param cons constellation enum
 * \return The 0-based signal_id
 */
u8 code_to_msm_signal_id(const code_t code, const rtcm_constellation_t cons) {
  assert(RTCM_CONSTELLATION_INVALID != cons &&
         RTCM_CONSTELLATION_COUNT != cons);
  assert(CODE_INVALID != code);
  /* look up using the signal to code functions */
  switch (cons) {
    case RTCM_CONSTELLATION_GPS:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_gps_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_SBAS:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_sbas_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_GLO:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_glo_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_BDS:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_bds_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_QZS:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_qzs_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_GAL:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_gal_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      break;
  }
  log_info("Code %d not found in RTCM constellation %u", code, cons);
  return MSM_SIGNAL_MASK_SIZE;
}

static bool prn_valid(rtcm_constellation_t cons, u8 prn) {
  return (RTCM_CONSTELLATION_INVALID != cons) &&
         (RTCM_CONSTELLATION_COUNT > cons) &&
         (prn >= prn_table[cons].first_prn) &&
         (prn < prn_table[cons].first_prn + prn_table[cons].num_sats);
}

/** Get the PRN from an MSM satellite index
 *
 * \param header Pointer to message header
 * \param satellite_index 0-based index into the satellite mask
 * \return PRN (or 0 for invalid constellation)
 */
u8 msm_sat_to_prn(const rtcm_msm_header *header, u8 satellite_index) {
  rtcm_constellation_t cons = to_constellation(header->msg_num);
  if (RTCM_CONSTELLATION_INVALID == cons || RTCM_CONSTELLATION_COUNT == cons) {
    return PRN_INVALID;
  }
  assert(satellite_index <= MSM_SATELLITE_MASK_SIZE);
  u8 prn_index = find_nth_mask_value(
      MSM_SATELLITE_MASK_SIZE, header->satellite_mask, satellite_index + 1);

  u8 prn = prn_table[cons].first_prn + prn_index;
  return prn_valid(cons, prn) ? prn : PRN_INVALID;
}

/** Get the index to MSM satellite mask corresponding to a PRN
 *
 * \param header Pointer to message header
 * \param PRN
 * \return satellite_index 0-based index into the satellite mask
 */
u8 prn_to_msm_sat_index(const rtcm_msm_header *header, u8 prn) {
  rtcm_constellation_t cons = to_constellation(header->msg_num);
  u8 sat_id = prn_to_msm_sat_id(prn, cons);
  assert(sat_id <= MSM_SATELLITE_MASK_SIZE);
  return count_mask_values(sat_id, header->satellite_mask);
}

/** Get the MSM satellite ID corresponding to a PRN
 *
 * \param PRN
 * \param cons constellation enum
 * \return satellite_index 0-based index into the satellite mask
 */
u8 prn_to_msm_sat_id(u8 prn, rtcm_constellation_t cons) {
  assert(prn_valid(cons, prn));
  return prn - prn_table[cons].first_prn;
}

/** Find the frequency of an MSM signal
 *
 * \param header Pointer to message header
 * \param signal_index 0-based index into the signal mask
 * \param glo_fcn The FCN value for GLO satellites
 * \param p_freq Pointer to write the frequency output to
 * \return true if a valid frequency was returned
 */
bool msm_signal_frequency(const rtcm_msm_header *header,
                          const u8 signal_index,
                          const u8 glo_fcn,
                          double *p_freq) {
  assert(signal_index <= MSM_SIGNAL_MASK_SIZE);
  code_t code = msm_signal_to_code(header, signal_index);

  /* TODO: use sid_to_carr_freq from LNSP */

  switch ((int8_t)code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L1P:
    case CODE_GPS_L1CI:
    case CODE_GPS_L1CQ:
    case CODE_GPS_L1CX:
      *p_freq = GPS_L1_HZ;
      return true;
    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
    case CODE_GPS_L2CX:
    case CODE_GPS_L2P:
      *p_freq = GPS_L2_HZ;
      return true;
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
      *p_freq = GPS_L5_HZ;
      return true;
    case CODE_GLO_L1OF:
    case CODE_GLO_L1P:
      /* GLO FCN given in the sat info field, see Table 3.4-6 */
      if (MSM_GLO_FCN_UNKNOWN != glo_fcn) {
        *p_freq = GLO_L1_HZ + (glo_fcn - MSM_GLO_FCN_OFFSET) * GLO_L1_DELTA_HZ;
        return true;
      } else {
        return false;
      }
    case CODE_GLO_L2OF:
    case CODE_GLO_L2P:
      if (MSM_GLO_FCN_UNKNOWN != glo_fcn) {
        *p_freq = GLO_L2_HZ + (glo_fcn - MSM_GLO_FCN_OFFSET) * GLO_L2_DELTA_HZ;
        return true;
      } else {
        return false;
      }
    case CODE_BDS2_B1:
      *p_freq = BDS2_B11_HZ;
      return true;
    case CODE_BDS2_B2:
      *p_freq = BDS2_B2_HZ;
      return true;
    case CODE_BDS3_B1CI:
    case CODE_BDS3_B1CQ:
    case CODE_BDS3_B1CX:
      *p_freq = BDS3_B1C_HZ;
      return true;
    case CODE_BDS3_B3I:
    case CODE_BDS3_B3Q:
    case CODE_BDS3_B3X:
      *p_freq = BDS3_B3_HZ;
      return true;
    case CODE_BDS3_B5I:
    case CODE_BDS3_B5Q:
    case CODE_BDS3_B5X:
      *p_freq = BDS3_B5_HZ;
      return true;
    case CODE_BDS3_B7I:
    case CODE_BDS3_B7Q:
    case CODE_BDS3_B7X:
      *p_freq = BDS3_B7_HZ;
      return true;
    case CODE_SBAS_L1CA:
      *p_freq = SBAS_L1_HZ;
      return true;
    case CODE_SBAS_L5I:
    case CODE_SBAS_L5Q:
    case CODE_SBAS_L5X:
      *p_freq = SBAS_L5_HZ;
      return true;
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
      *p_freq = GAL_E1_HZ;
      return true;
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
      *p_freq = GAL_E7_HZ;
      return true;
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
      *p_freq = GAL_E5_HZ;
      return true;
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
      *p_freq = GAL_E6_HZ;
      return true;
    case CODE_GAL_E8I:
    case CODE_GAL_E8Q:
    case CODE_GAL_E8X:
      *p_freq = GAL_E8_HZ;
      return true;
    case CODE_QZS_L1CA:
    case CODE_QZS_L1CI:
    case CODE_QZS_L1CQ:
    case CODE_QZS_L1CX:
      *p_freq = QZS_L1_HZ;
      return true;
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
      *p_freq = QZS_L2_HZ;
      return true;
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
      *p_freq = QZS_L5_HZ;
      return true;
    case CODE_INVALID:
    case CODE_COUNT:
    default:
      return false;
  }
}

/** Find the frequency channel number (FCN) of a GLO signal
 *
 * \param header Pointer to message header
 * \param sat_index 0-based index into the satellite mask
 * \param fcn_from_satinfo FCN (or MSM_GLO_FCN_UNKNOWN)
 * \param glo_sv_id_fcn_map Optional GLO FCN table (size MAX_GLO_PRN + 1)
 * \param glo_fcn Output pointer for the FCN value
 * \return true if a valid FCN was returned
 */
bool msm_get_glo_fcn(const rtcm_msm_header *header,
                     const u8 sat,
                     const u8 fcn_from_sat_info,
                     const u8 glo_sv_id_fcn_map[],
                     u8 *glo_fcn) {
  if (RTCM_CONSTELLATION_GLO != to_constellation(header->msg_num)) {
    return false;
  }

  /* get FCN from sat_info if valid */
  *glo_fcn = fcn_from_sat_info;
  if (MSM_GLO_FCN_UNKNOWN == *glo_fcn && NULL != glo_sv_id_fcn_map) {
    /* use the lookup table if given */
    u8 sat_id = msm_sat_to_prn(header, sat);
    *glo_fcn = glo_sv_id_fcn_map[sat_id];
  }
  /* valid values are from 0 to MSM_GLO_MAX_FCN */
  return (*glo_fcn <= MSM_GLO_MAX_FCN);
}

/** Return the RTCM message number for a constellation and MSM message type
 * \param cons RTCM constellation enum
 * \param msm_type MSM message type enum
 * \return RTCM message number, or 0 on failure
 */
u16 to_msm_msg_num(rtcm_constellation_t cons, msm_enum msm_type) {
  if (MSM_UNKNOWN == msm_type) {
    return 0;
  }
  switch (cons) {
    case RTCM_CONSTELLATION_GPS:
      return 1070 + (u8)msm_type;
    case RTCM_CONSTELLATION_GLO:
      return 1080 + (u8)msm_type;
    case RTCM_CONSTELLATION_GAL:
      return 1090 + (u8)msm_type;
    case RTCM_CONSTELLATION_SBAS:
      return 1100 + (u8)msm_type;
    case RTCM_CONSTELLATION_QZS:
      return 1110 + (u8)msm_type;
    case RTCM_CONSTELLATION_BDS:
      return 1120 + (u8)msm_type;
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      return 0;
  }
}

u8 msm_get_num_signals(const rtcm_msm_header *header) {
  return count_mask_values(MSM_SIGNAL_MASK_SIZE, header->signal_mask);
}

u8 msm_get_num_satellites(const rtcm_msm_header *header) {
  return count_mask_values(MSM_SATELLITE_MASK_SIZE, header->satellite_mask);
}

u8 msm_get_num_cells(const rtcm_msm_header *header) {
  u8 cell_size = msm_get_num_satellites(header) * msm_get_num_signals(header);
  return count_mask_values(cell_size, header->cell_mask);
}

static void msm_add_to_header_err(code_t code, u8 prn, const char reason[]) {
  char code_string[SID_STR_LEN_MAX];
  gnss_signal_t sid = {prn, code};
  sid_to_string(code_string, SID_STR_LEN_MAX, sid);
  log_info("Cannot add %s to MSM header: %s", code_string, reason);
}

/* add the given signal and satellite to the MSM header, returns true if
 * successful, false on failure (invalid signal, or cell mask full) */
bool msm_add_to_header(rtcm_msm_header *header, code_t code, u8 prn) {
  /* should not try to add more satellites or signals if cell mask is already
   * filled */
  assert(msm_get_num_cells(header) == 0);

  rtcm_constellation_t cons = to_constellation(header->msg_num);

  if (CODE_INVALID == code) {
    msm_add_to_header_err(code, prn, "invalid code");
    return false;
  }
  if (!prn_valid(cons, prn)) {
    msm_add_to_header_err(code, prn, "invalid PRN");
    return false;
  }

  u8 sat_id = prn_to_msm_sat_id(prn, cons);
  u8 signal_id = code_to_msm_signal_id(code, cons);

  u8 num_sats = msm_get_num_satellites(header);
  u8 num_signals = msm_get_num_signals(header);

  /* add a new satellite to the mask only if it fits in the cell mask */
  if (!header->satellite_mask[sat_id]) {
    if ((num_sats + 1) * num_signals <= MSM_MAX_CELLS) {
      header->satellite_mask[sat_id] = true;
      num_sats++;
    } else {
      msm_add_to_header_err(
          code, prn, "cell size limit for satellites reached");
      return false;
    }
  }

  /* add a new signal to the mask only if it fits in the cell mask */
  if (!header->signal_mask[signal_id]) {
    if (num_sats * (num_signals + 1) <= MSM_MAX_CELLS) {
      header->signal_mask[signal_id] = true;
      num_signals++;
    } else {
      msm_add_to_header_err(code, prn, "cell size limit for signals reached");
      return false;
    }
  }
  return true;
}

/* given a header with satellite and signal masks built, allocate a cell for the
 * given signal and satellite */
bool msm_add_to_cell_mask(rtcm_msm_header *header, code_t code, u8 prn) {
  uint8_t num_sigs = msm_get_num_signals(header);
  assert(num_sigs > 0);

  rtcm_constellation_t cons = to_constellation(header->msg_num);

  if (CODE_INVALID == code) {
    msm_add_to_header_err(code, prn, "invalid code");
    return false;
  }
  if (!prn_valid(cons, prn)) {
    msm_add_to_header_err(code, prn, "invalid PRN");
    return false;
  }

  u8 sat_id = prn_to_msm_sat_id(prn, cons);
  if (!header->satellite_mask[sat_id]) {
    msm_add_to_header_err(code, prn, "not in satellite mask");
    return false;
  }

  u8 signal_id = code_to_msm_signal_id(code, cons);
  if (!header->signal_mask[signal_id]) {
    msm_add_to_header_err(code, prn, "not in signal mask");
    return false;
  }

  u8 sat_index = prn_to_msm_sat_index(header, prn);
  u8 signal_index = code_to_msm_signal_index(header, code);

  /* Mark the cell defined by this satellite/signal pair in the cell mask */
  u8 cell_id = sat_index * num_sigs + signal_index;
  assert(cell_id < MSM_MAX_CELLS);
  assert(!header->cell_mask[cell_id]);
  header->cell_mask[cell_id] = true;

  return true;
}

float convert_ura_to_uri(uint8_t ura) {
  /* Convert between RTCM/GPS URA ("User Range Accuracy") index to a number in
   * meters.
   * See section 2.5.3, "User Range Accuracy", in the GPS signal specification.
   * Indices 1, 3, and 5 are hard-coded according to spec, and 15 is hard-coded
   * according to SBP/Piksi convention. */
  if (ura == 1) {
    return 2.8f;
  }
  if (ura == 3) {
    return 5.7f;
  }
  if (ura == 5) {
    return 11.3f;
  }
  if (ura <= 6) {
    return powf(2, (1 + ura / 2.0f));
  }
  if (ura > 6 && ura < 15) {
    return powf(2, (ura - 2));
  }
  if (ura == 15) {
    return 6144;
  }
  return -1;
}

uint8_t convert_gps_uri_to_ura(float uri) {
  /* Convert between RTCM/GPS URA ("User Range Accuracy") number in
   * meters to the encoded index.
   * See section 2.5.3, "User Range Accuracy", in the GPS signal specification.
   * Indices 1, 3, and 5 are hard-coded according to spec, and 15 is hard-coded
   * according to SBP/Piksi convention. */
  uint8_t ura;
  if (fabsf(uri - 2.0f) < FLOAT_EQUALITY_EPS) {
    ura = 0;
  } else if (fabsf(uri - 2.8f) < FLOAT_EQUALITY_EPS) {
    ura = 1;
  } else if (fabsf(uri - 4.0f) < FLOAT_EQUALITY_EPS) {
    ura = 2;
  } else if (fabsf(uri - 5.7f) < FLOAT_EQUALITY_EPS) {
    ura = 3;
  } else if (fabsf(uri - 8.0f) < FLOAT_EQUALITY_EPS) {
    ura = 4;
  } else if (fabsf(uri - 11.3f) < FLOAT_EQUALITY_EPS) {
    ura = 5;
  } else if (fabsf(uri - 16.0f) < FLOAT_EQUALITY_EPS) {
    ura = 6;
  } else if (fabsf(uri - 6144.0f) < FLOAT_EQUALITY_EPS) {
    ura = 15;
  } else {
    ura = (uint8_t)(log2f(uri) + 2.0);
  }
  return ura;
}

uint8_t convert_glo_uri_to_ura(float uri) {
  /* Convert between RTCM/GLO FT ("GLONASS-M predicted satellite user range
   * accuracy") index to a number in meters.
   * See table 4.4 in GLO signal specification.
   * Indices 1, 3, and 5 are hard-coded according to spec, and 15 is hard-coded
   * according to SBP/Piksi convention. */
  uint8_t ura;
  if (fabsf(uri - 1.0f) < FLOAT_EQUALITY_EPS) {
    ura = 0;
  } else if (fabsf(uri - 2.0f) < FLOAT_EQUALITY_EPS) {
    ura = 1;
  } else if (fabsf(uri - 2.5f) < FLOAT_EQUALITY_EPS) {
    ura = 2;
  } else if (fabsf(uri - 4.0f) < FLOAT_EQUALITY_EPS) {
    ura = 3;
  } else if (fabsf(uri - 5.0f) < FLOAT_EQUALITY_EPS) {
    ura = 4;
  } else if (fabsf(uri - 7.0f) < FLOAT_EQUALITY_EPS) {
    ura = 5;
  } else if (fabsf(uri - 10.0f) < FLOAT_EQUALITY_EPS) {
    ura = 6;
  } else if (fabsf(uri - 12.0f) < FLOAT_EQUALITY_EPS) {
    ura = 7;
  } else if (fabsf(uri - 14.0f) < FLOAT_EQUALITY_EPS) {
    ura = 8;
  } else if (fabsf(uri - 16.0f) < FLOAT_EQUALITY_EPS) {
    ura = 9;
  } else if (fabsf(uri - 32.0f) < FLOAT_EQUALITY_EPS) {
    ura = 10;
  } else if (fabsf(uri - 64.0f) < FLOAT_EQUALITY_EPS) {
    ura = 11;
  } else if (fabsf(uri - 128.0f) < FLOAT_EQUALITY_EPS) {
    ura = 12;
  } else if (fabsf(uri - 256.0f) < FLOAT_EQUALITY_EPS) {
    ura = 13;
  } else if (fabsf(uri - 512.0f) < FLOAT_EQUALITY_EPS) {
    ura = 14;
  } else if (fabsf(uri - 6144.0f) < FLOAT_EQUALITY_EPS) {
    ura = 15;
  } else {
    ura = (uint8_t)(log2f(uri) + 2.0);
  }
  return ura;
}

uint8_t convert_bds_uri_to_ura(float uri) {
  /* (meters See: BDS ICD Section 5.2.4.: to define nominal
  values, N = 0-6: use 2^(1+N/2) (round to one
  decimal place i.e. 2.8, 5.7 and 11.3) , N=
  7-15:use 2^(N-2), 8192 specifies use at own
  risk) */
  uint8_t ura;
  if (fabsf(uri - 2.0f) < FLOAT_EQUALITY_EPS) {
    ura = 0;
  } else if (fabsf(uri - 2.8f) < FLOAT_EQUALITY_EPS) {
    ura = 1;
  } else if (fabsf(uri - 4.0f) < FLOAT_EQUALITY_EPS) {
    ura = 2;
  } else if (fabsf(uri - 5.7f) < FLOAT_EQUALITY_EPS) {
    ura = 3;
  } else if (fabsf(uri - 8.0f) < FLOAT_EQUALITY_EPS) {
    ura = 4;
  } else if (fabsf(uri - 11.3f) < FLOAT_EQUALITY_EPS) {
    ura = 5;
  } else if (fabsf(uri - 16.0f) < FLOAT_EQUALITY_EPS) {
    ura = 6;
  } else if (fabsf(uri - 32.0f) < FLOAT_EQUALITY_EPS) {
    ura = 7;
  } else if (fabsf(uri - 64.0f) < FLOAT_EQUALITY_EPS) {
    ura = 8;
  } else if (fabsf(uri - 128.0f) < FLOAT_EQUALITY_EPS) {
    ura = 9;
  } else if (fabsf(uri - 256.0f) < FLOAT_EQUALITY_EPS) {
    ura = 10;
  } else if (fabsf(uri - 512.0f) < FLOAT_EQUALITY_EPS) {
    ura = 11;
  } else if (fabsf(uri - 1024.0f) < FLOAT_EQUALITY_EPS) {
    ura = 12;
  } else if (fabsf(uri - 2048.0f) < FLOAT_EQUALITY_EPS) {
    ura = 13;
  } else if (fabsf(uri - 4096.0f) < FLOAT_EQUALITY_EPS) {
    ura = 14;
  } else if (fabsf(uri - 8196.0f) < FLOAT_EQUALITY_EPS) {
    ura = 15;
  } else {
    ura = (uint8_t)(log2f(uri) + 2.0);
  }
  return ura;
}

/** Calculate the GPS ephemeris curve fit interval.
 *
 * \param fit_interval_flag The curve fit interval flag. 0 is 4 hours, 1 is >4
 * hours.
 * \param iodc The IODC value.
 * \return the curve fit interval in seconds.
 */
u32 rtcm3_decode_fit_interval_gps(u8 fit_interval_flag, u16 iodc) {
  u8 fit_interval = 4; /* This is in hours */

  if (fit_interval_flag) {
    fit_interval = 6;

    if ((iodc >= 240) && (iodc <= 247)) {
      fit_interval = 8;
    } else if (((iodc >= 248) && (iodc <= 255)) || (iodc == 496)) {
      fit_interval = 14;
    } else if (((iodc >= 497) && (iodc <= 503)) ||
               ((iodc >= 1021) && (iodc <= 1023))) {
      fit_interval = 26;
    } else if ((iodc >= 504) && (iodc <= 510)) {
      fit_interval = 50;
    } else if ((iodc == 511) || ((iodc >= 752) && (iodc <= 756))) {
      fit_interval = 74;
    } else if (iodc == 757) {
      fit_interval = 98;
    }
  }

  return fit_interval * 60 * 60;
}

/** Calculate the GPS ephemeris curve fit interval.
 *
 * \param fit_interval The curve fit interval in seconds
 * \return the curve fit interval flag
 */
u8 rtcm3_encode_fit_interval_gps(u32 fit_interval) {
  /* 4 hours in seconds */
  if (fit_interval == 4 * 60 * 60) {
    return false;
  }
  return true;
}

/** Calculate the GLO ephemeris curve fit interval.
 *
 * \param fit_interval The curve fit interval in seconds
 * \return the curve fit interval flag
 */
u8 rtcm3_encode_fit_interval_glo(u32 fit_interval) {
  // Return the flag based on the curve fit in minutes
  switch (fit_interval / 60) {
    case (30 + 10):
      return 1;
    case (45 + 10):
      return 2;
    case (60 + 10):
    default:
      return 0;
  }
}

float convert_sisa_to_meters(const uint8_t sisa) {
  /* Convert between RTCM/GAL SISA  index to a number in meters.*/
  if (sisa <= FIRST_SISA_STEP) {
    return FIRST_SISA_MIN_METERS + sisa * FIRST_SISA_RESOLUTION;
  }
  if (sisa <= SECOND_SISA_STEP) {
    return SECOND_SISA_MIN_METERS +
           (sisa - FIRST_SISA_STEP) * SECOND_SISA_RESOLUTION;
  }
  if (sisa <= THIRD_SISA_STEP) {
    return THIRD_SISA_MIN_METERS +
           (sisa - SECOND_SISA_STEP) * THIRD_SISA_RESOLUTION;
  }
  if (sisa <= FOURTH_SISA_STEP) {
    return FOURTH_SISA_MIN_METERS +
           (sisa - THIRD_SISA_STEP) * FOURTH_SISA_RESOLUTION;
  }
  return -1;
}

uint8_t convert_meters_to_sisa(const float ura) {
  /* Convert between meters and GAL SISA index.*/
  if (ura < 0) {
    return SISA_NAPA;
  }

  if (ura <= FIRST_SISA_MAX_METERS) {
    return (uint8_t)(((ura - FIRST_SISA_MIN_METERS) / FIRST_SISA_RESOLUTION) +
                     0.5);
  }
  if (ura <= SECOND_SISA_MAX_METERS) {
    return (uint8_t)(((ura - SECOND_SISA_MIN_METERS) / SECOND_SISA_RESOLUTION) +
                     FIRST_SISA_STEP + 0.5);
  }
  if (ura <= THIRD_SISA_MAX_METERS) {
    return (uint8_t)(((ura - THIRD_SISA_MIN_METERS) / THIRD_SISA_RESOLUTION) +
                     SECOND_SISA_STEP + 0.5);
  }
  if (ura <= FOURTH_SISA_MAX_METERS) {
    return (uint8_t)(((ura - FOURTH_SISA_MIN_METERS) / FOURTH_SISA_RESOLUTION) +
                     THIRD_SISA_STEP + 0.5);
  }
  return -1;
}
