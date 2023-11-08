/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <assert.h>
#include <gnss-converters/options.h>
#include <rtcm3/constants.h>
#include <rtcm3/msm_utils.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

#include "rtcm3_utils.h"

float sbp_signal_biases[CODE_COUNT] = {0.0f};
float sbp_glo_code_bias[2] = {0.0f};
float sbp_glo_phase_bias[2] = {0.0f};
bool constellation_mask[CONSTELLATION_COUNT] = {false};
verbosity_level_t verbosity_level = {VERB_NORMAL};

/** Find the frequency of an MSM signal
 *
 * \param header Pointer to message header
 * \param signal_index 0-based index into the signal mask
 * \param glo_fcn The FCN value for GLO satellites
 * \param glo_fcn_valid Validity flag for glo_fcn
 * \param p_code Pointer to write the code offset to
 * \param p_phase Pointer to write the carrier phase offset to
 * \return true if a valid frequency was returned
 */
void msm_glo_fcn_bias(const rtcm_msm_header *header,
                      const u8 signal_index,
                      const u8 glo_fcn,
                      double *p_code,
                      double *p_phase) {
  assert(signal_index <= MSM_SIGNAL_MASK_SIZE);
  assert(p_code);
  assert(p_phase);

  (*p_code) = 0.0;
  (*p_phase) = 0.0;
  code_t code = msm_signal_to_code(header, signal_index);

  switch ((int8_t)code) {
    case CODE_GLO_L1OF:
    case CODE_GLO_L1P:
      if (MSM_GLO_FCN_UNKNOWN != glo_fcn) {
        (*p_code) = (glo_fcn - MSM_GLO_FCN_OFFSET) * sbp_glo_code_bias[0];
        (*p_phase) = (glo_fcn - MSM_GLO_FCN_OFFSET) * sbp_glo_phase_bias[0];
      }
      break;
    case CODE_GLO_L2OF:
    case CODE_GLO_L2P:
      if (MSM_GLO_FCN_UNKNOWN != glo_fcn) {
        (*p_code) = (glo_fcn - MSM_GLO_FCN_OFFSET) * sbp_glo_code_bias[1];
        (*p_phase) = (glo_fcn - MSM_GLO_FCN_OFFSET) * sbp_glo_phase_bias[1];
      }
      break;
    default:
      break;
  }
}
