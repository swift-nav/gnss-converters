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
#ifndef _OPTIONS_H
#define _OPTIONS_H

#include <swiftnav/signal.h>
#include <rtcm3/messages.h>

typedef enum verbosity_level_e {
    VERB_NORMAL = 0,
    VERB_HIGH,
    VERB_HIGHEST
} verbosity_level_t;


extern float sbp_signal_biases[CODE_COUNT];
extern float sbp_glo_code_bias[2];
extern float sbp_glo_phase_bias[2];
extern bool constellation_mask[CONSTELLATION_COUNT];
extern verbosity_level_t verbosity_level;

void msm_glo_fcn_bias(const rtcm_msm_header *header,
                      const u8 signal_index,
                      const u8 glo_fcn,
                      double *p_code,
                      double *p_phase);

#endif /* _OPTIONS_H */
