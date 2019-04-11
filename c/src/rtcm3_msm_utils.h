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

#ifndef GNSS_CONVERTERS_RTCM3_MSM_UTILS_H
#define GNSS_CONVERTERS_RTCM3_MSM_UTILS_H

#include <stdbool.h>
#include <stdint.h>

#include <rtcm3/messages.h>
#include <libswiftnav/signal.h>

bool msm_signal_frequency(const rtcm_msm_header *header,
                          const u8 signal_index,
                          const u8 glo_fcn,
                          const bool glo_fcn_valid,
                          double *p_freq);
code_t msm_signal_to_code(const rtcm_msm_header *header, u8 signal_index);
u8 code_to_msm_signal_index(const rtcm_msm_header *header, code_t code);
u8 code_to_msm_signal_id(code_t code, rtcm_constellation_t cons);
u8 msm_sat_to_prn(const rtcm_msm_header *header, u8 satellite_index);
u8 prn_to_msm_sat_index(const rtcm_msm_header *header, u8 prn);
u8 prn_to_msm_sat_id(u8 prn, rtcm_constellation_t cons);
bool msm_get_glo_fcn(const rtcm_msm_header *header,
                     const u8 sat,
                     const u8 fcn_from_sat_info,
                     const u8 glo_sv_id_fcn_map[],
                     u8 *glo_fcn);
u16 to_msm_msg_num(rtcm_constellation_t cons, msm_enum msm_type);
u8 msm_get_num_signals(const rtcm_msm_header *header);
u8 msm_get_num_satellites(const rtcm_msm_header *header);
u8 msm_get_num_cells(const rtcm_msm_header *header);

bool msm_add_to_header(rtcm_msm_header *header, code_t code, u8 prn);
bool msm_add_to_cell_mask(rtcm_msm_header *header, code_t code, u8 prn);

#endif /* GNSS_CONVERTERS_RTCM3_MSM_UTILS_H */
