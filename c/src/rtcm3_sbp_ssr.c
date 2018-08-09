

#include "rtcm3_sbp_internal.h"
#include <math.h>
#include <string.h>
#include <rtcm3_msm_utils.h>
#include <libsbp/ssr.h>
#include <stdio.h>

#define BIAS_MESSAGE_LENGTH 256

code_t get_gps_ssr_sbp_code(uint8_t signal_index){
  switch(signal_index){
    case 0: return CODE_GPS_L1CA;
    case 1: return CODE_GPS_L1P;
    case 2: return CODE_INVALID;
    case 3: return CODE_INVALID;
    case 4: return CODE_INVALID;
    case 5: return CODE_GPS_L2CM;
    case 6: return CODE_GPS_L2P;
    case 7: return CODE_GPS_L2CM;
    case 8: return CODE_GPS_L2CL;
    case 9: return CODE_GPS_L2CX;
    case 10: return CODE_GPS_L2P;
    case 11: return CODE_INVALID;
    case 12: return CODE_INVALID;
    case 13: return CODE_INVALID;
    case 14: return CODE_GPS_L5I;
    case 15: return CODE_GPS_L5Q;
    default: return CODE_INVALID;
  }
}

code_t get_sbp_code(constellation_t cons, uint8_t signal_index) {
  if (cons == CONSTELLATION_GPS) {
    return get_gps_ssr_sbp_code(signal_index);
  }
  return CODE_INVALID;
}

void rtcm3_ssr_orbit_clock_to_sbp(rtcm_msg_orbit_clock *msg_orbit_clock, struct rtcm3_sbp_state *state){
(void)msg_orbit_clock;
(void)state;
}

void rtcm3_ssr_code_bias_to_sbp(rtcm_msg_code_bias *msg_code_biases, struct rtcm3_sbp_state *state){
  uint8_t buffer[BIAS_MESSAGE_LENGTH];
  uint8_t length;
  msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t*)buffer;
  for(int sat_count=0; sat_count < msg_code_biases->header.num_sats; sat_count++){
    memset(buffer,0,BIAS_MESSAGE_LENGTH);
    length = 0;
    compute_gps_message_time(msg_code_biases->header.epoch_time * S_TO_MS, &sbp_code_bias->time, &state->time_from_rover_obs);
    length += sizeof(sbp_code_bias->time);
    sbp_code_bias->sid.sat = msg_code_biases->sats[sat_count].sat_id;
    sbp_code_bias->sid.code = 0;
    length += sizeof(sbp_code_bias->sid);
    sbp_code_bias->update_interval = msg_code_biases->header.update_interval;
    length += sizeof(sbp_code_bias->update_interval);
    sbp_code_bias->iod_ssr = msg_code_biases->header.iod_ssr;
    length += sizeof(sbp_code_bias->iod_ssr);
    for(int sig_count=0; sig_count < msg_code_biases->sats[sat_count].num_code_biases; sig_count++){
      sbp_code_bias->biases[sig_count].code = get_sbp_code(msg_code_biases->header.constellation,
                                                           msg_code_biases->sats[sat_count].signals[sig_count].signal_id);
      length += sizeof(sbp_code_bias->biases[sig_count].code);
      sbp_code_bias->biases[sig_count].value = msg_code_biases->sats[sat_count].signals[sig_count].code_bias;
      length += sizeof(sbp_code_bias->biases[sig_count].value);
    }
    state->cb_rtcm_to_sbp(SBP_MSG_SSR_CODE_BIASES,
                          length,
                          (u8 *)sbp_code_bias,
                          0,
                          state->context);
  }
}

void rtcm3_ssr_phase_bias_to_sbp(rtcm_msg_phase_bias *msg_phase_biases, struct rtcm3_sbp_state *state){
  (void)msg_phase_biases;
  (void)state;
}
