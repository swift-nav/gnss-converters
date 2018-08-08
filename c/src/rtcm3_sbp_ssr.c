

#include "rtcm3_sbp_internal.h"
#include <math.h>


#define BIAS_MESSAGE_LENGTH 256
void rtcm3_ssr_orbit_clock_to_sbp(rtcm_msg_orbit_clock *msg_orbit_clock, struct rtcm3_sbp_state *state){

}

void rtcm3_ssr_code_bias_to_sbp(rtcm_msg_code_bias *msg_code_biases, struct rtcm3_sbp_state *state){
  uint8_t buffer[BIAS_MESSAGE_LENGTH];
  msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t*)buffer;

  for(int sat_count=0; msg_code_biases->num_sats; sat_count++){
    memset(buffer,0,BIAS_MESSAGE_LENGTH);
    sbp_code_bias->time = msg_code_biases->epoch_time;
    sbp_code_bias->sid.sat_id = msg_code_biases->sats[sat_count].sat_id;
    sbp_code_bias->sid.code = 0
    sbp_code_bias->update_interval = msg_code_biases->header.update_interval;
    sbp_code_bias->iod_ssr = msg_code_biases->header.iod_ssr;
    for(int sig_count=0; msg_code_biases->sats[sat_count].num_code_biases; sig_count++){
      sbp_code_bias->biases[sig_count].code = msm_signal_to_code(msg_code_biases->sats[sat_count].signals[sig_count].signal_id)
      sbp_code_bias->biases[sig_count].value = msg_code_biases->sats[sat_count].signals[sig_count].code_bias;
    }
    state->
  }
}

void rtcm3_ssr_phase_bias_to_sbp(rtcm_msg_phase_bias *msg_phase_biases, struct rtcm3_sbp_state *state){

}
