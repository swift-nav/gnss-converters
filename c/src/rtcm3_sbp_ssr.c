

#include "rtcm3_sbp_internal.h"
#include <math.h>
#include <string.h>
#include <rtcm3_msm_utils.h>
#include <libsbp/ssr.h>
#include <stdio.h>

#define SSR_MESSAGE_LENGTH 256

void rtcm3_ssr_orbit_clock_to_sbp(rtcm_msg_orbit_clock *msg_orbit_clock, struct rtcm3_sbp_state *state){
  uint8_t buffer[SSR_MESSAGE_LENGTH];
  uint8_t length;
  msg_ssr_orbit_clock_t *sbp_orbit_clock = (msg_ssr_orbit_clock_t*)buffer;

  for(int sat_count=0; sat_count < msg_orbit_clock->header.num_sats; sat_count++){
    memset(buffer,0,SSR_MESSAGE_LENGTH);
    length = 0;
    if(msg_orbit_clock->header.constellation == CONSTELLATION_GPS) {
      compute_gps_message_time(msg_orbit_clock->header.epoch_time * S_TO_MS, &sbp_orbit_clock->time, &state->time_from_rover_obs);
    } else if (msg_orbit_clock->header.constellation == CONSTELLATION_GLO) {
      compute_glo_time(msg_orbit_clock->header.epoch_time * S_TO_MS, &sbp_orbit_clock->time, &state->time_from_rover_obs, state);
    }
    if(!gps_time_valid(&sbp_orbit_clock->time)){
      /* Invalid time */
      return;
    }

    length += sizeof(sbp_orbit_clock->time);

    sbp_orbit_clock->sid.sat = msg_orbit_clock->orbit[sat_count].sat_id;
    sbp_orbit_clock->sid.code = 0;
    length += sizeof(sbp_orbit_clock->sid);

    sbp_orbit_clock->update_interval = msg_orbit_clock->header.update_interval;
    length += sizeof(sbp_orbit_clock->update_interval);

    sbp_orbit_clock->iod_ssr = msg_orbit_clock->header.iod_ssr;
    length += sizeof(sbp_orbit_clock->iod_ssr);

    sbp_orbit_clock->iod = msg_orbit_clock->orbit[sat_count].iode;
    length += sizeof(sbp_orbit_clock->iod);

    sbp_orbit_clock->radial = msg_orbit_clock->orbit[sat_count].radial;
    length += sizeof(sbp_orbit_clock->radial);

    sbp_orbit_clock->along = msg_orbit_clock->orbit[sat_count].along_track;
    length += sizeof(sbp_orbit_clock->along);

    sbp_orbit_clock->cross = msg_orbit_clock->orbit[sat_count].cross_track;
    length += sizeof(sbp_orbit_clock->cross);

    sbp_orbit_clock->dot_radial = msg_orbit_clock->orbit[sat_count].dot_radial;
    length += sizeof(sbp_orbit_clock->dot_radial);

    sbp_orbit_clock->dot_along = msg_orbit_clock->orbit[sat_count].dot_along_track;
    length += sizeof(sbp_orbit_clock->dot_along);

    sbp_orbit_clock->dot_cross = msg_orbit_clock->orbit[sat_count].dot_cross_track;
    length += sizeof(sbp_orbit_clock->dot_cross);

    sbp_orbit_clock->c0 = msg_orbit_clock->clock[sat_count].c0;
    length += sizeof(sbp_orbit_clock->c0);

    sbp_orbit_clock->c1 = msg_orbit_clock->clock[sat_count].c1;
    length += sizeof(sbp_orbit_clock->c1);

    sbp_orbit_clock->c2 = msg_orbit_clock->clock[sat_count].c2;
    length += sizeof(sbp_orbit_clock->c2);

    state->cb_rtcm_to_sbp(SBP_MSG_SSR_ORBIT_CLOCK,
                          length,
                          (u8 *)sbp_orbit_clock,
                          0,
                          state->context);
  }
}

void rtcm3_ssr_code_bias_to_sbp(rtcm_msg_code_bias *msg_code_biases, struct rtcm3_sbp_state *state){
  uint8_t buffer[SSR_MESSAGE_LENGTH];
  uint8_t length;
  msg_ssr_code_biases_t *sbp_code_bias = (msg_ssr_code_biases_t*)buffer;
  for(int sat_count=0; sat_count < msg_code_biases->header.num_sats; sat_count++){
    memset(buffer,0,SSR_MESSAGE_LENGTH);
    length = 0;
    if(msg_code_biases->header.constellation == CONSTELLATION_GPS) {
      compute_gps_message_time(msg_code_biases->header.epoch_time * S_TO_MS, &sbp_code_bias->time, &state->time_from_rover_obs);
    } else if (msg_code_biases->header.constellation == CONSTELLATION_GLO) {
      compute_glo_time(msg_code_biases->header.epoch_time * S_TO_MS, &sbp_code_bias->time, &state->time_from_rover_obs, state);
    }
    if(!gps_time_valid(&sbp_code_bias->time)){
      /* Invalid time */
      return;
    }
    length += sizeof(sbp_code_bias->time);

    sbp_code_bias->sid.sat = msg_code_biases->sats[sat_count].sat_id;
    sbp_code_bias->sid.code = 0;
    length += sizeof(sbp_code_bias->sid);

    sbp_code_bias->update_interval = msg_code_biases->header.update_interval;
    length += sizeof(sbp_code_bias->update_interval);

    sbp_code_bias->iod_ssr = msg_code_biases->header.iod_ssr;
    length += sizeof(sbp_code_bias->iod_ssr);

    for(int sig_count=0; sig_count < msg_code_biases->sats[sat_count].num_code_biases; sig_count++){
      sbp_code_bias->biases[sig_count].code = msg_code_biases->sats[sat_count].signals[sig_count].signal_id;
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
  uint8_t buffer[SSR_MESSAGE_LENGTH];
  uint8_t length;
  msg_ssr_phase_biases_t *sbp_phase_bias = (msg_ssr_phase_biases_t*)buffer;
  for(int sat_count=0; sat_count < msg_phase_biases->header.num_sats; sat_count++){
    memset(buffer,0,SSR_MESSAGE_LENGTH);
    length = 0;
    if(msg_phase_biases->header.constellation == CONSTELLATION_GPS) {
      compute_gps_message_time(msg_phase_biases->header.epoch_time * S_TO_MS, &sbp_phase_bias->time, &state->time_from_rover_obs);
    } else if (msg_phase_biases->header.constellation == CONSTELLATION_GLO) {
      compute_glo_time(msg_phase_biases->header.epoch_time * S_TO_MS, &sbp_phase_bias->time, &state->time_from_rover_obs, state);  
    }
    if(!gps_time_valid(&sbp_phase_bias->time)){
      /* Invalid time */
      return;
    }
    length += sizeof(sbp_phase_bias->time);

    sbp_phase_bias->sid.sat = msg_phase_biases->sats[sat_count].sat_id;
    sbp_phase_bias->sid.code = 0;
    length += sizeof(sbp_phase_bias->sid);

    sbp_phase_bias->update_interval = msg_phase_biases->header.update_interval;
    length += sizeof(sbp_phase_bias->update_interval);

    sbp_phase_bias->iod_ssr = msg_phase_biases->header.iod_ssr;
    length += sizeof(sbp_phase_bias->iod_ssr);

    sbp_phase_bias->dispersive_bias = msg_phase_biases->header.dispersive_bias_consistency;
    length += sizeof(sbp_phase_bias->dispersive_bias);

    sbp_phase_bias->mw_consistency = msg_phase_biases->header.melbourne_wubbena_consistency;
    length += sizeof(sbp_phase_bias->mw_consistency);

    sbp_phase_bias->yaw = msg_phase_biases->sats[sat_count].yaw_angle;
    length += sizeof(sbp_phase_bias->yaw);

    sbp_phase_bias->yaw_rate = msg_phase_biases->sats[sat_count].yaw_rate;
    length += sizeof(sbp_phase_bias->yaw_rate);

    for(int sig_count=0; sig_count < msg_phase_biases->sats[sat_count].num_phase_biases; sig_count++){
      sbp_phase_bias->biases[sig_count].code = msg_phase_biases->sats[sat_count].signals[sig_count].signal_id;
      length += sizeof(sbp_phase_bias->biases[sig_count].code);

      sbp_phase_bias->biases[sig_count].integer_indicator = msg_phase_biases->sats[sat_count].signals[sig_count].integer_indicator;
      length += sizeof(sbp_phase_bias->biases[sig_count].integer_indicator);

      sbp_phase_bias->biases[sig_count].widelane_integer_indicator = msg_phase_biases->sats[sat_count].signals[sig_count].widelane_indicator;
      length += sizeof(sbp_phase_bias->biases[sig_count].widelane_integer_indicator);

      sbp_phase_bias->biases[sig_count].discontinuity_counter = msg_phase_biases->sats[sat_count].signals[sig_count].discontinuity_indicator;
      length += sizeof(sbp_phase_bias->biases[sig_count].discontinuity_counter);

      sbp_phase_bias->biases[sig_count].bias = msg_phase_biases->sats[sat_count].signals[sig_count].phase_bias;
      length += sizeof(sbp_phase_bias->biases[sig_count].bias);
    }
    state->cb_rtcm_to_sbp(SBP_MSG_SSR_PHASE_BIASES,
                          length,
                          (u8 *)sbp_phase_bias,
                          0,
                          state->context);
  }
}
