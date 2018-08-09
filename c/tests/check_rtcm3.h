#ifndef CHECK_RTCM3_H
#define CHECK_RTCM3_H

#include <libsbp/logging.h>
#include "../src/rtcm3_sbp_internal.h"

/* rtcm helper defines and functions */

#define MAX_FILE_SIZE 2337772
#define RTCM3_PREAMBLE 0xD3

#define FLOAT_EPS 1e-6

static double expected_L1CA_bias = 0.0;
static double expected_L1P_bias = 0.0;
static double expected_L2CA_bias = 0.0;
static double expected_L2P_bias = 0.0;

static sbp_gps_time_t previous_obs_time = {.tow = 0, .wn = INVALID_TIME};
static u8 previous_n_meas = 0;
static u8 previous_num_obs = 0;

static struct rtcm3_sbp_state state;

/* fixture globals and functions */
gps_time_sec_t current_time;

void rtcm3_setup_basic(void);
void update_obs_time(const msg_obs_t *msg);
void test_RTCM3(
    const char *filename,
    void (*cb_rtcm_to_sbp)(u16 msg_id, u8 length, u8 *buffer, u16 sender_id, void *context),
    gps_time_sec_t current_time);

#endif /* CHECK_RTCM3_H */
