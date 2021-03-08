/**
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gnss-converters/time_truth.h>

#include <stdalign.h>
#include <stdatomic.h>
#include <string.h>

#include <libsbp/gnss.h>
#include <libsbp/observation.h>
#include <swiftnav/ephemeris.h>

/**
 * Time truth state machine data
 */
struct time_truth {
  float tow;
  s16 wn;
  uint8_t state;
};

/** Time truth actual type (ie. underlying implementation of time_truth_t) */
typedef _Atomic struct time_truth atomic_time_truth_t;

static_assert(sizeof(atomic_time_truth_t) <= sizeof(time_truth_t),
              "opaque type's size must be greater than or equal to the actual "
              "type's size");
static_assert(
    alignof(time_truth_t) == alignof(max_align_t),
    "opaque type must be aligned to the highest alignment requirement");

void time_truth_init(time_truth_t *instance) {
  struct time_truth value;
  memset(&value, 0, sizeof(value));

  value.wn = WN_UNKNOWN;
  value.tow = TOW_UNKNOWN;
  value.state = TIME_TRUTH_UNKNOWN;

  atomic_init((atomic_time_truth_t *)instance, value);
}

bool time_truth_update(time_truth_t *instance,
                       enum time_truth_source source,
                       gps_time_t time) {
  if (!gps_time_valid(&time)) {
    log_warn("Invalid time submitted to time truth");
    return false;
  }

  if (time.wn < GPS_WEEK_REFERENCE) {
    log_warn("Time submitted to time truth predates build time");
    return false;
  }

  struct time_truth current_time_truth;
  struct time_truth new_time_truth;

  memset(&current_time_truth, 0, sizeof(current_time_truth));
  memset(&new_time_truth, 0, sizeof(new_time_truth));

  current_time_truth = atomic_load((atomic_time_truth_t *)instance);

  do {
    const gps_time_t current_time = {.wn = current_time_truth.wn,
                                     .tow = current_time_truth.tow};

    switch (current_time_truth.state) {
      case TIME_TRUTH_UNKNOWN:
        switch (source) {
          case TIME_TRUTH_EPH_GPS:
          case TIME_TRUTH_EPH_GAL:
          case TIME_TRUTH_EPH_BDS:
            new_time_truth.wn = time.wn;
            new_time_truth.tow = (float)time.tow;
            new_time_truth.state = TIME_TRUTH_SOLVED;
            break;
          default:
            return false;
        }
        break;

      case TIME_TRUTH_SOLVED:
        switch (source) {
          case TIME_TRUTH_EPH_GPS:
          case TIME_TRUTH_EPH_GAL:
          case TIME_TRUTH_EPH_BDS:
            if (gpsdifftime(&time, &current_time) <= 0) {
              return false;
            }
            new_time_truth.wn = time.wn;
            new_time_truth.tow = (float)time.tow;
            new_time_truth.state = TIME_TRUTH_SOLVED;
            break;
          default:
            return false;
        }
        break;

      default:
        return false;
    }
  } while (!atomic_compare_exchange_strong(
      (atomic_time_truth_t *)instance, &current_time_truth, new_time_truth));

  if (current_time_truth.state != new_time_truth.state) {
    const char *new_state_name = NULL;
    switch (new_time_truth.state) {
      case TIME_TRUTH_UNKNOWN:
        new_state_name = "Unknown";
        break;
      case TIME_TRUTH_SOLVED:
        new_state_name = "Solved";
        break;
      default:
        new_state_name = "-";
        break;
    }
    log_info("Time truth state changed to \"%s\"", new_state_name);
  }

  return true;
}

void time_truth_get(time_truth_t *instance,
                    enum time_truth_state *state,
                    gps_time_t *time) {
  struct time_truth value = atomic_load((atomic_time_truth_t *)instance);

  if (state != NULL) {
    *state = value.state;
  }
  if (time != NULL) {
    time->wn = value.wn;
    time->tow = value.tow;
  }
}

bool time_truth_update_from_sbp(time_truth_t *instance,
                                uint16_t message_type,
                                uint8_t length,
                                const uint8_t *payload) {
  (void)length;

  enum time_truth_source time_source;
  gps_time_sec_t time;

  switch (message_type) {
    case SBP_MSG_EPHEMERIS_GPS:
      time_source = TIME_TRUTH_EPH_GPS;
      time = ((const msg_ephemeris_gps_t *)payload)->common.toe;
      break;
    case SBP_MSG_EPHEMERIS_GAL:
      time_source = TIME_TRUTH_EPH_GAL;
      time = ((const msg_ephemeris_gal_t *)payload)->common.toe;
      break;
    case SBP_MSG_EPHEMERIS_BDS:
      time_source = TIME_TRUTH_EPH_BDS;
      time = ((const msg_ephemeris_bds_t *)payload)->common.toe;
      break;
    default:
      return false;
  };

  gps_time_t time_truth_time;
  time_truth_time.wn = (s16)time.wn;
  time_truth_time.tow = time.tow;

  return time_truth_update(instance, time_source, time_truth_time);
}
