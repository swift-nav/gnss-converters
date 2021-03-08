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

#ifndef GNSS_CONVERTERS_TIME_TRUTH_H
#define GNSS_CONVERTERS_TIME_TRUTH_H

#include <stddef.h>
#include <swiftnav/gnss_time.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Time truth opaque type */
typedef union {
  max_align_t align;
  uint8_t size[sizeof(uint64_t)];
} time_truth_t;

/**
 * Set of "time truth" states
 */
enum time_truth_state {
  /** No confidence in its ability to determine absolute GPS time */
  TIME_TRUTH_UNKNOWN,

  /** High confidence in its ability to determine absolute GPS time */
  TIME_TRUTH_SOLVED
};

/**
 * Set of time sources for the "time truth" state machine
 */
enum time_truth_source {
  /** GPS ephemeride */
  TIME_TRUTH_EPH_GPS,

  /** Galileo ephemeride */
  TIME_TRUTH_EPH_GAL,

  /** BeiDou ephemeride */
  TIME_TRUTH_EPH_BDS,
};

/**
 * Initializes the "time truth" state machine
 *
 * @param instance pointer to the state machine struct
 */
void time_truth_init(time_truth_t* instance);

/**
 * Passes in new time information into the "time truth" state machine
 *
 * @param instance pointer to the state machine struct
 * @param source source in which the time information is based off of
 * @param time absolute time represented in GPS time format
 * @return true if the time truth time and/or state has been updated, otherwise
 * false
 *
 * @note invalid absolute GPS times are not permitted into the function, an
 * warning log will be recorded and the return will be false. an absolute GPS
 * time which is invalid for a constellation will be treated in similar manner,
 * for instance, a EPH_GAL source with GPS time { .wn = 1000, .tow = 0 } would
 * indicate a time prior to the GAL epoch of { .wn = 1024, .tow = 0 } and is
 * therefore invalid.
 */
bool time_truth_update(time_truth_t* instance,
                       enum time_truth_source source,
                       gps_time_t time);

/**
 * Retrieves the time truth's current time and state
 *
 * @param instance pointer to the state machine struct
 * @param state pointer to state object to fill in (NULL is valid)
 * @param time pointer to absolute gps time object to fill in (NULL is valid)
 *
 * @note if the time truth's state is TIME_TRUTH_UNKNOWN, then the returned
 * time is always invalid.
 */
void time_truth_get(time_truth_t* instance,
                    enum time_truth_state* state,
                    gps_time_t* time);

/**
 * Updates the time truth instance based on the SBP message content.
 *
 * @param instance pointer to the state machine struct
 * @param message_type SBP message type
 * @param length length of the SBP message payload in bytes
 * @param payload SBP message payload
 * @return true if the time truth time and/or state has been updated, otherwise
 * false
 */
bool time_truth_update_from_sbp(time_truth_t* instance,
                                uint16_t message_type,
                                uint8_t length,
                                const uint8_t* payload);

#ifdef __cplusplus
}
#endif

#endif  // GNSS_CONVERTERS_TIME_TRUTH_H
