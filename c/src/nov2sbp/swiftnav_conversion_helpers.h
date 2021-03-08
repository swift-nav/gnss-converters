/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef NOVATEL_SWIFTNAV_CONVERSION_HELPERS_H_
#define NOVATEL_SWIFTNAV_CONVERSION_HELPERS_H_

/**
 * This header provides inline conversion tools for going from the core
 * Novatel message structs into the corresponding Swiftnav/SBP structs. We use
 * the structs which match most closely with the Novatel types.
 *
 * In practice, for every Novatel type, there exists a
 * type in either Swiftnav or SBP providing more or less a 1-to-1 match.
 */

#include "parser/novatel.h"

#include <swiftnav/gnss_time.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/pvt_result.h>

extern "C" {
#include <libsbp/imu.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/sbp.h>
}

namespace Novatel {

/**
 * Novatel LLH accuracy is in meters, SBP is in millimeters.
 * For the horizontal accuracy, we just take the max of the lat/lon accuracies
 * provided by Novatel.
 */
void convert_bestpos_to_pos_llh(const BinaryHeader *header,
                                const Message::BESTPOS_t *bestpos,
                                msg_pos_llh_t *pos_llh);

/**
 * Convert Novatel BESTVEL message to SBP VEL NED message
 */
void convert_bestvel_to_vel_ned(const BinaryHeader *header,
                                const Message::BESTVEL_t *bestvel,
                                msg_vel_ned_t *vel_ned);

/**
 * The Novatel header timestamp is accurate to the millisecond level.
 */
void convert_header_to_gps_time(const BinaryHeader *header,
                                msg_gps_time_t *gps_time);

/**
 * Compare two msg_gps_time_t structures, return true if equal
 */
bool gps_time_compare(const msg_gps_time_t *a, const msg_gps_time_t *b);

/**
 * Of note:
 *
 * Novatel provides two IODEs, but as far as I can tell, they are always
 * identical. For this conversion, we just take the first one.
 *
 * The Novatel URA is the square of the nominal value.
 *
 * We assign all PRNs to GPS_L1CA.
 *
 * Everything else is pretty straight forward.
 */
void convert_gpsephem_to_ephemeris_gps(const Message::GPSEPHEM_t *gpsephem,
                                       msg_ephemeris_gps_t *ephem);

/**
 * Straightforward extraction of Novatel's GLONASS ephemeris
 * message. Notably, the Novatel health bits do not follow the
 * standard used in SwiftNav.
 */
void convert_gloephemeris_to_ephemeris_glo(
    const Message::GLOEPHEMERIS_t *gloephem, msg_ephemeris_glo_t *ephem);

/**
 * This is a subset of all the fields in a navigation_measurement_t.
 * Many of the fields in that struct are derived, so they have no correspondence
 * with the Novatel data. Here we present the fields necessary for a 1-to-1
 * mapping with the Novatel data -- the units are consistent with the proper
 * navigation_measurement_t so conversion should be trivial.
 */
struct mini_navigation_measurement_t {
  double raw_pseudorange;       // meters
  double raw_carrier_phase;     // cycles
  double raw_measured_doppler;  // Hz
  double cn0;                   // dB-Hz
  double lock_time;             // seconds
  gnss_signal_t sid;            // signal identifier
  nav_meas_flags_t flags;       // measurement status flags
};

/**
 * This conversion is mostly straightforward. The only tricks are first scaling
 * all of the Novatel values appropriately, and second properly setting the
 * measurement status flags based on the Novatel status bitfield.
 *
 * For information about the Novatel record structure and bitfields, see the
 * manual: https://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf
 */
void convert_rangecmp_record_to_mini_navmeas(
    const Message::RANGECMP_record_t *record,
    mini_navigation_measurement_t *nm);

/**
 * Convert Novatel INSATT message to SBP Orient Euler message
 */
void convert_insatt_to_orient_euler(const BinaryHeader *header,
                                    const Message::INSATT_t *insatt,
                                    msg_orient_euler_t *orient_euler);

/**
 * Convert Novatel RAWIMUSX message to SBP Angular Rate message
 */
bool convert_rawimu_to_angular_rate(const BinaryHeader *header,
                                    const Message::RAWIMUSX_t *rawimu,
                                    const msg_gps_time_t *last_imu_time,
                                    msg_angular_rate_t *angular_rate);

/**
 * Convert Novatel RAWIMUSX message to SBP Raw IMU Data message
 */
bool convert_rawimu_to_imu_raw(const BinaryHeader *header,
                               const Message::RAWIMUSX_t *rawimu,
                               const msg_gps_time_t *last_imu_time,
                               msg_imu_raw_t *imu_raw);

/**
 * Convert Novatel RAWIMUSX message to SBP Auxiliary IMU Data message
 */
bool convert_rawimu_to_imu_aux(const BinaryHeader *header,
                               const Message::RAWIMUSX_t *rawimu,
                               const msg_gps_time_t *last_aux_time,
                               msg_imu_aux_t *imu_aux);

}  // namespace Novatel

#endif
