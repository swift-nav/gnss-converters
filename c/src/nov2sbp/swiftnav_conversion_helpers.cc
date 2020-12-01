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

#include <cmath>
#include <string>

#include <swiftnav/geoid_model.h>
#include "swiftnav_conversion_helpers.h"

/**
 * Helpers for rounding and truncating values.
 */
static inline uint16_t float_to_u16(float value) {
  return MIN(static_cast<uint16_t>(round(value)), UINT16_MAX);
}

static inline int32_t double_to_s32(double value) {
  return MIN(static_cast<int32_t>(round(value)), INT32_MAX);
}

static inline double radians_to_microdegrees(double val) {
  return val * 180. / M_PI * 1000000.;
}

static inline double degrees_to_microdegrees(double val) {
  return val * 1000000.;
}

// tracks flags from BESTPOS/BESTVEL messages
static class PositionFlagsTracker {
 private:
  uint32_t tow_;
  uint8_t flags_;

 public:
  PositionFlagsTracker() : tow_(~0u), flags_(0) {}

  void set(uint32_t tow, uint8_t flags) {
    tow_ = tow;
    flags_ = flags;
  }

  uint8_t get(uint32_t tow) const {
    if (tow == tow_) {
      return flags_;
    }

    return 0;
  }
} last_position_flags;

namespace Novatel {

// Table 81: Position or Velocity Type
typedef enum position_type_t {
  // No solution
  POS_TYPE_NONE = 0,
  // Position has been fixed by the FIX position command or by position
  // averaging.
  POS_TYPE_FIXEDPOS = 1,
  // Position has been fixed by the FIX height or FIX auto command or by
  // position averaging
  POS_TYPE_FIXEDHEIGHT = 2,
  // Solution from floating point carrier phase ambiguities
  POS_TYPE_FLOATCONV = 4,
  // Solution from wide-lane ambiguities
  POS_TYPE_WIDELANE = 5,
  // Solution from narrow-lane ambiguities
  POS_TYPE_NARROWLANE = 6,
  // Velocity computed using instantaneous Doppler
  POS_TYPE_DOPPLER_VELOCITY = 8,
  // Single point position
  POS_TYPE_SINGLE = 16,
  // Pseudorange differential solution
  POS_TYPE_PSRDIFF = 17,
  // Solution calculated using corrections from an SBAS
  POS_TYPE_WAAS = 18,
  // Propagated by a Kalman filter without new observation
  POS_TYPE_PROPOGATED = 19,
  // Floating L1 ambiguity solution
  POS_TYPE_L1_FLOAT = 32,
  // Floating ionospheric-free ambiguity solution
  POS_TYPE_IONOFREE_FLOAT = 33,
  // Floating narrow-lane ambiguity solution
  POS_TYPE_NARROW_FLOAT = 34,
  // Integer L1 ambiguity solution
  POS_TYPE_L1_INT = 48,
  // Integer wide-lane ambiguity solution
  POS_TYPE_WIDE_INT = 49,
  // Integer narrow-lane ambiguity solution
  POS_TYPE_NARROW_INT = 50,
  // RTK status where the RTK filter is directly initialized from the INS filter
  POS_TYPE_RTK_DIRECT_INS = 51,
  // INS calculated position corrected for the antenna
  POS_TYPE_INS_SBAS = 52,
  // INS pseudorange single point solution – no DGPS corrections
  POS_TYPE_INS_PSRSP = 53,
  // INS pseudorange differential solution
  POS_TYPE_INS_PSRDIFF = 54,
  // INS RTK floating point ambiguities solution
  POS_TYPE_INS_RTKFLOAT = 55,
  //  INS RTK fixed ambiguities solution
  POS_TYPE_INS_RTKFIXED = 56,
  // Converging TerraStar-C solution
  POS_TYPE_PPP_CONVERGING = 68,
  // Converged TerraStar-C solution
  POS_TYPE_PPP = 69,
  // Solution accuracy is within UAL operational limit
  POS_TYPE_OPERATIONAL = 70,
  // Solution accuracy is outside UAL operational limit but within warning limit
  POS_TYPE_WARNING = 71,
  // Solution accuracy is outside UAL limits
  POS_TYPE_OUT_OF_BOUNDS = 72,
  // INS NovAtel CORRECT Precise Point Positioning (PPP) solution converging
  POS_TYPE_INS_PPP_CONVERGING = 73,
  // INS NovAtel CORRECT PPP solution
  POS_TYPE_INS_PPP = 74,
  // Converging TerraStar-L solution
  POS_TYPE_PPP_BASIC_CONVERGING = 77,
  // Converged TerraStar-L solution
  POS_TYPE_PPP_BASIC = 78,
  //  INS NovAtel CORRECT PPP basic solution
  POS_TYPE_INS_PPP_BASIC = 79,
  // INS NovAtel CORRECT PPP basic solution converging
  POS_TYPE_INS_PPP_BASIC_CONVERGING = 80
} position_type_t;

// Table 218: IMU Type
typedef enum imu_type_t {
  // Unknown IMU type (default)
  UNKNOWN = 0,
  // Honeywell HG1700 AG11
  HG1700_AG11 = 1,
  // Honeywell HG1700 AG17
  HG1700_AG17 = 4,
  // Honeywell HG1900 CA29
  HG1900_CA29 = 5,
  // Northrop Grumman LN200/LN200C
  LN200 = 8,
  // Honeywell HG1700 AG58
  HG1700_AG58 = 11,
  // Honeywell HG1700 AG62
  HG1700_AG62 = 12,
  // iMAR iIMU-FSAS
  IMAR_FSAS = 13,
  // KVH CPT IMU
  KVH_COTS = 16,
  // Honeywell HG1930 AA99
  HG1930_AA99 = 20,
  // Northrop Grumman Litef ISA-100C
  ISA100C = 26,
  // Honeywell HG1900 CA50
  HG1900_CA50 = 27,
  // Honeywell HG1930 CA50
  HG1930_CA50 = 28,
  // Analog Devices ADIS16488
  ADIS16488 = 31,
  // Sensonor STIM300
  STIM300 = 32,
  // KVH1750 IMU
  KVH_1750 = 33,
  // Epson G320N
  EPSON_G320 = 41,
  // Northrop Grumman Litef μIMU-IC
  LITEF_MICROIMU = 52,
  // Sensonor STIM300, Direct Connection
  STIM300D = 56,
  // Honeywell HG4930 AN01
  HG4930_AN01 = 58,
  // Epson G370N
  EPSON_G370 = 61,
  // Epson G320N - 200 Hz
  EPSON_G320_200HZ = 62
} imu_type_t;

// Table 225: Inertial Solution Status
typedef enum inertial_solution_status_t {
  // logs are present, but the alignment routine has not started; INS is
  // inactive.
  INS_INACTIVE_IMU = 0,
  // INS is in alignment mode.
  INS_ALIGNING = 1,
  // The INS solution is in navigation mode but the azimuth solution uncertainty
  // has exceeded the threshold. The default threshold is 2 degrees for most
  // IMUs. The solution is still valid but you should monitor the solution
  // uncertainty in the INSSTDEV log (see page 963). You may encounter this
  // state during times when the GNSS, used to aid the INS, is absent.
  // The INS solution uncertainty contains outliers and the solution may be
  // outside specifications. The solution is still valid but you should monitor
  // the solution uncertainty in the INSSTDEV log (see page 963). It may be
  // encountered during times when GNSS is absent or poor.
  INS_HIGH_VARIANCE = 2,
  // The INS filter is in navigation mode and the INS solution is good.
  INS_SOLUTION_GOOD = 3,
  // The INS filter is in navigation mode and the GNSS solution is suspected to
  // be in error.
  // This may be due to multipath or limited satellite visibility. The inertial
  // filter has rejected the GNSS position and is waiting for the solution
  // quality to improve.
  INS_SOLUTION_FREE = 6,
  // The INS filter is in navigation mode, but not enough vehicle dynamics have
  // been experienced for the system to be within specifications.
  INS_ALIGNMENT_COMPLETE = 7,
  // INS is determining the IMU axis aligned with gravity.
  DETERMINING_ORIENTATION = 8,
  // The INS filter has determined the IMU orientation and is awaiting an
  // initial position estimate to begin the alignment process.
  WAITING_INITIALPOS = 9,
  // The INS filer has orientation, initial biases, initial position and valid
  // roll/pitch estimated. Will not proceed until initial azimuth is entered.
  WAITING_AZIMUTH = 10,
  // The INS filter is estimating initial biases during the first 10 seconds of
  // stationary data.
  INITIALIZING_BIASES = 11,
  // The INS filter has not completely aligned, but has detected motion.
  MOTION_DETECT = 12
} inertial_solution_status_t;

/*
 * Convert Novatel position type into SBP flags field indicating
 * fix mode and INS status
 */
static uint8_t get_position_flags(uint32_t pos_type) {
  uint8_t fix_mode = 0;
  bool ins_used = false;

  switch (pos_type) {
    case POS_TYPE_NONE:
      break;
    case POS_TYPE_SINGLE:
      fix_mode = POSITION_MODE_SPP;
      break;
    case POS_TYPE_PSRDIFF:
      fix_mode = POSITION_MODE_DGNSS;
      break;
    case POS_TYPE_WAAS:
      fix_mode = POSITION_MODE_SBAS;
      break;
    case POS_TYPE_NARROW_FLOAT:
      fix_mode = POSITION_MODE_FLOAT;
      break;
    case POS_TYPE_L1_INT:
    case POS_TYPE_NARROW_INT:
    case POS_TYPE_WIDE_INT:
      fix_mode = POSITION_MODE_FIXED;
      break;
    case POS_TYPE_INS_PSRSP:
      fix_mode = POSITION_MODE_SPP;
      ins_used = true;
      break;
    case POS_TYPE_INS_SBAS:
      fix_mode = POSITION_MODE_SBAS;
      ins_used = true;
      break;
    case POS_TYPE_INS_PSRDIFF:
      fix_mode = POSITION_MODE_DGNSS;
      ins_used = true;
      break;
    case POS_TYPE_INS_RTKFLOAT:
      fix_mode = POSITION_MODE_FLOAT;
      ins_used = true;
      break;
    case POS_TYPE_INS_RTKFIXED:
      fix_mode = POSITION_MODE_FIXED;
      ins_used = true;
      break;
    default:
      fix_mode = POSITION_MODE_SPP;
      break;
  }

  if (ins_used) {
    fix_mode |= 8;
  }

  return fix_mode;
}

/*
 * BESTPOS provides height above mean sea level whereas SBP expects
 * height above WGS84 ellipsoid
 */
static double convert_to_wgs84_height(double lat, double lon, double hgt) {
  double sep = get_geoid_offset(lat * D2R, lon * D2R);
  return hgt + sep;
}

/**
 * Novatel LLH accuracy is in meters, SBP is in millimeters.
 * For the horizontal accuracy, we just take the max of the lat/lon accuracies
 * provided by Novatel.
 */
void convert_bestpos_to_pos_llh(const BinaryHeader *header,
                                const Message::BESTPOS_t *bestpos,
                                msg_pos_llh_t *pos_llh) {
  uint32_t tow = static_cast<uint32_t>(header->ms);
  pos_llh->tow = tow;
  pos_llh->lat = bestpos->lat;
  pos_llh->lon = bestpos->lon;
  pos_llh->height =
      convert_to_wgs84_height(bestpos->lat, bestpos->lon, bestpos->hgt);
  pos_llh->v_accuracy = float_to_u16(1000.0f * bestpos->hgt_stddev);
  pos_llh->h_accuracy = float_to_u16(
      fmax(1000.0f * bestpos->lat_stddev, 1000.0f * bestpos->lon_stddev));
  pos_llh->n_sats = bestpos->num_sv_in_soln;
  uint8_t flags = get_position_flags(bestpos->pos_type);
  pos_llh->flags = flags;
  last_position_flags.set(tow, flags);
}

/**
 * Convert Novatel BESTVEL message to SBP VEL NED message
 */
void convert_bestvel_to_vel_ned(const BinaryHeader *header,
                                const Message::BESTVEL_t *bestvel,
                                msg_vel_ned_t *vel_ned) {
  uint32_t tow = static_cast<uint32_t>(header->ms);
  vel_ned->tow = tow;
  vel_ned->n = double_to_s32(1000. * bestvel->hor_speed *
                             cos(bestvel->trk_gnd * M_PI / 180.));
  vel_ned->e = double_to_s32(1000. * bestvel->hor_speed *
                             sin(bestvel->trk_gnd * M_PI / 180.));
  vel_ned->d = -double_to_s32(1000. * bestvel->vert_speed);
  vel_ned->h_accuracy = 0;
  vel_ned->v_accuracy = 0;
  vel_ned->n_sats = 0;
  /*
   * Note: flags are only non-zero if the BESTPOS message for this epoch
   * is present in the log before the corresponding BESTVEL message
   */
  vel_ned->flags = last_position_flags.get(tow);
}

/**
 * The Novatel header timestamp is accurate to the millisecond level.
 */
void convert_header_to_gps_time(const BinaryHeader *header,
                                msg_gps_time_t *gps_time) {
  gps_time->wn = header->week;
  gps_time->tow = static_cast<uint32_t>(header->ms);
  gps_time->ns_residual = 0;
  gps_time->flags = 0x1;  // Indicates that time source is GNSS solution.
}

/**
 * Compare two msg_gps_time_t structures, return true if equal
 */
bool gps_time_compare(const msg_gps_time_t *a, const msg_gps_time_t *b) {
  return (a->wn == b->wn && a->tow == b->tow &&
          a->ns_residual == b->ns_residual && a->flags == b->flags);
}

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
                                       msg_ephemeris_gps_t *ephem) {
  assert(gpsephem->prn <= UINT8_MAX);
  assert(gpsephem->toe <= UINT32_MAX);
  assert(gpsephem->toc <= UINT32_MAX);
  assert(gpsephem->week <= UINT16_MAX);
  ephem->common.sid.sat = static_cast<uint8_t>(gpsephem->prn);
  ephem->common.sid.code = 0x0;  // 0x0 = GPS_L1CA
  ephem->common.toe.tow = static_cast<uint32_t>(gpsephem->toe);
  ephem->common.toe.wn = static_cast<uint16_t>(gpsephem->week);
  ephem->common.ura = static_cast<float>(sqrt(gpsephem->URA));
  ephem->common.fit_interval = 14400;  // Four hours in seconds.
  ephem->common.valid = 0x1;
  ephem->common.health_bits = static_cast<uint8_t>(gpsephem->health);
  ephem->tgd = static_cast<float>(gpsephem->tgd);
  ephem->c_rc = static_cast<float>(gpsephem->crc);
  ephem->c_rs = static_cast<float>(gpsephem->crs);
  ephem->c_uc = static_cast<float>(gpsephem->cuc);
  ephem->c_us = static_cast<float>(gpsephem->cus);
  ephem->c_ic = static_cast<float>(gpsephem->cic);
  ephem->c_is = static_cast<float>(gpsephem->cis);
  ephem->dn = gpsephem->delta_N;
  ephem->m0 = gpsephem->M0;
  ephem->ecc = gpsephem->ecc;
  ephem->sqrta = sqrt(gpsephem->A);
  ephem->omega0 = gpsephem->omega0;
  ephem->omegadot = gpsephem->omega_dot;
  ephem->w = gpsephem->omega;
  ephem->inc = gpsephem->I0;
  ephem->inc_dot = gpsephem->I_dot;
  ephem->af0 = static_cast<float>(gpsephem->af0);
  ephem->af1 = static_cast<float>(gpsephem->af1);
  ephem->af2 = static_cast<float>(gpsephem->af2);
  ephem->toc.tow = static_cast<uint32_t>(gpsephem->toc);
  ephem->toc.wn = static_cast<uint16_t>(gpsephem->week);
  ephem->iode = static_cast<uint8_t>(gpsephem->iode1);
  ephem->iodc = static_cast<uint16_t>(gpsephem->iodc);
}

/**
 * Straightforward extraction of Novatel's GLONASS ephemeris
 * message. Notably, the Novatel health bits do not follow the
 * standard used in SwiftNav.
 */
void convert_gloephemeris_to_ephemeris_glo(
    const Message::GLOEPHEMERIS_t *gloephem, msg_ephemeris_glo_t *ephem) {
  assert(0 && "Partially implemented. Not tested or ready for use.");
  ephem->common.sid.sat = 0;
  ephem->common.sid.code = 0;
  ephem->common.toe.tow = 0;
  ephem->common.toe.wn = 0;
  ephem->common.ura = 0;
  ephem->common.fit_interval = 0;
  ephem->common.valid = 0x1;      //
  ephem->common.health_bits = 0;  // THIS NEEDS CONVERSION.
  ephem->gamma = static_cast<float>(gloephem->gamma);
  ephem->tau = static_cast<float>(gloephem->tau_n);
  ephem->d_tau = static_cast<float>(gloephem->delta_tau_n);
  ephem->pos[0] = gloephem->pos_x;
  ephem->pos[1] = gloephem->pos_y;
  ephem->pos[2] = gloephem->pos_z;
  ephem->vel[0] = gloephem->vel_x;
  ephem->vel[1] = gloephem->vel_y;
  ephem->vel[2] = gloephem->vel_z;
  ephem->acc[0] = static_cast<float>(gloephem->LS_acc_x);
  ephem->acc[1] = static_cast<float>(gloephem->LS_acc_y);
  ephem->acc[2] = static_cast<float>(gloephem->LS_acc_z);
  ephem->fcn = static_cast<uint8_t>(gloephem->sloto);
  ephem->iod = static_cast<uint8_t>(gloephem->issue);
}

/**
 * Use the tracking status bitfield of a Novatel RANGECMP message
 * to determine the satellite code identifier.
 */
code_t convert_tracking_status_to_gnss_code(uint32_t tracking_status) {
  static constexpr uint32_t kSatelliteSystemOffset = 16;
  static constexpr uint32_t kSatelliteSystemMask = 0x00070000;
  uint32_t satellite_system =
      ((tracking_status & kSatelliteSystemMask) >> kSatelliteSystemOffset);

  static constexpr uint32_t kSignalTypeOffset = 21;
  static constexpr uint32_t kSignalTypeMask = 0x03F00000;
  uint32_t signal_type =
      ((tracking_status & kSignalTypeMask) >> kSignalTypeOffset);

  // Novatel satellite systems.
  enum {
    kGPS = 0,
    kGLONASS = 1,
    kSBAS = 2,
    kGalileo = 3,
    kBeiDou = 4,
    kQZSS = 5,
    kReserved = 6,
    kOther = 7,
  };

  // Novatel signal types.
  enum {
    kGPS_L1CA = 0,
    kGPS_L2P = 5,
    kGPS_L2P_codeless = 9,
    kGPS_L5Q = 14,
    kGPS_L2C = 17,
    kGLO_L1CA = 0,
    kGLO_L2CA = 1,
    kGLO_L2P = 2,
    kGalileo_E1C = 2,
    kGalileo_E5aQ = 12,
    kGalileo_E5bQ = 17,
    kGalileo_AltBOCQ = 20,
    kSBAS_L1CA = 0,
    kSBAS_L5I = 6,
    kBeiDou_B1D1 = 0,
    kBeiDou_B2D1 = 1,
    kBeiDou_B1D2 = 4,
    kBeiDou_B2D2 = 5,
    kQZSS_L1CA = 0,
    kQZSS_L5Q = 14,
    kQZSS_L2C = 17,
    kOther_L = 19,
  };

  // Saddle up your horses...
  switch (satellite_system) {
    case kGPS:
      switch (signal_type) {
        case kGPS_L1CA:
          return CODE_GPS_L1CA;
        case kGPS_L2P:
          return CODE_GPS_L2P;
        default:
          break;
      }
      break;
    case kGLONASS:
      switch (signal_type) {
        case kGLO_L1CA:
          return CODE_GLO_L1OF;
        case kGLO_L2CA:
          return CODE_GLO_L2OF;
        default:
          break;
      }
      break;
    // Not implemented.
    case kSBAS:
      break;
    case kGalileo:
      break;
    case kBeiDou:
      break;
    case kQZSS:
      break;
    case kOther:
      break;
    default:
      break;
  }
  // assert(0 && "Unsupported signal.");
  return CODE_INVALID;
}

/**
 * Use the tracking status bitfield of a Novatel RANGECMP message to
 * determine the measurement status flags as used in a navmeas struct.
 */
nav_meas_flags_t convert_tracking_status_to_navmeas_flags(
    uint32_t tracking_status) {
  static constexpr uint32_t kCodeLockBitMask = 0x00001000;
  static constexpr uint32_t kPhaseLockBitMask = 0x00000400;
  static constexpr uint32_t kHalfCycleBitMask = 0x10000000;
  static constexpr uint32_t kParityKnownBitMask = 0x00000800;
  static constexpr uint32_t kPhaseValidBitMask =
      (kPhaseLockBitMask | kParityKnownBitMask);
  nav_meas_flags_t flags = 0;
  // clang-format off
  flags |= ((tracking_status & kCodeLockBitMask) != 0)   ? NAV_MEAS_FLAG_CODE_VALID         : 0;
  flags |= ((tracking_status & kPhaseValidBitMask) != 0) ? NAV_MEAS_FLAG_PHASE_VALID        : 0;
  flags |= ((tracking_status & kCodeLockBitMask) != 0)   ? NAV_MEAS_FLAG_MEAS_DOPPLER_VALID : 0;
  flags |= ((tracking_status & kHalfCycleBitMask) != 0)  ? NAV_MEAS_FLAG_HALF_CYCLE_KNOWN   : 0;
  flags |= ((tracking_status & kCodeLockBitMask) != 0)   ? NAV_MEAS_FLAG_CN0_VALID          : 0;
  // clang-format on
  return flags;
}

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
    mini_navigation_measurement_t *nm) {
  nm->raw_pseudorange = static_cast<double>(record->pseudorange) / 128.0;
  nm->raw_carrier_phase = static_cast<double>(record->ADR) / 256.0;
  nm->raw_measured_doppler = static_cast<double>(record->doppler) / 256.0;
  nm->cn0 = static_cast<double>(record->CN0) + 20.0;
  nm->lock_time = static_cast<double>(record->lock_time) / 32.0;
  nm->sid.sat = record->prn_slot;
  nm->sid.code = convert_tracking_status_to_gnss_code(record->tracking_status);
  nm->flags = convert_tracking_status_to_navmeas_flags(record->tracking_status);
}

/**
 * Convert Novatel INSATT message to SBP Orient Euler message
 */
void convert_insatt_to_orient_euler(const BinaryHeader *header,
                                    const Message::INSATT_t *insatt,
                                    msg_orient_euler_t *orient_euler) {
  orient_euler->tow = static_cast<uint32_t>(header->ms);

  orient_euler->roll = double_to_s32(degrees_to_microdegrees(insatt->roll));
  orient_euler->pitch = double_to_s32(degrees_to_microdegrees(insatt->pitch));
  orient_euler->yaw = double_to_s32(degrees_to_microdegrees(insatt->azimuth));

  // accuracy values are not available from INSATT
  orient_euler->roll_accuracy = 0.;
  orient_euler->pitch_accuracy = 0.;
  orient_euler->yaw_accuracy = 0.;

  orient_euler->flags = (insatt->status == INS_SOLUTION_GOOD) ? 1 : 0;
}

/**
 * Convert Novatel RAWIMUSX message to SBP Angular Rate message
 */
void convert_rawimu_to_angular_rate(const BinaryHeader *header,
                                    const Message::RAWIMUSX_t *rawimu,
                                    msg_angular_rate_t *angular_rate) {
  angular_rate->tow = static_cast<uint32_t>(header->ms);

  assert(!(rawimu->imu_info & 1));  // no IMU error
  assert(!(rawimu->imu_info & 2));  // IMU is not encrypted
  assert(rawimu->imu_type == HG4930_AN01);

  // x100 gives radians per second
  angular_rate->x = double_to_s32(
      radians_to_microdegrees((float)rawimu->x_gyro * 100.));  // NOLINT
  angular_rate->y = double_to_s32(
      radians_to_microdegrees((float)rawimu->y_gyro * 100.));  // NOLINT
  angular_rate->z = double_to_s32(
      radians_to_microdegrees((float)rawimu->z_gyro * 100.));  // NOLINT

  angular_rate->flags = 1;  // INS valid
}

}  // namespace Novatel
