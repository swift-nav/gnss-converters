#ifndef GNSS_CONVERTERS_EPH_SAT_DATA_H
#define GNSS_CONVERTERS_EPH_SAT_DATA_H

#include <swiftnav/signal.h>

struct sat_data {
  struct subframe {
    u32 words[10];
  } sf[3];
  unsigned vmask;
};

struct gal_sat_data {
  struct page {
    u32 words[8];
  } pg[5];
  unsigned vmask;
};

struct glo_sat_data {
  struct string {
    u32 words[4];
  } string[5];
  unsigned vmask;
  u16 curr_superframe_id;
};

struct eph_sat_data {
  struct sat_data gps_sat_data[NUM_SATS_GPS];
  struct sat_data bds_sat_data[NUM_SATS_BDS];
  struct gal_sat_data gal_sat_data[NUM_SATS_GAL];
  struct glo_sat_data glo_sat_data[NUM_SATS_GLO];
};

#endif
