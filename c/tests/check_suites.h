#ifndef CHECK_SUITES_H
#define CHECK_SUITES_H

Suite* utils_suite(void);
Suite* rtcm3_suite(void);
Suite* rtcm3_ssr_suite(void);
Suite* nmea_suite(void);
Suite* ubx_suite(void);
Suite* ixcom_suite(void);
Suite* options_suite(void);
Suite* rtcm3_time_suite(void);
Suite* time_truth_suite(void);

#endif /* CHECK_SUITES_H */
