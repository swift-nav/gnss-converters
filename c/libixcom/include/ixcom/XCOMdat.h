/*
 * Copyright 2020:
 * iMAR Navigation GmbH,
 * Im Reihersbruch 3,
 * D-66386 St. Ingbert
 */
#ifndef XCOMDAT_H_
#define XCOMDAT_H_

#include <stdint.h>

#include <swiftnav/macros.h>

/*
 * General
 */
#define XCOM_SYNC_BYTE 0x7E /**< Synchronization character */
/*
 * iXCOM Message IDs
 */
enum XComMessageID {
  XCOM_MSGID_IMURAW = 0x00,    /**< IMU data - calibrated */
  XCOM_MSGID_WHEELDATA = 0x16, /**< (Single-) Odometer measurements */
};

typedef struct SWIFT_ATTR_PACKED {
  uint8_t sync;          /**< Synchronization character (is always set to 0x7E;
                            XCOM_SYNC_BYTE) */
  uint8_t msg_id;        /**< iXCOM message ID. iXCOM distinguishes between four
                            different types of message IDs (ordinary message IDs;
                            command ID; parameter ID, response ID) */
  uint8_t frame_counter; /**< The frame counter counts from 0 to 255 and starts
                            at 0 again after it has reached 255. For messages,
                            this can be used to detect lost messages. For
                            parameters and commands, the system response to an
                            input parameter or command will contain the same
                            frame counter as the input. */
  uint8_t trigger_source; /**< See <iXCOM LOG command trigger type> */
  uint16_t msg_len;  /**< Length of the overall message (from sync byte to crc16
                        field), depends on the message log (num- ber of bytes) */
  uint16_t gps_week; /**< GPS week number */
  uint32_t gps_time_sec;  /**< GPS time of week - integer part in [sec] */
  uint32_t gps_time_usec; /**< GPS time of week - fractional part in [μs] */
} XCOMHeader;

/**
 * The global status is a summarized status word that contains a combination of
 * several status bits. This status word should be used to evaluate the data
 * integrity and is attached as part of the footer at the end of every message.
 * If an error occurs, the detailed status should be read.
 */
typedef struct {
  uint16_t HWERROR : 1;  /**< Unexpected hardware error. */
  uint16_t COMERROR : 1; /**< Internal communication error between the
                            individual components. */
  uint16_t
      NAVERROR : 1; /**< Internal error in the navigation solution. If this bit
                       is Critical set the INS solution should not be used. */
  uint16_t CALERROR : 1;     /**< Internal error in the calibration table. */
  uint16_t GYROVERRANGE : 1; /**< Gyro over range error. To see which axis is
                                affected, the extended system status should be
                                triggered. */
  uint16_t ACCOVERRAGE : 1;  /**< Acceleration over range error. To see which
                                axis is affected, the extended system status
                                should be triggered. */
  uint16_t GNSSINVALID : 1;  /**< GNSS solution is invalid */
  uint16_t STANDBY : 1;      /**< System stays in standby mode. */
  uint16_t DYNAMICALIGN : 1; /**< The INS is in dynamic alignment mode */
  uint16_t TIMEINVALID : 1;  /**< The message time tag in the header represents
                                internal system time and has not been set from
                                GNSS. */
  uint16_t NAVMODE : 1;      /**< The INS has reached the fine-heading mode */
  uint16_t AHRSMODE : 1;     /**< The INS has reached the AHRS mode*/
  uint16_t
      ALIGNMODE : 2; /**< Alignment mode
                                                      0: LEVELLING: This
                        solution status is set during the levelling phase. 1:
                        STATIONARY_ALIGN: This solution status is set during
                        stationary alignment of the INS. 2: ALIGNMENT_COMPLETE:
                        This solution status is set if the alignment has
                        finished and the standard deviation of the estimated
                        heading is larger than THR_HEADING (set through
                        parameter PAREKF_HDGPOSTHR). 3: HEADING_GOOD: This
                        solution status is set if the alignment has finished and
                        the standard deviation of the estimated heading is lower
                        than THR_HEADING (set through parameter
                        PAREKF_HDGPOSTHR).
                                              */
  uint16_t
      POSMODE : 2; /**< Position mode:
                                           0: POS_BAD_ACCURACY: This solution
                      status is set during alignment, if no valid position
                      solution is available. 1: POS_MEDIUM_ACCURACY: This
                      solution status is set, if the INS has calculated a valid
                      position solution and the standard deviation of the
                      estimated position is lower than THR_POS_MED AND larger
                      than THR_POS_HIGH (set through parameter
                      PAREKF_HDGPOSTHR). 2: POS_HIGH_ACCURACY: This solution
                      status is set if the INS has calculated a valid position
                      solution and the standard deviation of the estimated
                      position is lower than THR_POS_HIGH (set through parameter
                      PAREKF_HDGPOSTHR). 3: POS_UNDEFINED: The position accuracy
                      is undefined.
                                    */
} XCOMGlobalStatus;

/**
 * Global status alignment mode
 */
enum XCOMGlobalStatusAlignmode {
  Levelling = 0, /**< This solution status is set during the levelling phase */
  Aligning = 1,  /**< This solution status is set during stationary alignment of
                    the INS */
  AlignmentComplete =
      2, /**< This solution status is set if the alignment has finished and the
            standard deviation of the estimated heading is larger than
            THR_HEADING (set through parameter PAREKF_HDGPOSTHR) */
  AlignmentGood =
      3 /**< This solution status is set if the alignment has finished and the
           standard deviation of the estimated heading is lower than THR_HEADING
           (set through parameter PAREKF_HDGPOSTHR) */
};

/**
 * Global status position mode
 */
enum XCOMGlobalStatusPosMode {
  PosBad = 0,    /**< This solution status is set during alignment, if no valid
                    position solution is available */
  PosMadium = 1, /**< This solution status is set, if the INS has calculated a
                    valid position solution and the standard deviation of the
                    estimated position is lower than THR_POS_MED AND larger than
                    THR_POS_HIGH (set through parameter PAREKF_HDGPOSTHR) */
  PosHigh = 2,   /**< This solution status is set if the INS has calculated a
                    valid position solution and the standard deviation of the
                    estimated position is lower than THR_POS_HIGH (set through
                    parameter PAREKF_HDGPOSTHR) */
  PosUndefined = 3 /**< The position accuracy is undefined */
};

typedef struct SWIFT_ATTR_PACKED {
  union { /**< The global status is a summarized status word that contains a
             combination of several status bits. This status word should be used
             to evaluate the data integrity and is attached as part of the
             footer at the end of every message. If an error occurs, the
             detailed status should be read. */
    XCOMGlobalStatus bits;
    uint16_t value;
  } global_status;
  uint16_t crc16; /**< Always sent, calculated over all bytes starting with
                     synchronization byte. */
} XCOMFooter;

typedef struct SWIFT_ATTR_PACKED {
  XCOMHeader header;
  float acc[3]; /**< Calibrated acceleration along IMU x-, y- and z-axis in
                   [m/s2] */
  float omg[3]; /**< Calibrated angular rate along IMU x-, y- and z-axis in
                   [rad/s] */
  XCOMFooter footer;
} XCOMmsg_IMURAW;

typedef struct SWIFT_ATTR_PACKED {
  XCOMHeader header;
  float speed;
  int32_t ticks;
  XCOMFooter footer;
} XCOMmsg_WHEELDATA;

#endif /* XCOMDAT_H_ */
