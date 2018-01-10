{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.Ephemerides
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Ephemerides Conversions.

module Data.RTCM3.SBP.Ephemerides
  ( gpsConverter
  , glonassConverter
  ) where

import BasicPrelude
import Control.Lens
import Data.Bits
import Data.Conduit
import Data.RTCM3
import Data.RTCM3.SBP.Time
import Data.RTCM3.SBP.Types
import Data.Word
import SwiftNav.SBP

--------------------------------------------------------------------------------
-- GNSS RTCM observation reconstruction utilities

-- | Produce GPS time from RTCM time (week-number % 1024, tow in seconds, scaled 2^4).
-- Stateful but non-side-effecting. The week number is mod-1024 - add in the base
-- wn from the stored wn masked to 10 bits.
toGpsTimeSec :: MonadStore e m => Word16 -> Word16 -> m GpsTimeSec
toGpsTimeSec wn tow = do
  time <- view storeCurrentGpsTime
  wn'  <- view gpsTime_wn <$> liftIO time
  pure GpsTimeSec
    { _gpsTimeSec_tow = 16 * fromIntegral tow
    , _gpsTimeSec_wn  = wn' `shiftR` 10 `shiftL` 10 + wn
    }

-- | Produce GPS time from GLONASS RTCM time.
--
toGlonassTimeSec :: MonadStore e m => Word8 -> m GpsTimeSec
toGlonassTimeSec epoch = do
  time <- view storeCurrentGpsTime
  t    <- liftIO time
  let epoch' = fromIntegral epoch * 15 * minuteMillis - 3 * hourMillis + gpsLeapMillis
      epoch''
        | epoch' < 0 = epoch' + dayMillis
        | otherwise = epoch'
      dow = fromIntegral (t ^. gpsTime_tow) `div` dayMillis
  pure GpsTimeSec
    { _gpsTimeSec_tow = fromIntegral $ (dow * dayMillis + epoch'') `div` 1000
    , _gpsTimeSec_wn  = t ^. gpsTime_wn
    }

-- | Validate IODE/IODC flags. The IODC and IODE flags (least significant bits) should be
-- equal. IODC is Word16 while IODE is Word8, so we shift IODC to drop the most significant bits.
-- We return a 1 if valid, 0 if invalid.
validateIodcIode :: Word16 -> Word8 -> Word8
validateIodcIode iodc iode =
  bool 0 1 $ iodc' == iode
  where
    iodc' = fromIntegral $ iodc `shiftL` 8 `shiftR` 8

-- | Convert between RTCM/GPS URA ("User Range Accuracy") index to a number in meters.
-- See section 2.5.3, "User Range Accuracy", in the GPS signal specification.
-- Indices 1, 3, and 5 are hard-coded according to spec, and 15 is hard-coded according
-- to SBP/Piksi convention.
uriToUra :: Word8 -> Double
uriToUra uri
  | uri == 1 = 2.8
  | uri == 3 = 5.7
  | uri == 5 = 11.3
  | uri == 15 = 6144
  | uri <= 6 = 2 ** (1 + (fromIntegral uri / 2))
  | uri > 6 && uri < 15 = 2 ** (fromIntegral uri - 2)
  | otherwise = -1

-- | Lookup Table 4.4 from GLONASS ICD: convert F_T word to accuracy in meters
--
ftToUra :: Word8 -> Double
ftToUra ft
  | ft == 0 = 1
  | ft == 1 = 2
  | ft == 2 = 2.5
  | ft == 3 = 4
  | ft == 4 = 5
  | ft == 5 = 7
  | ft == 6 = 10
  | ft == 7 = 12
  | ft == 8 = 14
  | ft == 9 = 16
  | ft == 10 = 32
  | ft == 11 = 64
  | ft == 12 = 128
  | ft == 13 = 256
  | ft == 14 = 512
  | otherwise = -1

-- | Decode SBP 'fitInterval' from RTCM/GPS 'fitIntervalFlag' and IODC.
-- Implementation adapted from libswiftnav/ephemeris.c.
gpsFitInterval :: Bool -> Word16 -> Word32
gpsFitInterval fitInt iodc
  | not fitInt = 4 * 60 * 60
  | iodc >= 240 && iodc <= 247 = 8 * 60 * 60
  | (iodc >= 248 && iodc <= 255) || iodc == 496 = 14 * 60 * 60
  | (iodc >= 497 && iodc <= 503) || (iodc >= 1021 && iodc <= 1023) = 26 * 60 * 60
  | iodc >= 504 && iodc <= 510 = 50 * 60 * 60
  | iodc == 511 || (iodc >= 752 && iodc <= 756) = 74 * 60 * 60
  | iodc == 757 = 98 * 60 * 60
  | otherwise = 6 * 60 * 60

-- | Convert GLONASS fit interval - Table 4.3 from GLONASS ICD.
--
glonassFitInterval :: Word8 -> Word32
glonassFitInterval fi
  | fi == 0 = (60 + 10) * 60
  | fi == 1 = (30 + 10) * 60
  | fi == 2 = (45 + 10) * 60
  | fi == 3 = (60 + 10) * 60
  | otherwise = (60 + 10) * 60

-- | Construct an EphemerisCommonContent from an RTCM 1019 message.
--
toGpsEphemerisCommonContent :: MonadStore e m => Msg1019 -> m EphemerisCommonContent
toGpsEphemerisCommonContent m = do
  toe <- toGpsTimeSec (m ^. msg1019_ephemeris ^. gpsEphemeris_wn) (m ^. msg1019_ephemeris ^. gpsEphemeris_toe)
  pure EphemerisCommonContent
    { _ephemerisCommonContent_sid = GnssSignal
      { _gnssSignal_sat  = m ^. msg1019_header ^. gpsEphemerisHeader_sat
      , _gnssSignal_code = 0 -- there is an L2P status flag in msg 1019, but I don't think that applies
      }
    , _ephemerisCommonContent_toe          = toe
    , _ephemerisCommonContent_ura          = uriToUra (m ^. msg1019_ephemeris ^. gpsEphemeris_svHealth)
    , _ephemerisCommonContent_fit_interval = gpsFitInterval (m ^. msg1019_ephemeris ^. gpsEphemeris_fitInterval) (m ^. msg1019_ephemeris ^. gpsEphemeris_iodc)
    , _ephemerisCommonContent_valid        = validateIodcIode (m ^. msg1019_ephemeris ^. gpsEphemeris_iodc) (m ^. msg1019_ephemeris ^. gpsEphemeris_iode)
    , _ephemerisCommonContent_health_bits  = m ^. msg1019_ephemeris ^. gpsEphemeris_svHealth
    }

-- | Construct an EphemerisCommonContent from an RTCM 1020 message.
--
toGlonassEphemerisCommonContent :: MonadStore e m => Msg1020 -> m EphemerisCommonContent
toGlonassEphemerisCommonContent m = do
  toe <- toGlonassTimeSec (m ^. msg1020_ephemeris ^. glonassEphemeris_tb)
  pure EphemerisCommonContent
    { _ephemerisCommonContent_sid = GnssSignal
      { _gnssSignal_sat  = m ^. msg1020_header ^. glonassEphemerisHeader_sat
      , _gnssSignal_code = 3
      }
    , _ephemerisCommonContent_toe          = toe
    , _ephemerisCommonContent_ura          = ftToUra (m ^. msg1020_ephemeris ^. glonassEphemeris_mft)
    , _ephemerisCommonContent_fit_interval = glonassFitInterval (m ^. msg1020_ephemeris ^. glonassEphemeris_p1)
    , _ephemerisCommonContent_valid        = 1
    , _ephemerisCommonContent_health_bits  = bool 0 1 $ (m ^. msg1020_ephemeris ^. glonassEphemeris_mln5) || (m ^. msg1020_ephemeris ^. glonassEphemeris_mi3)
    }

(##) :: (Floating a1, Integral a2) => a1 -> a2 -> a1
(##) p x = (2 ** p) * fromIntegral x

--------------------------------------------------------------------------------
-- RTCM to SBP conversion utilities: RTCM Msgs. 1002 (L1 RTK), 1004 (L1+L2 RTK),
-- 1005 (antenna position), 1006 (antenna position).

-- | Convert an RTCM 1019 GPS ephemeris message into an SBP MsgEphemerisGps.
--
gpsConverter :: MonadStore e m => Msg1019 -> Conduit i m [SBPMsg]
gpsConverter m = do
  common <- toGpsEphemerisCommonContent m
  toc    <- toGpsTimeSec (m ^. msg1019_ephemeris ^. gpsEphemeris_wn) (m ^. msg1019_ephemeris ^. gpsEphemeris_toc)
  let pi' = 3.1415926535898
      m'  = MsgEphemerisGps
              { _msgEphemerisGps_common   = common
              , _msgEphemerisGps_tgd      =       (-31) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_tgd)
              , _msgEphemerisGps_c_rs     =        (-5) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_c_rs)
              , _msgEphemerisGps_c_rc     =        (-5) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_c_rc)
              , _msgEphemerisGps_c_uc     =       (-29) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_c_uc)
              , _msgEphemerisGps_c_us     =       (-29) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_c_us)
              , _msgEphemerisGps_c_ic     =       (-29) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_c_ic)
              , _msgEphemerisGps_c_is     =       (-29) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_c_is)
              , _msgEphemerisGps_dn       = pi' * (-43) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_dn)
              , _msgEphemerisGps_m0       = pi' * (-31) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_m0)
              , _msgEphemerisGps_ecc      =       (-33) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_ecc)
              , _msgEphemerisGps_sqrta    =       (-19) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_sqrta)
              , _msgEphemerisGps_omega0   = pi' * (-31) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_omega0)
              , _msgEphemerisGps_omegadot = pi' * (-43) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_omegadot)
              , _msgEphemerisGps_w        = pi' * (-31) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_w)
              , _msgEphemerisGps_inc      = pi' * (-31) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_i0)
              , _msgEphemerisGps_inc_dot  = pi' * (-43) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_idot)
              , _msgEphemerisGps_af0      =       (-31) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_af0)
              , _msgEphemerisGps_af1      =       (-43) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_af1)
              , _msgEphemerisGps_af2      =       (-55) ## (m ^. msg1019_ephemeris ^. gpsEphemeris_af2)
              , _msgEphemerisGps_iodc     =                 m ^. msg1019_ephemeris ^. gpsEphemeris_iodc
              , _msgEphemerisGps_iode     =                 m ^. msg1019_ephemeris ^. gpsEphemeris_iode
              , _msgEphemerisGps_toc      = toc
              }
  yield [SBPMsgEphemerisGps m' $ toSBP m' 61440]

-- | Convert an RTCM 1020 GLONASS ephemeris message into an SBP MsgEphemerisGlo.
--
glonassConverter :: MonadStore e m => Msg1020 -> Conduit i m [SBPMsg]
glonassConverter m = do
  common <- toGlonassEphemerisCommonContent m
  let m' = MsgEphemerisGlo
             { _msgEphemerisGlo_common = common
             , _msgEphemerisGlo_gamma  = (-40) ## (m ^. msg1020_ephemeris ^. glonassEphemeris_gammaN)
             , _msgEphemerisGlo_tau    = (-30) ## (m ^. msg1020_ephemeris ^. glonassEphemeris_tauN)
             , _msgEphemerisGlo_d_tau  = (-30) ## (m ^. msg1020_ephemeris ^. glonassEphemeris_mdeltatau)
             , _msgEphemerisGlo_pos    = (* 1000) . ((-11) ##) <$>
                 [ m ^. msg1020_ephemeris ^. glonassEphemeris_xn
                 , m ^. msg1020_ephemeris ^. glonassEphemeris_yn
                 , m ^. msg1020_ephemeris ^. glonassEphemeris_zn
                 ]
             , _msgEphemerisGlo_vel    = (* 1000) . ((-20) ##) <$>
                 [ m ^. msg1020_ephemeris ^. glonassEphemeris_xndot
                 , m ^. msg1020_ephemeris ^. glonassEphemeris_yndot
                 , m ^. msg1020_ephemeris ^. glonassEphemeris_zndot
                 ]
             , _msgEphemerisGlo_acc    = (* 1000) . ((-30) ##) <$>
                 [ m ^. msg1020_ephemeris ^. glonassEphemeris_xndotdot
                 , m ^. msg1020_ephemeris ^. glonassEphemeris_yndotdot
                 , m ^. msg1020_ephemeris ^. glonassEphemeris_zndotdot
                 ]
             , _msgEphemerisGlo_fcn    = (m ^. msg1020_header ^. glonassEphemerisHeader_channel) + 1
             , _msgEphemerisGlo_iod    = ((m ^. msg1020_ephemeris ^. glonassEphemeris_tb) * 15 * 60) .&. 127
             }
  yield [SBPMsgEphemerisGlo m' $ toSBP m' 61440]
