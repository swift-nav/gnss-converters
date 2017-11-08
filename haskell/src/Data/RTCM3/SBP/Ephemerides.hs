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
  ( converter
  ) where

import BasicPrelude
import Control.Lens
import Data.Bits
import Data.Conduit
import Data.RTCM3
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
uriToUra :: Double -> Double
uriToUra uri
  | uri < 0 = -1
  | uri == 1 = 2.8
  | uri == 3 = 5.7
  | uri == 5 = 11.3
  | uri == 15 = 6144
  | uri <= 6 = 2 ** (1 + (uri / 2))
  | uri > 6 && uri < 15 = 2 ** (uri - 2)
  | otherwise = -1

-- | Decode SBP 'fitInterval' from RTCM/GPS 'fitIntervalFlag' and IODC.
-- Implementation adapted from libswiftnav/ephemeris.c.
fitInterval :: Bool -> Word16 -> Word32
fitInterval fitInt iodc
  | not fitInt = 4 * 60 * 60
  | iodc >= 240 && iodc <= 247 = 8 * 60 * 60
  | (iodc >= 248 && iodc <= 255) || iodc == 496 = 14 * 60 * 60
  | (iodc >= 497 && iodc <= 503) || (iodc >= 1021 && iodc <= 1023) = 26 * 60 * 60
  | iodc >= 504 && iodc <= 510 = 50 * 60 * 60
  | iodc == 511 || (iodc >= 752 && iodc <= 756) = 74 * 60 * 60
  | iodc == 757 = 98 * 60 * 60
  | otherwise = 6 * 60 * 60

-- | Construct an EphemerisCommonContent from an RTCM 1019 message.
toGpsEphemerisCommonContent :: MonadStore e m => Msg1019 -> m EphemerisCommonContent
toGpsEphemerisCommonContent m = do
  toe <- toGpsTimeSec (m ^. msg1019_ephemeris ^. gpsEphemeris_wn) (m ^. msg1019_ephemeris ^. gpsEphemeris_toe)
  pure EphemerisCommonContent
    { _ephemerisCommonContent_sid = GnssSignal
      { _gnssSignal_sat  = m ^. msg1019_header ^. gpsEphemerisHeader_sat
      , _gnssSignal_code = 0 -- there is an L2P status flag in msg 1019, but I don't think that applies
      }
    , _ephemerisCommonContent_toe          = toe
    , _ephemerisCommonContent_ura          = uriToUra (fromIntegral $ m ^. msg1019_ephemeris ^. gpsEphemeris_svHealth)
    , _ephemerisCommonContent_fit_interval = fitInterval (m ^. msg1019_ephemeris ^. gpsEphemeris_fitInterval) (m ^. msg1019_ephemeris ^. gpsEphemeris_iodc)
    , _ephemerisCommonContent_valid        = validateIodcIode (m ^. msg1019_ephemeris ^. gpsEphemeris_iodc) (m ^. msg1019_ephemeris ^. gpsEphemeris_iode)
    , _ephemerisCommonContent_health_bits  = m ^. msg1019_ephemeris ^. gpsEphemeris_svHealth
    }

--------------------------------------------------------------------------------
-- RTCM to SBP conversion utilities: RTCM Msgs. 1002 (L1 RTK), 1004 (L1+L2 RTK),
-- 1005 (antenna position), 1006 (antenna position).

-- | Convert an RTCM 1019 GPS ephemeris message into an SBP MsgEphemerisGps.
converter :: MonadStore e m => Msg1019 -> Conduit i m [SBPMsg]
converter m = do
  common <- toGpsEphemerisCommonContent m
  toc    <- toGpsTimeSec (m ^. msg1019_ephemeris ^. gpsEphemeris_wn) (m ^. msg1019_ephemeris ^. gpsEphemeris_toc)
  let pi'    = 3.1415926535898
      p ## x = (2 ** p) * fromIntegral x
      m'     = MsgEphemerisGps
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
