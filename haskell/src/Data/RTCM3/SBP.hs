{-# LANGUAGE LambdaCase        #-}
{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Conversions.

module Data.RTCM3.SBP
  ( toWn
  , mjdEpoch
  , converter
  , newStore
  , validateIodcIode
  , gpsUriToUra
  ) where

import           BasicPrelude
import           Control.Lens
import           Data.Bits
import           Data.Conduit
import           Data.IORef
import           Data.RTCM3
import qualified Data.RTCM3.SBP.Observations as Observations
import           Data.RTCM3.SBP.Time
import           Data.RTCM3.SBP.Types
import           Data.Word
import           SwiftNav.SBP

{-# ANN module ("HLint: ignore Redundant if"::String) #-}

--------------------------------------------------------------------------------
-- SBP, GNSS, RTCM constant definitions

-- | The official GPS value of Pi. This is the value used by the CS to curve
-- fit ephemeris parameters and should be used in all ephemeris calculations.
gpsPi :: Double
gpsPi = 3.1415926535898

--------------------------------------------------------------------------------
-- General utilities

applyScaleFactor :: (Floating a, Integral b) => a -> b -> a
applyScaleFactor p x = (2 ** p)  * fromIntegral x

--------------------------------------------------------------------------------
-- GNSS RTCM observation reconstruction utilities


fromEcefVal :: Int64 -> Double
fromEcefVal x = fromIntegral x / 10000

-- | Produce GPS time from RTCM time (week-number % 1024, tow in seconds, scaled 2^4).
-- Stateful but non-side-effecting. The week number is mod-1024 - add in the base
-- wn from the stored wn masked to 10 bits.
toGpsTimeSec :: MonadStore e m => Word16 -> Word16 -> m GpsTimeSec
toGpsTimeSec wn tow = do
  stateWn <- view storeWn >>= liftIO . readIORef
  return GpsTimeSec
    { _gpsTimeSec_tow = 16 * fromIntegral tow
    , _gpsTimeSec_wn  = stateWn `shiftR` 10 `shiftL` 10 + wn
    }

-- | MJD GPS Epoch - First day in GPS week 0. See DF051 of the RTCM3 spec
--
mjdEpoch :: Word16
mjdEpoch = 44244

-- | Convert from MJD to GPS week number
--
-- See DF051 of the RTCM3 spec
toWn :: Word16 -> Word16
toWn mjd = (mjd - mjdEpoch) `div` 7

-- | Validate IODE/IODC flags. The IODC and IODE flags (least significant bits) should be
-- equal. IODC is Word16 while IODE is Word8, so we shift IODC to drop the most significant bits.
-- We return a 1 if valid, 0 if invalid.
validateIodcIode :: Word16 -> Word8 -> Word8
validateIodcIode iodc iode =
  if iodc' == iode then 1 else 0
  where
    iodc' = fromIntegral $ iodc `shiftL` 8 `shiftR` 8

-- | Construct an EphemerisCommonContent from an RTCM 1019 message.
toGpsEphemerisCommonContent :: MonadStore e m => Msg1019 -> m EphemerisCommonContent
toGpsEphemerisCommonContent m = do
  toe <- toGpsTimeSec (m ^. msg1019_ephemeris ^. gpsEphemeris_wn) (m ^. msg1019_ephemeris ^. gpsEphemeris_toe)
  return EphemerisCommonContent
    { _ephemerisCommonContent_sid = GnssSignal16
      { _gnssSignal16_sat  = m ^. msg1019_header ^. gpsEphemerisHeader_sat
      , _gnssSignal16_code = 0 -- there is an L2P status flag in msg 1019, but I don't think that applies
      }
    , _ephemerisCommonContent_toe          = toe
    , _ephemerisCommonContent_ura          = gpsUriToUra (fromIntegral $ m ^. msg1019_ephemeris ^. gpsEphemeris_svHealth)
    , _ephemerisCommonContent_fit_interval = decodeFitInterval (m ^. msg1019_ephemeris ^. gpsEphemeris_fitInterval) (m ^. msg1019_ephemeris ^. gpsEphemeris_iodc)
    , _ephemerisCommonContent_valid        = validateIodcIode (m ^. msg1019_ephemeris ^. gpsEphemeris_iodc) (m ^. msg1019_ephemeris ^. gpsEphemeris_iode)
    , _ephemerisCommonContent_health_bits  = m ^. msg1019_ephemeris ^. gpsEphemeris_svHealth
    }

-- | 16 bit SBP Sender Id is 12 bit RTCM Station Id with high nibble or'd in
--
toSender :: Word16 -> Word16
toSender = (.|. 0xf000)

-- | Decode SBP 'fitInterval' from RTCM/GPS 'fitIntervalFlag' and IODC.
-- Implementation adapted from libswiftnav/ephemeris.c.
decodeFitInterval :: Bool -> Word16 -> Word32
decodeFitInterval fitInt iodc
  | not fitInt = 4 * 60 * 60
  | iodc >= 240 && iodc <= 247 = 8 * 60 * 60
  | (iodc >= 248 && iodc <= 255) || iodc == 496 = 14 * 60 * 60
  | (iodc >= 497 && iodc <= 503) || (iodc >= 1021 && iodc <= 1023) = 26 * 60 * 60
  | iodc >= 504 && iodc <= 510 = 50 * 60 * 60
  | iodc == 511 || (iodc >= 752 && iodc <= 756) = 74 * 60 * 60
  | iodc == 757 = 98 * 60 * 60
  | otherwise = 6 * 60 * 60

-- | Convert between RTCM/GPS URA ("User Range Accuracy") index to a number in meters.
-- See section 2.5.3, "User Range Accuracy", in the GPS signal specification.
-- Indices 1, 3, and 5 are hard-coded according to spec, and 15 is hard-coded according
-- to SBP/Piksi convention.
gpsUriToUra :: Double -> Double
gpsUriToUra uri
  | uri < 0 = -1
  | uri == 1 = 2.8
  | uri == 3 = 5.7
  | uri == 5 = 11.3
  | uri == 15 = 6144
  | uri <= 6 = 2 ** (1 + (uri / 2))
  | uri > 6 && uri < 15 = 2 ** (uri - 2)
  | otherwise = -1

--------------------------------------------------------------------------------
-- RTCM to SBP conversion utilities: RTCM Msgs. 1002 (L1 RTK), 1004 (L1+L2 RTK),
-- 1005 (antenna position), 1006 (antenna position).

-- | Convert an RTCM 1005 antenna reference position message into an SBP
-- MsgBasePosEcef.
fromMsg1005 :: MonadStore e m => Msg1005 -> m MsgBasePosEcef
fromMsg1005 m =
  return MsgBasePosEcef
    { _msgBasePosEcef_x = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_x
    , _msgBasePosEcef_y = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_y
    , _msgBasePosEcef_z = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_z
    }

-- | Convert an RTCM 1006 antenna reference position message into an SBP
-- MsgBasePosEcef.
fromMsg1006 :: MonadStore e m => Msg1006 -> m MsgBasePosEcef
fromMsg1006 m =
  return MsgBasePosEcef
    { _msgBasePosEcef_x = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_x
    , _msgBasePosEcef_y = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_y
    , _msgBasePosEcef_z = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_z
    }

-- | Convert an RTCM 1019 GPS ephemeris message into an SBP MsgEphemerisGps.
fromMsg1019 :: MonadStore e m => Msg1019 -> m MsgEphemerisGps
fromMsg1019 m = do
  commonContent <- toGpsEphemerisCommonContent m
  toc           <- toGpsTimeSec (m ^. msg1019_ephemeris ^. gpsEphemeris_wn) (m ^. msg1019_ephemeris ^. gpsEphemeris_toc)
  return MsgEphemerisGps
    { _msgEphemerisGps_common   = commonContent
    , _msgEphemerisGps_tgd      =         applyScaleFactor (-31) $ m ^. msg1019_ephemeris ^. gpsEphemeris_tgd
    , _msgEphemerisGps_c_rs     =         applyScaleFactor (-5)  $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_rs
    , _msgEphemerisGps_c_rc     =         applyScaleFactor (-5)  $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_rc
    , _msgEphemerisGps_c_uc     =         applyScaleFactor (-29) $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_uc
    , _msgEphemerisGps_c_us     =         applyScaleFactor (-29) $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_us
    , _msgEphemerisGps_c_ic     =         applyScaleFactor (-29) $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_ic
    , _msgEphemerisGps_c_is     =         applyScaleFactor (-29) $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_is
    , _msgEphemerisGps_dn       = gpsPi * applyScaleFactor (-43)  (m ^. msg1019_ephemeris ^. gpsEphemeris_dn)
    , _msgEphemerisGps_m0       = gpsPi * applyScaleFactor (-31)  (m ^. msg1019_ephemeris ^. gpsEphemeris_m0)
    , _msgEphemerisGps_ecc      =         applyScaleFactor (-33) $ m ^. msg1019_ephemeris ^. gpsEphemeris_ecc
    , _msgEphemerisGps_sqrta    =         applyScaleFactor (-19) $ m ^. msg1019_ephemeris ^. gpsEphemeris_sqrta
    , _msgEphemerisGps_omega0   = gpsPi * applyScaleFactor (-31)  (m ^. msg1019_ephemeris ^. gpsEphemeris_omega0)
    , _msgEphemerisGps_omegadot = gpsPi * applyScaleFactor (-43)  (m ^. msg1019_ephemeris ^. gpsEphemeris_omegadot)
    , _msgEphemerisGps_w        = gpsPi * applyScaleFactor (-31)  (m ^. msg1019_ephemeris ^. gpsEphemeris_w)
    , _msgEphemerisGps_inc      = gpsPi * applyScaleFactor (-31)  (m ^. msg1019_ephemeris ^. gpsEphemeris_i0)
    , _msgEphemerisGps_inc_dot  = gpsPi * applyScaleFactor (-43)  (m ^. msg1019_ephemeris ^. gpsEphemeris_idot)
    , _msgEphemerisGps_af0      =         applyScaleFactor (-31) $ m ^. msg1019_ephemeris ^. gpsEphemeris_af0
    , _msgEphemerisGps_af1      =         applyScaleFactor (-43) $ m ^. msg1019_ephemeris ^. gpsEphemeris_af1
    , _msgEphemerisGps_af2      =         applyScaleFactor (-55) $ m ^. msg1019_ephemeris ^. gpsEphemeris_af2
    , _msgEphemerisGps_iodc     =                                  m ^. msg1019_ephemeris ^. gpsEphemeris_iodc
    , _msgEphemerisGps_iode     =                                  m ^. msg1019_ephemeris ^. gpsEphemeris_iode
    , _msgEphemerisGps_toc      = toc
    }

msg1005Converter :: MonadStore e m => Msg1005 -> Conduit i m [SBPMsg]
msg1005Converter m = do
  m' <- fromMsg1005 m
  yield [SBPMsgBasePosEcef m' $ toSBP m' $ toSender $ m ^. msg1005_reference ^. antennaReference_station]

msg1006Converter :: MonadStore e m => Msg1006 -> Conduit i m [SBPMsg]
msg1006Converter m = do
  m' <- fromMsg1006 m
  yield [SBPMsgBasePosEcef m' $ toSBP m' $ toSender $ m ^. msg1006_reference ^. antennaReference_station]

msg1019Converter :: MonadStore e m => Msg1019 -> Conduit i m [SBPMsg]
msg1019Converter m = do
  m' <- fromMsg1019 m
  yield [SBPMsgEphemerisGps m' $ toSBP m' $ toSender 0]

-- | Convert an RTCM message into possibly multiple SBP messages.
--
converter :: MonadStore e m => Conduit RTCM3Msg m [SBPMsg]
converter =
  awaitForever $ \case
    (RTCM3Msg1002 m _rtcm3) -> Observations.converter m
    (RTCM3Msg1004 m _rtcm3) -> Observations.converter m
    (RTCM3Msg1010 m _rtcm3) -> Observations.converter m
    (RTCM3Msg1012 m _rtcm3) -> Observations.converter m
    (RTCM3Msg1005 m _rtcm3) -> msg1005Converter m
    (RTCM3Msg1006 m _rtcm3) -> msg1006Converter m
    (RTCM3Msg1019 m _rtcm3) -> msg1019Converter m
    _rtcm3Msg -> mempty

newStore :: IO Store
newStore = do
  t <- currentGpsTime
  Store <$> newIORef (t ^. gpsTimeNano_wn) <*> newIORef t <*> newIORef mempty <*> newIORef mempty
