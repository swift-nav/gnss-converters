{-# LANGUAGE LambdaCase #-}

-- |
-- Module:      Data.RTCM3.SBP
-- Copyright:   Copyright (C) 2015 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Mark Fine <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Conversions.

module Data.RTCM3.SBP
  ( convert
  ) where

import BasicPrelude
import Control.Lens
import Data.Bits
import Data.Time
import Data.Word
import Data.RTCM3
import SwiftNav.SBP

fromEcefVal :: Int64 -> Double
fromEcefVal x = fromIntegral x / 10000

toGPSTime :: MonadIO m => GpsObservationHeader -> m ObsGPSTime
toGPSTime hdr = do
  today <- utctDay <$> liftIO getCurrentTime
  return ObsGPSTime
    { _obsGPSTime_tow = hdr ^. gpsObservationHeader_tow
    , _obsGPSTime_wn  = fromIntegral $ div (diffDays today (fromGregorian 1980 1 6)) 7
    }

fromGpsObservationHeader :: MonadIO m => GpsObservationHeader -> m ObservationHeader
fromGpsObservationHeader hdr = do
  t <- toGPSTime hdr
  return ObservationHeader
    { _observationHeader_t     = t
    , _observationHeader_n_obs = 0x10
    }

toP :: GpsL1Observation -> GpsL1ExtObservation -> Word32
toP l1 l1e = round $ p * 100 where
  p = (0.02 :: Double) * fromIntegral (l1 ^. gpsL1Observation_pseudorange) +
      299792.458 * fromIntegral (l1e ^. gpsL1ExtObservation_ambiguity)

toL :: GpsL1Observation -> GpsL1ExtObservation -> CarrierPhase
toL l1 l1e = CarrierPhase
  { _carrierPhase_i = fromIntegral $ iint `shiftR` 8
  , _carrierPhase_f = fromIntegral $ iint .&. 0xFF
  } where
    p = 0.02 * fromIntegral (l1 ^. gpsL1Observation_pseudorange) +
        299792.458 * fromIntegral (l1e ^. gpsL1ExtObservation_ambiguity)
    lm :: Double
    lm = p + 0.0005 * fromIntegral (l1 ^. gpsL1Observation_carrierMinusCode)
    l = (lm - 22e6) / 0.190293673
    iint :: Int64
    iint = - round (l * 255.0)

toCn0 :: GpsL1ExtObservation -> Word8
toCn0 = (^. gpsL1ExtObservation_cnr)

toLock :: GpsL1Observation -> Word16
toLock _l1 = 0

toSid :: Word8 -> GnssSignal
toSid sat = GnssSignal
  { _gnssSignal_sat           = fromIntegral $ sat - 1
  , _gnssSignal_band          = 0
  , _gnssSignal_constellation = 0
  }

fromObservation1002 :: Observation1002 -> Maybe PackedObsContent
fromObservation1002 obs =
  if obs ^. observation1002_l1 ^. gpsL1Observation_code then Nothing else Just PackedObsContent
    { _packedObsContent_P    = toP l1 l1e
    , _packedObsContent_L    = toL l1 l1e
    , _packedObsContent_cn0  = toCn0 l1e
    , _packedObsContent_lock = toLock l1
    , _packedObsContent_sid  = toSid sat
    } where
      sat = obs ^. observation1002_sat
      l1  = obs ^. observation1002_l1
      l1e = obs ^. observation1002_l1e

fromObservation1004 :: Observation1004 -> Maybe PackedObsContent
fromObservation1004 obs =
  if obs ^. observation1004_l1 ^. gpsL1Observation_code then Nothing else Just PackedObsContent
    { _packedObsContent_P    = toP l1 l1e
    , _packedObsContent_L    = toL l1 l1e
    , _packedObsContent_cn0  = toCn0 l1e
    , _packedObsContent_lock = toLock l1
    , _packedObsContent_sid  = toSid sat
    }  where
      sat = obs ^. observation1004_sat
      l1  = obs ^. observation1004_l1
      l1e = obs ^. observation1004_l1e

fromMsg1002 :: MonadIO m => Msg1002 -> m MsgObs
fromMsg1002 msg = do
  header <- fromGpsObservationHeader $ msg ^. msg1002_header
  return MsgObs
    { _msgObs_header = header
    , _msgObs_obs    = mapMaybe fromObservation1002 $ msg ^. msg1002_observations
    }

fromMsg1004 :: MonadIO m => Msg1004 -> m MsgObs
fromMsg1004 msg = do
  header <- fromGpsObservationHeader $ msg ^. msg1004_header
  return MsgObs
    { _msgObs_header = header
    , _msgObs_obs    = mapMaybe fromObservation1004 $ msg ^. msg1004_observations
    }

fromMsg1005 :: MonadIO m => Msg1005 -> m MsgBasePosEcef
fromMsg1005 msg =
  return MsgBasePosEcef
    { _msgBasePosEcef_x = fromEcefVal $ msg ^. msg1005_reference ^. antennaReference_ecef_x
    , _msgBasePosEcef_y = fromEcefVal $ msg ^. msg1005_reference ^. antennaReference_ecef_y
    , _msgBasePosEcef_z = fromEcefVal $ msg ^. msg1005_reference ^. antennaReference_ecef_z
    }

fromMsg1006 :: MonadIO m => Msg1006 -> m MsgBasePosEcef
fromMsg1006 msg =
  return MsgBasePosEcef
    { _msgBasePosEcef_x = fromEcefVal $ msg ^. msg1006_reference ^. antennaReference_ecef_x
    , _msgBasePosEcef_y = fromEcefVal $ msg ^. msg1006_reference ^. antennaReference_ecef_y
    , _msgBasePosEcef_z = fromEcefVal $ msg ^. msg1006_reference ^. antennaReference_ecef_z
    }

convert :: MonadIO m => RTCM3Msg -> m (Maybe SBPMsg)
convert = \case
  (RTCM3Msg1002 msg _rtcm3) -> do
    msg' <- fromMsg1002 msg
    return $ Just $ SBPMsgObs msg' $ toSBP msg' defaultSender
  (RTCM3Msg1004 msg _rtcm3) -> do
    msg' <- fromMsg1004 msg
    return $ Just $ SBPMsgObs msg' $ toSBP msg' defaultSender
  (RTCM3Msg1005 msg _rtcm3) -> do
    msg' <- fromMsg1005 msg
    return $ Just $ SBPMsgBasePosEcef msg' $ toSBP msg' defaultSender
  (RTCM3Msg1006 msg _rtcm3) -> do
    msg' <- fromMsg1006 msg
    return $ Just $ SBPMsgBasePosEcef msg' $ toSBP msg' defaultSender
  _rtcm3Msg -> return Nothing
