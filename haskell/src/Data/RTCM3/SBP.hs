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
import Data.Word
import Data.RTCM3
import SwiftNav.SBP

fromEcefVal :: Int64 -> Double
fromEcefVal x = fromIntegral x / 10000

toGPSTime :: GpsObservationHeader -> ObsGPSTime
toGPSTime hdr = ObsGPSTime
  { _obsGPSTime_tow = hdr ^. gpsObservationHeader_tow
  , _obsGPSTime_wn  = 1874
  }

fromGpsObservationHeader :: GpsObservationHeader -> ObservationHeader
fromGpsObservationHeader hdr = ObservationHeader
  { _observationHeader_t     = toGPSTime hdr
  , _observationHeader_n_obs = fromIntegral $ hdr ^. gpsObservationHeader_n
  }

toP :: GpsL1Observation -> GpsL1ExtObservation -> Word32
toP l1 l1e = round $ p * 100 where
  p = 0.02 * fromIntegral (l1 ^. gpsL1Observation_pseudorange) +
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

toSid :: Word8 -> SBPGnssSignal
toSid sat = SBPGnssSignal
  { _sBPGnssSignal_sat           = fromIntegral $ sat - 1
  , _sBPGnssSignal_band          = 0
  , _sBPGnssSignal_constellation = 0
  }

fromObservation1002 :: Observation1002 -> PackedObsContent
fromObservation1002 obs = PackedObsContent
  { _packedObsContent_P    = toP l1 l1e
  , _packedObsContent_L    = toL l1 l1e
  , _packedObsContent_cn0  = toCn0 l1e
  , _packedObsContent_lock = toLock l1
  , _packedObsContent_sid  = toSid sat
  } where
    sat = obs ^. observation1002_sat
    l1  = obs ^. observation1002_l1
    l1e = obs ^. observation1002_l1e

fromObservation1004 :: Observation1004 -> PackedObsContent
fromObservation1004 obs = PackedObsContent
  { _packedObsContent_P    = toP l1 l1e
  , _packedObsContent_L    = toL l1 l1e
  , _packedObsContent_cn0  = toCn0 l1e
  , _packedObsContent_lock = toLock l1
  , _packedObsContent_sid  = toSid sat
  }  where
    sat = obs ^. observation1004_sat
    l1  = obs ^. observation1004_l1
    l1e = obs ^. observation1004_l1e

fromMsg1005 :: Msg1005 -> MsgBasePosEcef
fromMsg1005 msg = MsgBasePosEcef
  { _msgBasePosEcef_x = fromEcefVal $ msg ^. msg1005_reference ^. antennaReference_ecef_x
  , _msgBasePosEcef_y = fromEcefVal $ msg ^. msg1005_reference ^. antennaReference_ecef_y
  , _msgBasePosEcef_z = fromEcefVal $ msg ^. msg1005_reference ^. antennaReference_ecef_z
  }

fromMsg1006 :: Msg1006 -> MsgBasePosEcef
fromMsg1006 msg = MsgBasePosEcef
  { _msgBasePosEcef_x = fromEcefVal $ msg ^. msg1006_reference ^. antennaReference_ecef_x
  , _msgBasePosEcef_y = fromEcefVal $ msg ^. msg1006_reference ^. antennaReference_ecef_y
  , _msgBasePosEcef_z = fromEcefVal $ msg ^. msg1006_reference ^. antennaReference_ecef_z
  }

fromMsg1002 :: Msg1002 -> MsgObs
fromMsg1002 msg = MsgObs
  { _msgObs_header = fromGpsObservationHeader $ msg ^. msg1002_header
  , _msgObs_obs    = map fromObservation1002 $ msg ^. msg1002_observations
  }

fromMsg1004 :: Msg1004 -> MsgObs
fromMsg1004 msg = MsgObs
  { _msgObs_header = fromGpsObservationHeader $ msg ^. msg1004_header
  , _msgObs_obs    = map fromObservation1004 $ msg ^. msg1004_observations
  }

convert :: RTCM3Msg -> Maybe SBPMsg
convert = \case
  (RTCM3Msg1002 msg _rtcm3) -> Just $
    SBPMsgObs msg' $ toSBP msg' defaultSender where
      msg' = fromMsg1002 msg
  (RTCM3Msg1004 msg _rtcm3) -> Just $
    SBPMsgObs msg' $ toSBP msg' defaultSender where
      msg' = fromMsg1004 msg
  (RTCM3Msg1005 msg _rtcm3) -> Just $
    SBPMsgBasePosEcef msg' $ toSBP msg' defaultSender where
      msg' = fromMsg1005 msg
  (RTCM3Msg1006 msg _rtcm3) -> Just $
    SBPMsgBasePosEcef msg' $ toSBP msg' defaultSender where
      msg' = fromMsg1006 msg
  _rtcm3Msg -> Nothing


