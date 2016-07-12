{-# LANGUAGE LambdaCase #-}

-- |
-- Module:      Data.RTCM3.SBP
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Mark Fine <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Conversions.

module Data.RTCM3.SBP where

import BasicPrelude
import Control.Lens
import Data.Bits
import qualified Data.HashMap.Strict as H
import Data.List.Extra hiding (concat, map)
import Data.Time
import Data.Word
import Data.RTCM3
import SwiftNav.SBP


--------------------------------------------------------------------------------
-- SBP, GNSS, RTCM constant definitions


-- | Two centimeters
--
-- SBP pseudoranges are in units of 2cm, and there are 50 of those in a meter.
twoCM :: Double
twoCM = 0.02

sbpCMinM :: Double
sbpCMinM = 50

-- | Speed of light (meters/msec)
lightSpeedMMSEC :: Double
lightSpeedMMSEC = 299792.458

-- | Speed of light (meters/sec)
lightSpeedMS :: Double
lightSpeedMS = 299792458.0

-- | L1 GPS center frequency (Hz)
l1frequency :: Double
l1frequency = 1.57542e9

-- | L2 GPS center frequency (Hz)
l2frequency :: Double
l2frequency = 1.22760e9

-- | We only support PRNS 1 - 32
maxSats :: Word8
maxSats = 32

-- | Q32.8 carrier phase representation width
q32Width :: Double
q32Width = 256

-- | RTCM phase range resolution.
--
-- See DF018, pg. 3-19 of the RTCM3 spec
phaseRangeRes :: Double
phaseRangeRes = 0.0005

-- | SBP L1 GNSS signal value
--
-- See: https://github.com/swift-nav/libswiftnav/blob/master/include/libswiftnav/signal.h#L65
l1CSidCode :: Word8
l1CSidCode = 0

-- | SBP L2CM GNSS signal value
--
-- See: https://github.com/swift-nav/libswiftnav/blob/master/include/libswiftnav/signal.h#L65
l2CMSidCode :: Word8
l2CMSidCode = 1

-- | SBP L1P and L2P GNSS signal values
--
-- Note that libswiftnav currently does not support L1P or L2P observations,
-- just L1C and L2C. This is a stop gap definition so that we can properly
-- serialize out SBP observations with L1P and L2P observations from external
-- receivers.
l1PSidCode :: Word8
l1PSidCode = 5

l2PSidCode :: Word8
l2PSidCode = 6

-- | L2C code indicator value
--
-- See DF016, pg. 3-17 of the RTCM3 spec
codeIndicator_L2C :: Word8
codeIndicator_L2C = 0

-- | L2P code indicator value
--
-- See DF016, pg. 3-17 of the RTCM3 spec
codeIndicator_L2PD :: Word8
codeIndicator_L2PD = 1

-- | Map L2 codes to SBP GnssSignal codes
--
l2codeToSBPSignalCode :: H.HashMap Word8 Word8
l2codeToSBPSignalCode = H.fromList [(0, l2CMSidCode), (1, l2PSidCode)]

-- | Maximum number of packed observations to allow in a single SBP message.
--
maxObsPerMessage :: Int
maxObsPerMessage = floor $ (maxPayloadSize - headerSize) / packedObsSize
  where
    maxPayloadSize = 255
    headerSize     = 7
    packedObsSize  = 16.0


--------------------------------------------------------------------------------
-- GNSS RTCM observation reconstruction utilities


fromEcefVal :: Int64 -> Double
fromEcefVal x = fromIntegral x / 10000

toGPSTime :: MonadIO m => GpsObservationHeader -> m ObsGPSTime
toGPSTime hdr = do
  today <- utctDay <$> liftIO getCurrentTime
  return ObsGPSTime
    { _obsGPSTime_tow = hdr ^. gpsObservationHeader_tow
    , _obsGPSTime_wn  = fromIntegral $ div (diffDays today (fromGregorian 1980 1 6)) 7
    }

-- | Construct metric pseudorange (meters!) from L1 RTCM observation.
--
-- See DF011, pg. 3-16 of the RTCM3 spec
metricPseudorange :: GpsL1Observation -> GpsL1ExtObservation -> Double
metricPseudorange l1 l1e =
  twoCM * fromIntegral (l1 ^. gpsL1Observation_pseudorange) +
  lightSpeedMMSEC * fromIntegral (l1e ^. gpsL1ExtObservation_ambiguity)

-- | Construct L1 SBP pseudorange for L1 RTCM observation
--
-- See DF011, pg. 3-16 of the RTCM3 spec
toP_L1 :: GpsL1Observation -> GpsL1ExtObservation -> Word32
toP_L1 l1 l1e = round $ sbpCMinM * metricPseudorange l1 l1e

-- | Construct L2 SBP pseudorange from L1/L2 RTCM observations
--
-- See DF017, pg. 3-18 of the RTCM3 spec
toP_L2 :: GpsL1Observation -> GpsL1ExtObservation -> GpsL2Observation -> Word32
toP_L2 l1 l1e l2 = round $ p * sbpCMinM where
  p = metricPseudorange l1 l1e +
      twoCM * fromIntegral (l2 ^. gpsL2Observation_pseudorangeDifference)

-- | Construct SBP L1 GPS carrier phase from L1 RTCM observation
--
-- See DF012, pg. 3-16 of the RTCM3 spec
toL_L1 :: GpsL1Observation -> GpsL1ExtObservation -> CarrierPhase
toL_L1 l1 l1e = CarrierPhase
  { _carrierPhase_i = fromIntegral li
  , _carrierPhase_f = fromIntegral lf
  } where
    p = metricPseudorange l1 l1e
    -- Convert to SBP carrier phase representation per
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L39
    lm :: Double
    lm = p + phaseRangeRes * fromIntegral (l1 ^. gpsL1Observation_carrierMinusCode)
    l = lm / (lightSpeedMS / l1frequency)
    li :: Int32
    li = floor (l)
    lf :: Word8
    lf = truncate ((l - fromIntegral li) * q32Width)

-- | Construct SBP L2 GPS carrier phase from L2 RTCM observation
--
-- See DF018, pg. 3-18 of the RTCM3 spec
toL_L2 :: GpsL1Observation
       -> GpsL1ExtObservation
       -> GpsL2Observation
       -> GpsL2ExtObservation
       -> CarrierPhase
toL_L2 l1 l1e l2 l2e = CarrierPhase
  { _carrierPhase_i = fromIntegral li
  , _carrierPhase_f = fromIntegral lf
  } where
    p = metricPseudorange l1 l1e
    -- Convert to SBP carrier phase representation per
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L39
    lm :: Double
    lm = p + phaseRangeRes * fromIntegral (l2 ^. gpsL2Observation_carrierMinusCode)
    l = lm / (lightSpeedMS / l2frequency)
    li :: Int32
    li = floor (l)
    lf :: Word8
    lf = truncate ((l - fromIntegral li) * q32Width)

toCn0_L1 :: GpsL1ExtObservation -> Word8
toCn0_L1 = (^. gpsL1ExtObservation_cnr)

toCn0_L2 :: GpsL2ExtObservation -> Word8
toCn0_L2 = (^. gpsL2ExtObservation_cnr)

toLock_L1 :: GpsL1Observation -> Word16
toLock_L1 _l1 = 0

toLock_L2 :: GpsL2Observation -> Word16
toLock_L2 _l2 = 0

-- | Construct sequenced SBP observation header
--
fromGpsObservationHeader :: MonadIO m
                         => Word8                -- ^ Total messages
                         -> Word8                -- ^ Message in sequence
                         -> GpsObservationHeader -- ^ RTCM observation header
                         -> m ObservationHeader
fromGpsObservationHeader totalMsgs n hdr = do
  t <- toGPSTime hdr
  return ObservationHeader
    { _observationHeader_t     = t
    -- First nibble is the size of the sequence (n), second nibble is the
    -- zero-indexed counter (ith packet of n). See observation header packing
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L63
    , _observationHeader_n_obs = totalMsgs `shiftL` 4 .|. n
    }

-- | Construct an L1 SBP PackedObsContent an RTCM satellite vehicle observation
--
fromL1SatelliteObservation :: Word8                -- ^ Satellite PRN
                           -> GpsL1Observation
                           -> GpsL1ExtObservation
                           -> PackedObsContent
fromL1SatelliteObservation sat l1 l1e =
  -- Checks GPS L1 code indicator for RTCM message 1002.
  -- See DF016, pg. 3-17 of the RTCM3 spec.
  if l1 ^. gpsL1Observation_code then
    PackedObsContent
      { _packedObsContent_P    = toP_L1 l1 l1e
      , _packedObsContent_L    = toL_L1 l1 l1e
      , _packedObsContent_cn0  = toCn0_L1 l1e
      , _packedObsContent_lock = toLock_L1 l1
      , _packedObsContent_sid  = GnssSignal
        { _gnssSignal_sat      = fromIntegral $ sat - 1
        , _gnssSignal_code     = l1PSidCode
        , _gnssSignal_reserved = 0
        }
      }
  else
    PackedObsContent
      { _packedObsContent_P    = toP_L1 l1 l1e
      , _packedObsContent_L    = toL_L1 l1 l1e
      , _packedObsContent_cn0  = toCn0_L1 l1e
      , _packedObsContent_lock = toLock_L1 l1
      , _packedObsContent_sid  = GnssSignal
        { _gnssSignal_sat      = fromIntegral $ sat - 1
        , _gnssSignal_code     = l1CSidCode
        , _gnssSignal_reserved = 0
        }
      }

-- | Construct SBP GPS observation message (possibly chunked).
--
chunkToMsgObs :: MonadIO m
              => Msg1004            -- ^ RTCM 1004 observation message
              -> Word8              -- ^ Total messages
              -> Word8              -- ^ Message in sequence
              -> [PackedObsContent]
              -> m MsgObs
chunkToMsgObs m totalMsgs n packed = do
  header <- fromGpsObservationHeader totalMsgs n $ m ^. msg1004_header
  return MsgObs
    { _msgObs_header = header
    , _msgObs_obs    = packed
    }

-- | Sender Id is Station Id with high byte or'd in
--
toSender :: Word16 -> Word16
toSender = (.|. 0xf00)


--------------------------------------------------------------------------------
-- RTCM to SBP conversion utilities: RTCM Msgs. 1002 (L1 RTK), 1004 (L1+L2 RTK),
-- 1005 (antenna position), 1006 (antenna position).


-- | Construct an L1 SBP PackedObsContent from an RTCM Msg 1002.
--
fromObservation1002 :: Observation1002 -> Maybe PackedObsContent
fromObservation1002 obs =
  -- Only lower set of PRN numbers (1-32) are supported
  if sat > maxSats then Nothing else Just (fromL1SatelliteObservation sat l1 l1e)
    where
      sat = obs ^. observation1002_sat
      l1  = obs ^. observation1002_l1
      l1e = obs ^. observation1002_l1e

-- | Convert an RTCM L1 1002 observation into an SBP MsgObs.
--
fromMsg1002 :: MonadIO m => Msg1002 -> m MsgObs
fromMsg1002 m = do
  header <- fromGpsObservationHeader 1 0 (m ^. msg1002_header)
  return MsgObs
    { _msgObs_header = header
    , _msgObs_obs    = mapMaybe fromObservation1002 $ m ^. msg1002_observations
    }

-- | Construct an L1/L2 SBP PackedObsContent from an RTCM Msg 1004.
--
fromObservation1004 :: Observation1004 -> Maybe [PackedObsContent]
fromObservation1004 obs =
  -- Only lower set of PRN numbers (1-32) are supported
  if sat > maxSats then Nothing else mapM id [obs1, obs2]
    where
      sat = obs ^. observation1004_sat
      l1  = obs ^. observation1004_l1
      l1e = obs ^. observation1004_l1e
      -- Checks GPS L1 code indicator for RTCM message 1004
      -- See DF016, pg. 3-17 of the RTCM3 spec.
      obs1 = Just (fromL1SatelliteObservation sat l1 l1e)
      l2   = obs ^. observation1004_l2
      l2e  = obs ^. observation1004_l2e
      code = l2 ^. gpsL2Observation_code
      -- Checks GPS L2 code indicator.
      -- See DF016, pg. 3-17 of the RTCM3 spec.
      obs2 = if H.member code l2codeToSBPSignalCode then
               Just PackedObsContent
                 { _packedObsContent_P    = toP_L2 l1 l1e l2
                 , _packedObsContent_L    = toL_L2 l1 l1e l2 l2e
                 , _packedObsContent_cn0  = toCn0_L2 l2e
                 , _packedObsContent_lock = toLock_L2 l2
                 , _packedObsContent_sid  = GnssSignal
                   { _gnssSignal_sat      = fromIntegral $ sat - 1
                   , _gnssSignal_code     = l2codeToSBPSignalCode H.! code
                   , _gnssSignal_reserved = 0
                   }
                 }
             else
               Nothing

-- | Convert an RTCM L1+L2 1004 observation into multiple SBP MsgObs.
--
-- This chunking takes places because the number of observations in a given 1004
-- may very well exceed the maximum SBP supported payload size of 255 bytes.
fromMsg1004 :: MonadIO m => Msg1004 -> m [MsgObs]
fromMsg1004 m = do
  let obs       = concat $ mapMaybe fromObservation1004 $ m ^. msg1004_observations
      chunks    = zip [0..] $ chunksOf maxObsPerMessage obs
      totalMsgs = fromIntegral $ length chunks
  mapM (\(n, packed) -> chunkToMsgObs m totalMsgs n packed) chunks

-- | Convert an RTCM 1005 antenna reference position message into an SBP
-- MsgBasePosEcef.
fromMsg1005 :: MonadIO m => Msg1005 -> m MsgBasePosEcef
fromMsg1005 m =
  return MsgBasePosEcef
    { _msgBasePosEcef_x = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_x
    , _msgBasePosEcef_y = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_y
    , _msgBasePosEcef_z = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_z
    }

-- | Convert an RTCM 1006 antenna reference position message into an SBP
-- MsgBasePosEcef.
fromMsg1006 :: MonadIO m => Msg1006 -> m MsgBasePosEcef
fromMsg1006 m =
  return MsgBasePosEcef
    { _msgBasePosEcef_x = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_x
    , _msgBasePosEcef_y = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_y
    , _msgBasePosEcef_z = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_z
    }

-- | Convert an RTCM message into possibly multiple SBP messages.
--
convert :: MonadIO m => RTCM3Msg -> m (Maybe [SBPMsg])
convert = \case
  (RTCM3Msg1002 m _rtcm3) -> do
    let sender = m ^. msg1002_header ^. gpsObservationHeader_station
    m' <- fromMsg1002 m
    return $ Just [SBPMsgObs m' $ toSBP m' $ toSender sender]
  (RTCM3Msg1004 m _rtcm3) -> do
    let sender = m ^. msg1004_header ^. gpsObservationHeader_station
    m' <- fromMsg1004 m
    return $ Just (fmap (\x -> SBPMsgObs x $ toSBP x $ toSender sender) m')
  (RTCM3Msg1005 m _rtcm3) -> do
    let sender =  m ^. msg1005_reference ^. antennaReference_station
    m' <- fromMsg1005 m
    return $ Just [SBPMsgBasePosEcef m' $ toSBP m' $ toSender sender]
  (RTCM3Msg1006 m _rtcm3) -> do
    let sender = m ^. msg1006_reference ^. antennaReference_station
    m' <- fromMsg1006 m
    return $ Just [SBPMsgBasePosEcef m' $ toSBP m' $ toSender sender]
  _rtcm3Msg -> return Nothing
