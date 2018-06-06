{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.MSM
-- Copyright:   Copyright (C) 2017 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 MSM to SBP Observations.

module Data.RTCM3.SBP.MSM
  ( converter
  ) where

import BasicPrelude         hiding (null)
import Control.Lens
import Data.Bits
import Data.Conduit
import Data.IORef
import Data.List.Extra      hiding (filter, map, null, zip)
import Data.RTCM3
import Data.RTCM3.SBP.Time
import Data.RTCM3.SBP.Types
import Data.Vector          hiding (filter, length, map, zip)
import Data.Word
import SwiftNav.SBP

-- | Derive sender from station.
--
toSender :: Word16 -> Word16
toSender station = station .|. 61568

masks :: Bits a => Int -> a -> [b] -> [b]
masks n a bs = fst <$> filter snd (zip bs $ testBit a <$> [n-1,n-2..0])

mask :: FiniteBits a => a -> [Word8]
mask n = masks (finiteBitSize n) n [1..]

toCells :: MsmHeader -> [((Int, Word8), (Int, Word8))]
toCells hdr =
  map (\(i, (sat, sig)) -> (sat, (i, sig))) $
    zip [0..] $ masks (popCount (hdr ^. msmHeader_satelliteMask) * popCount (hdr ^. msmHeader_signalMask)) (hdr ^. msmHeader_cellMask) $ do
      sat <- zip [0..] $ mask (hdr ^. msmHeader_satelliteMask)
      sig <- mask (hdr ^. msmHeader_signalMask)
      pure (sat, sig)

-- | Max GPS satellite number.
--
gpsMaxSat :: Word8
gpsMaxSat = 32

toGpsSat :: Word8 -> Maybe Word8
toGpsSat sat
  | sat < 1 = Nothing
  | sat > gpsMaxSat = Nothing
  | otherwise = Just sat

toGpsCode :: Word8 -> Maybe Word8
toGpsCode sig
  | sig == 2  = Just 0
  | sig == 3  = Just 5
  | sig == 9  = Just 6
  | sig == 10 = Just 6
  | sig == 15 = Just 1
  | sig == 16 = Just 7
  | sig == 17 = Just 1
  -- | sig == 22 = Just 9
  -- | sig == 23 = Just 10
  -- | sig == 24 = Just 11
  | otherwise = Nothing

toGpsSignal :: Word8 -> Word8 -> Maybe GnssSignal
toGpsSignal sat sig = do
  sat' <- toGpsSat sat
  code <- toGpsCode sig
  Just $ GnssSignal sat' code

-- | Convert to SBP pseudorange.
--
toP :: Double -> Word32
toP p = round $ 50 * p

-- | Convert to SBP carrier-phase measurement.
--
toL :: Double -> CarrierPhase
toL l = if f /= 256 then CarrierPhase i (fromIntegral f) else CarrierPhase (i + 1) 0
  where
    i = floor l
    f = (round $ (l - fromIntegral i) * 256) :: Word16

-- | GPS pseudorange unit.
--
gpsPseudorange :: Double
gpsPseudorange = 299792.458

toD :: Doppler
toD = Doppler 0 0

toGpsFrequency :: Word8 -> Maybe Double
toGpsFrequency sig
  | sig == 2  = Just 1.57542e9
  | sig == 3  = Just 1.57542e9
  | sig == 9  = Just 1.22760e9
  | sig == 10 = Just 1.22760e9
  | sig == 15 = Just 1.22760e9
  | sig == 16 = Just 1.22760e9
  | sig == 17 = Just 1.22760e9
  | sig == 22 = Just (115 * 10.23e6)
  | sig == 23 = Just (115 * 10.23e6)
  | sig == 24 = Just (115 * 10.23e6)
  | otherwise = Nothing

-- | Convert from RTCMv3 lock indicator.
--
lock :: Word16 -> Word32
lock t
  |   t <  64 = 1000 * fromIntegral t
  |   t <  96 = 1000 * fromIntegral t *       2 -        64
  |   t < 128 = 1000 * fromIntegral t *       4 -       256
  |   t < 160 = 1000 * fromIntegral t *       8 -       768
  |   t < 192 = 1000 * fromIntegral t *      16 -      2048
  |   t < 224 = 1000 * fromIntegral t *      32 -      5120
  |   t < 256 = 1000 * fromIntegral t *      64 -     12288
  |   t < 288 = 1000 * fromIntegral t *     128 -     28672
  |   t < 320 = 1000 * fromIntegral t *     256 -     65536
  |   t < 352 = 1000 * fromIntegral t *     512 -    147456
  |   t < 384 = 1000 * fromIntegral t *    1024 -    327680
  |   t < 416 = 1000 * fromIntegral t *    2048 -    720896
  |   t < 448 = 1000 * fromIntegral t *    4096 -   1572864
  |   t < 480 = 1000 * fromIntegral t *    8192 -   3407872
  |   t < 512 = 1000 * fromIntegral t *   16384 -   7340032
  |   t < 544 = 1000 * fromIntegral t *   32768 -  15728640
  |   t < 576 = 1000 * fromIntegral t *   65536 -  33554432
  |   t < 608 = 1000 * fromIntegral t *  131072 -  71303168
  |   t < 640 = 1000 * fromIntegral t *  262144 - 150994944
  |   t < 672 = 1000 * fromIntegral t *  524288 - 318767104
  |   t < 704 = 1000 * fromIntegral t * 1048576 - 671088640
  | otherwise = 1000 * 67108864

-- | Convert to SBP lock time.
--
toLock :: Word32 -> Word8
toLock t
  | t <     32 = 0
  | t <     64 = 1
  | t <    128 = 2
  | t <    256 = 3
  | t <    512 = 4
  | t <   1024 = 5
  | t <   2048 = 6
  | t <   4096 = 7
  | t <   8192 = 8
  | t <  16384 = 9
  | t <  32768 = 10
  | t <  65536 = 11
  | t < 131072 = 12
  | t < 262144 = 13
  | t < 524288 = 14
  | otherwise  = 15

toPackedObsContent1074 :: Msm46SatelliteData -> Msm4SignalData -> (Int, Word8) -> (Int, Word8) -> Maybe PackedObsContent
toPackedObsContent1074 satData sigData (satIndex, sat) (sigIndex, sig)
  | no        = Nothing
  | otherwise = do
      sid  <- toGpsSignal sat sig
      freq <- toGpsFrequency sig
      Just PackedObsContent
        { _packedObsContent_P     = toP (roughPseudorange + finePseudorange)
        , _packedObsContent_L     = toL ((roughPseudorange + finePhaserange) * (freq / 299792458.0))
        , _packedObsContent_D     = toD
        , _packedObsContent_cn0   = ((sigData ^. msm4SignalData_cnrs) !! sigIndex) * 4
        , _packedObsContent_lock  = (sigData ^. msm4SignalData_lockTimes) !! sigIndex
        , _packedObsContent_sid   = sid
        , _packedObsContent_flags = pseudorangeValid .|. phaseValid .|. halfCycleResolved
        }
  where
    no                = ((satData ^. msm46SatelliteData_ranges) !! satIndex) == 255 ||
                        ((sigData ^. msm4SignalData_pseudoranges) !! sigIndex) == 16384 ||
                        ((sigData ^. msm4SignalData_phaseranges) !! sigIndex) == 2097152
    pseudorangeValid  = 1
    phaseValid        = 2
    halfCycleResolved = bool 4 0 $ (sigData ^. msm4SignalData_halfCycles) !! sigIndex
    roughPseudorange  = fromIntegral ((satData ^. msm46SatelliteData_ranges) !! satIndex) * gpsPseudorange +
                        fromIntegral ((satData ^. msm46SatelliteData_rangesModulo) !! satIndex) * gpsPseudorange / 1024
    finePseudorange   = fromIntegral ((sigData ^. msm4SignalData_pseudoranges) !! sigIndex) * gpsPseudorange * (2 ** (-24))
    finePhaserange    = fromIntegral ((sigData ^. msm4SignalData_phaseranges) !! sigIndex) * gpsPseudorange * (2 ** (-29))

toPackedObsContent1077 :: Msm57SatelliteData -> Msm7SignalData -> (Int, Word8) -> (Int, Word8) -> Maybe PackedObsContent
toPackedObsContent1077 satData sigData (satIndex, sat) (sigIndex, sig)
  | no        = Nothing
  | otherwise = do
      sid  <- toGpsSignal sat sig
      freq <- toGpsFrequency sig
      Just PackedObsContent
        { _packedObsContent_P     = toP (roughPseudorange + finePseudorange)
        , _packedObsContent_L     = toL ((roughPseudorange + finePhaserange) * (freq / 299792458.0))
        , _packedObsContent_D     = toD
        , _packedObsContent_cn0   = round (fromIntegral ((sigData ^. msm7SignalData_cnrs) !! sigIndex) * ((2 :: Double) ** (-4)) * 4)
        , _packedObsContent_lock  = toLock $ lock ((sigData ^. msm7SignalData_lockTimes) !! sigIndex)
        , _packedObsContent_sid   = sid
        , _packedObsContent_flags = pseudorangeValid .|. phaseValid .|. halfCycleResolved
        }
  where
    no                = ((satData ^. msm57SatelliteData_ranges) !! satIndex) == 255 ||
                        ((sigData ^. msm7SignalData_pseudoranges) !! sigIndex) == 524288 ||
                        ((sigData ^. msm7SignalData_phaseranges) !! sigIndex) == 8388608
    pseudorangeValid  = 1
    phaseValid        = 2
    halfCycleResolved = bool 4 0 $ (sigData ^. msm7SignalData_halfCycles) !! sigIndex
    roughPseudorange  = fromIntegral ((satData ^. msm57SatelliteData_ranges) !! satIndex) * gpsPseudorange +
                        fromIntegral ((satData ^. msm57SatelliteData_rangesModulo) !! satIndex) * gpsPseudorange / 1024
    finePseudorange   = fromIntegral ((sigData ^. msm7SignalData_pseudoranges) !! sigIndex) * gpsPseudorange * (2 ** (-29))
    finePhaserange    = fromIntegral ((sigData ^. msm7SignalData_phaseranges) !! sigIndex) * gpsPseudorange * (2 ** (-31))


class FromObservations a where
  gpsTime           :: MonadStore e m => a -> m (GpsTime, GpsTime)
  packedObsContents :: a -> [PackedObsContent]
  sender            :: a -> Word16
  multiple          :: a -> Bool

instance FromObservations Msg1074 where
  gpsTime m           = toGpsTime (m ^. msg1074_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1074_header . msmHeader_epoch)
  packedObsContents m = catMaybes $ uncurry (toPackedObsContent1074 (m ^. msg1074_satelliteData) (m ^. msg1074_signalData)) <$> toCells (m ^. msg1074_header)
  sender              = toSender . view (msg1074_header . msmHeader_station)
  multiple            = view (msg1074_header . msmHeader_multiple)

instance FromObservations Msg1075 where
  gpsTime m         = toGpsTime (m ^. msg1075_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1075_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1075_header . msmHeader_station)
  multiple          = view (msg1075_header . msmHeader_multiple)

instance FromObservations Msg1076 where
  gpsTime m         = toGpsTime (m ^. msg1076_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1076_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1076_header . msmHeader_station)
  multiple          = view (msg1076_header . msmHeader_multiple)

instance FromObservations Msg1077 where
  gpsTime m           = toGpsTime (m ^. msg1077_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1077_header . msmHeader_epoch)
  packedObsContents m = catMaybes $ uncurry (toPackedObsContent1077 (m ^. msg1077_satelliteData) (m ^. msg1077_signalData)) <$> toCells (m ^. msg1077_header)
  sender              = toSender . view (msg1077_header . msmHeader_station)
  multiple            = view (msg1077_header . msmHeader_multiple)

instance FromObservations Msg1084 where
  gpsTime m         = toGpsTime (m ^. msg1084_header . msmHeader_station) $ glonassRolloverGpsTime' (m ^. msg1084_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1084_header . msmHeader_station)
  multiple          = view (msg1084_header . msmHeader_multiple)

instance FromObservations Msg1085 where
  gpsTime m         = toGpsTime (m ^. msg1085_header . msmHeader_station) $ glonassRolloverGpsTime' (m ^. msg1085_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1085_header . msmHeader_station)
  multiple          = view (msg1085_header . msmHeader_multiple)

instance FromObservations Msg1086 where
  gpsTime m         = toGpsTime (m ^. msg1086_header . msmHeader_station) $ glonassRolloverGpsTime' (m ^. msg1086_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1086_header . msmHeader_station)
  multiple          = view (msg1086_header . msmHeader_multiple)

instance FromObservations Msg1087 where
  gpsTime m         = toGpsTime (m ^. msg1087_header . msmHeader_station) $ glonassRolloverGpsTime' (m ^. msg1087_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1087_header . msmHeader_station)
  multiple          = view (msg1087_header . msmHeader_multiple)

instance FromObservations Msg1094 where
  gpsTime m         = toGpsTime (m ^. msg1094_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1094_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1094_header . msmHeader_station)
  multiple          = view (msg1094_header . msmHeader_multiple)

instance FromObservations Msg1095 where
  gpsTime m         = toGpsTime (m ^. msg1095_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1095_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1095_header . msmHeader_station)
  multiple          = view (msg1095_header . msmHeader_multiple)

instance FromObservations Msg1096 where
  gpsTime m         = toGpsTime (m ^. msg1096_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1096_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1096_header . msmHeader_station)
  multiple          = view (msg1096_header . msmHeader_multiple)

instance FromObservations Msg1097 where
  gpsTime m         = toGpsTime (m ^. msg1097_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1097_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1097_header . msmHeader_station)
  multiple          = view (msg1097_header . msmHeader_multiple)

instance FromObservations Msg1104 where
  gpsTime m         = toGpsTime (m ^. msg1104_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1104_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1104_header . msmHeader_station)
  multiple          = view (msg1104_header . msmHeader_multiple)

instance FromObservations Msg1105 where
  gpsTime m         = toGpsTime (m ^. msg1105_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1105_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1105_header . msmHeader_station)
  multiple          = view (msg1105_header . msmHeader_multiple)

instance FromObservations Msg1106 where
  gpsTime m         = toGpsTime (m ^. msg1106_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1106_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1106_header . msmHeader_station)
  multiple          = view (msg1106_header . msmHeader_multiple)

instance FromObservations Msg1107 where
  gpsTime m         = toGpsTime (m ^. msg1107_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1107_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1107_header . msmHeader_station)
  multiple          = view (msg1107_header . msmHeader_multiple)

instance FromObservations Msg1114 where
  gpsTime m         = toGpsTime (m ^. msg1114_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1114_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1114_header . msmHeader_station)
  multiple          = view (msg1114_header . msmHeader_multiple)

instance FromObservations Msg1115 where
  gpsTime m         = toGpsTime (m ^. msg1115_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1115_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1115_header . msmHeader_station)
  multiple          = view (msg1115_header . msmHeader_multiple)

instance FromObservations Msg1116 where
  gpsTime m         = toGpsTime (m ^. msg1116_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1116_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1116_header . msmHeader_station)
  multiple          = view (msg1116_header . msmHeader_multiple)

instance FromObservations Msg1117 where
  gpsTime m         = toGpsTime (m ^. msg1117_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1117_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1117_header . msmHeader_station)
  multiple          = view (msg1117_header . msmHeader_multiple)

instance FromObservations Msg1124 where
  gpsTime m         = toGpsTime (m ^. msg1124_header . msmHeader_station) $ beidouRolloverGpsTime (m ^. msg1124_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1124_header . msmHeader_station)
  multiple          = view (msg1124_header . msmHeader_multiple)

instance FromObservations Msg1125 where
  gpsTime m         = toGpsTime (m ^. msg1125_header . msmHeader_station) $ beidouRolloverGpsTime (m ^. msg1125_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1125_header . msmHeader_station)
  multiple          = view (msg1125_header . msmHeader_multiple)

instance FromObservations Msg1126 where
  gpsTime m         = toGpsTime (m ^. msg1126_header . msmHeader_station) $ beidouRolloverGpsTime (m ^. msg1126_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1126_header . msmHeader_station)
  multiple          = view (msg1126_header . msmHeader_multiple)

instance FromObservations Msg1127 where
  gpsTime m         = toGpsTime (m ^. msg1127_header . msmHeader_station) $ beidouRolloverGpsTime (m ^. msg1127_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1127_header . msmHeader_station)
  multiple          = view (msg1127_header . msmHeader_multiple)

-- | Convert RTCMv3 observation(s) to SBP observations in chunks.
--
toMsgObs :: Applicative f => GpsTime -> [PackedObsContent] -> Word16 -> f [SBPMsg]
toMsgObs t obs s = do
  let chunks = chunksOf maxObs obs
  ifor chunks $ \i obs' -> do
    let n = length chunks `shiftL` 4 .|. i
        m = MsgObs (ObservationHeader t (fromIntegral n)) obs'
    pure $ SBPMsgObs m $ toSBP m s
  where
    maxObs  = (maxSize - hdrSize) `div` obsSize
    maxSize = 255
    hdrSize = 11
    obsSize = 17

-- | Convert RTCMv3 observation message to SBP observations message(s).
--
converter :: (MonadStore e m, FromObservations a) => a -> Conduit i m [SBPMsg]
converter m = do
  (t, t')      <- gpsTime m
  observations <- view storeObservations
  obs          <- liftIO $ readIORef observations
  when (t' /= t) $
    unless (null obs) $ do
      liftIO $ writeIORef observations mempty
      ms <- toMsgObs t (toList obs) $ sender m
      yield ms
  let obs' = fromList $ packedObsContents m
  if multiple m then liftIO $ modifyIORef' observations (obs' <>) else do
    obs'' <- liftIO $ readIORef observations
    unless (null obs'') $
      liftIO $ writeIORef observations mempty
    let obs''' = obs' <> obs''
    unless (null obs''') $ do
      ms <- toMsgObs t' (toList obs''') $ sender m
      yield ms
