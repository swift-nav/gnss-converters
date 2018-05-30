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
import Data.List.Extra      hiding (filter, null, zip)
import Data.RTCM3
import Data.RTCM3.SBP.Time
import Data.RTCM3.SBP.Types
import Data.Vector          hiding (filter, length, zip)
import Data.Word
import SwiftNav.SBP

-- | Derive sender from station.
--
toSender :: Word16 -> Word16
toSender station = station .|. 61440

masks :: Bits a => Int -> a -> [b] -> [b]
masks n a bs = fst <$> filter snd (zip bs $ testBit a <$> [n,n-1..0])

mask :: FiniteBits a => a -> [Word8]
mask n = masks (finiteBitSize n-1) n [(1::Word8)..]

_cellsMask :: Word64 -> Word32 -> Word64 -> [((Int, Word8), (Int, Word8))]
_cellsMask sats sigs cells = masks (popCount sats * popCount sigs) cells $ do
  sat <- zip [0..] $ mask sats
  sig <- zip [0..] $ mask sigs
  pure (sat, sig)

toCells :: MsmHeader -> [((Int, Word8), (Int, Word8))]
toCells hdr =
  masks (popCount (hdr ^. msmHeader_satelliteMask) * popCount (hdr ^. msmHeader_signalMask)) (hdr ^. msmHeader_cellMask) $ do
    sat <- zip [0..] $ mask (hdr ^. msmHeader_satelliteMask)
    sig <- zip [0..] $ mask (hdr ^. msmHeader_signalMask)
    pure (sat, sig)

toGpsSat :: Word8 -> Maybe Word8
toGpsSat sat
  | sat >= 64 = Nothing
  | otherwise = Just sat

toGpsCode :: Word8 -> Maybe Word8
toGpsCode sig
  | sig == 2  = Just 0
  | sig == 3  = Just 5
  | sig == 9  = Just 6
  | sig == 15 = Just 1
  | sig == 16 = Just 7
  | sig == 17 = Just 8
  | sig == 22 = Just 9
  | sig == 23 = Just 10
  | sig == 24 = Just 11
  | otherwise = Nothing

toGpsSignal :: Word8 -> Word8 -> Maybe GnssSignal
toGpsSignal sat sig = do
  sat' <- toGpsSat sat
  code <- toGpsCode sig
  Just $ GnssSignal sat' code

toPackedObsContent :: MsmHeader -> Msm46SatelliteData -> Msm4SignalData -> (Int, Word8) -> (Int, Word8) -> Maybe PackedObsContent
toPackedObsContent hdr satData sigData (satIndex, sat) (sigIndex, sig) = do
  sid <- toGpsSignal sat sig
  Just PackedObsContent
    { _packedObsContent_P     = undefined
    , _packedObsContent_L     = undefined
    , _packedObsContent_D     = undefined
    , _packedObsContent_cn0   = (sigData ^. msm4SignalData_cnrs) !! sigIndex
    , _packedObsContent_lock  = (sigData ^. msm4SignalData_lockTimes) !! sigIndex
    , _packedObsContent_sid   = sid
    , _packedObsContent_flags = undefined
    }

class FromObservations a where
  gpsTime           :: MonadStore e m => a -> m (GpsTime, GpsTime)
  packedObsContents :: a -> [PackedObsContent]
  sender            :: a -> Word16
  multiple          :: a -> Bool

instance FromObservations Msg1074 where
  gpsTime m           = toGpsTime (m ^. msg1074_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1074_header . msmHeader_epoch)
  packedObsContents m = catMaybes $ uncurry (toPackedObsContent (m ^. msg1074_header) (m ^. msg1074_satelliteData) (m ^. msg1074_signalData)) <$> toCells (m ^. msg1074_header)
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
  gpsTime m         = toGpsTime (m ^. msg1077_header . msmHeader_station) $ gpsRolloverGpsTime (m ^. msg1077_header . msmHeader_epoch)
  packedObsContents = const mempty
  sender            = toSender . view (msg1077_header . msmHeader_station)
  multiple          = view (msg1077_header . msmHeader_multiple)

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
