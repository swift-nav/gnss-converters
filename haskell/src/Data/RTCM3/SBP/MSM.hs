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
import Data.List.Extra      hiding (null)
import Data.RTCM3
import Data.RTCM3.SBP.Types
import Data.Vector          hiding (length)
import Data.Word
import SwiftNav.SBP

class FromObservations a where
  gpsTime           :: MonadStore e m => a -> m (GpsTime, GpsTime)
  packedObsContents :: a -> [PackedObsContent]
  sender            :: a -> Word16
  multiple          :: a -> Bool

instance FromObservations Msg1074 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1075 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1076 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1077 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1084 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1085 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1086 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1087 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1094 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1095 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1096 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1097 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1104 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1105 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1106 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1107 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1114 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1115 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1116 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1117 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1124 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1125 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1126 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

instance FromObservations Msg1127 where
  gpsTime           = undefined
  packedObsContents = undefined
  sender            = undefined
  multiple          = undefined

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
