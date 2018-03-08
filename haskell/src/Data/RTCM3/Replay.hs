{-# LANGUAGE LambdaCase        #-}
{-# LANGUAGE NoImplicitPrelude #-}

module Data.RTCM3.Replay
  ( replayer
  ) where

import BasicPrelude
import Control.Concurrent          hiding (yield)
import Control.Lens
import Data.Conduit
import Data.RTCM3
import Data.Time
import Data.Time.Calendar.WeekDate

-- | Produce GPS time of week from a GPS UTC time.
--
toTow :: UTCTime -> Word32
toTow t = floor since
  where
    (y, w, _d) = toWeekDate (utctDay t)
    begin      = addDays (-1) $ fromWeekDate y w 1
    since      = 1000 * diffUTCTime t (UTCTime begin 0)

-- | Produce current GPS time of week.
--
currentTow :: MonadIO m => m Word32
currentTow =
  liftIO $ toTow . addUTCTime (fromIntegral gpsLeapSeconds) <$> getCurrentTime
  where
    gpsLeapSeconds = 18 :: Int

-- | Delay between two time of week values - differences over 15 seconds are ignored.
--
delayTow :: MonadIO m => Word32 -> Word32 -> m ()
delayTow tow tow' =
  when (tow' > tow) $ do
    let diff = tow' - tow
    unless (diff > diffMilliseconds) $
      liftIO $ threadDelay $ fromIntegral $ diff * 1000
  where
    diffMilliseconds = 15 * 1000

-- | Replay observations.
--
replay :: MonadIO m => Word32 -> RTCM3Msg -> ConduitM i RTCM3Msg m Word32
replay tow = \case
  (RTCM3Msg1002 m _rtcm3) -> do
    let tow' = m ^. msg1002_header . gpsObservationHeader_tow
    delayTow tow tow'
    tow'' <- currentTow
    let n = set (msg1002_header . gpsObservationHeader_tow) tow'' m
    yield (RTCM3Msg1002 n (toRTCM3 n))
    pure tow'
  (RTCM3Msg1004 m _rtcm3) -> do
    let tow' = m ^. msg1004_header . gpsObservationHeader_tow
    delayTow tow tow'
    tow'' <- currentTow
    let n = set (msg1004_header . gpsObservationHeader_tow) tow'' m
    yield (RTCM3Msg1004 n (toRTCM3 n))
    pure tow'
  rtcm3Msg -> do
    yield rtcm3Msg
    pure tow

replayer :: MonadIO m => Conduit RTCM3Msg m RTCM3Msg
replayer = loop maxBound
  where
    loop tow =
      await >>=  maybe (pure ()) ((>>= loop) . replay tow)
