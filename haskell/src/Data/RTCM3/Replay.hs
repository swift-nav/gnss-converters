{-# LANGUAGE LambdaCase        #-}
{-# LANGUAGE NoImplicitPrelude #-}

module Data.RTCM3.Replay
  ( replay
  ) where

import BasicPrelude
import Control.Concurrent          hiding (yield)
import Control.Lens
import Data.Conduit
import Data.Conduit.List
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

-- | Update time of week on replayed messages to be current.
--
postDelay :: MonadIO m => Conduit RTCM3Msg m RTCM3Msg
postDelay = awaitForever $ \case
  (RTCM3Msg1002 m _rtcm3) -> do
    tow <- currentTow
    let n = set (msg1002_header . gpsObservationHeader_tow) tow m
    yield (RTCM3Msg1002 n (toRTCM3 n))
  (RTCM3Msg1004 m _rtcm3) -> do
    tow <- currentTow
    let n = set (msg1004_header . gpsObservationHeader_tow) tow m
    yield (RTCM3Msg1004 n (toRTCM3 n))
  _rtcm3Msg ->
    pure ()

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

-- | Delay between observations.
--
delay :: MonadIO m => Conduit (Word32, RTCM3Msg) m RTCM3Msg
delay = awaitForever $ uncurry $ \tow -> \case
  (RTCM3Msg1002 m rtcm3) -> peek >>= delayTow tow . fromMaybe maxBound . (fst <$>) >> yield (RTCM3Msg1002 m rtcm3)
  (RTCM3Msg1004 m rtcm3) -> peek >>= delayTow tow . fromMaybe maxBound . (fst <$>) >> yield (RTCM3Msg1004 m rtcm3)
  _rtcm3Msg              -> pure ()

-- | Setup observations with their time of weeks.
--
preDelay :: Monad m => Conduit RTCM3Msg m (Word32, RTCM3Msg)
preDelay = awaitForever $ \case
  (RTCM3Msg1002 m rtcm3) -> yield (m ^. msg1002_header ^. gpsObservationHeader_tow, RTCM3Msg1002 m rtcm3)
  (RTCM3Msg1004 m rtcm3) -> yield (m ^. msg1004_header ^. gpsObservationHeader_tow, RTCM3Msg1004 m rtcm3)
  _rtcm3Msg              -> pure ()

-- | Replay observations.
--
replay :: MonadIO m => Conduit RTCM3Msg m RTCM3Msg
replay = preDelay =$= delay =$= postDelay
