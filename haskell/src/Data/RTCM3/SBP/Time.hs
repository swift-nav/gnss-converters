{-# OPTIONS  -fno-warn-overflowed-literals #-}
{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.Time
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- SBP GPS Time helpers.

module Data.RTCM3.SBP.Time
  ( gpsLeapMillis
  , hourMillis
  , dayMillis
  , weekMillis
  , toWn
  , toStartDate
  , toTow
  , currentGpsTime
  , rolloverTowGpsTime
  , rolloverEpochGpsTime
  ) where

import BasicPrelude
import Control.Lens
import Data.Time
import Data.Time.Calendar.WeekDate
import Data.Word
import SwiftNav.SBP

-- | Beginning of GPS time.
--
gpsEpoch :: Day
gpsEpoch = fromGregorian 1980 1 6

-- | Number of GPS leap seconds.
--
gpsLeapSeconds :: Integer
gpsLeapSeconds = 18

-- | Number of GPS leap milliseconds.
--
gpsLeapMillis :: Integer
gpsLeapMillis = 1000 * gpsLeapSeconds

-- | Hour seconds
--
hourSeconds :: Integer
hourSeconds = 60 * 60

-- | Hour milliseconds
--
hourMillis :: Integer
hourMillis = 1000 * hourSeconds

-- | Day seconds
--
daySeconds :: Integer
daySeconds = 24 * hourSeconds

-- | Day milliseconds
--
dayMillis :: Integer
dayMillis = 1000 * daySeconds

-- | Number of seconds in a week.
--
weekSeconds :: Integer
weekSeconds = 7 * daySeconds

-- | Number of milliseconds in a week.
--
weekMillis :: Integer
weekMillis = 1000 * weekSeconds

-- | Produce GPS week number from a GPS UTC time.
--
toWn :: UTCTime -> Word16
toWn t = fromIntegral weeks
  where
    days  = diffDays (utctDay t) gpsEpoch
    weeks = days `div` 7

-- | Find the start of the GPS week, which is Sunday.
--
toStartDate :: (Integer, Int, Int) -> (Integer, Int, Int)
toStartDate (year, week, day)
  | day == 7  = (year,   week,   7)
  | week == 1 = (year-1, 52,     7)
  | otherwise = (year,   week-1, 7)

-- | Generate the start of the GPS week UTC time.
--
fromStartDate :: UTCTime -> UTCTime
fromStartDate t = UTCTime (fromWeekDate year week day) 0
  where
    (year, week, day) = toStartDate $ toWeekDate $ utctDay t

-- | Produce GPS time of week from a GPS UTC time.
--
toTow :: UTCTime -> Word32
toTow t = floor $ 1000 * diffUTCTime t (fromStartDate t)

-- | Get current GPS time.
--
currentGpsTime :: MonadIO m => m GpsTimeNano
currentGpsTime = do
  t <- liftIO $ addUTCTime (fromIntegral gpsLeapSeconds) <$> getCurrentTime
  pure $ GpsTimeNano (toTow t) 0 (toWn t)

-- | Update GPS time based on GPS time of week, handling week rollover.
--
rolloverTowGpsTime :: Word32 -> GpsTimeNano -> GpsTimeNano
rolloverTowGpsTime tow t = t & gpsTimeNano_tow .~ tow & rollover
  where
    rollover
      | increment = gpsTimeNano_wn +~ 1
      | decrement = gpsTimeNano_wn +~ -1
      | otherwise = gpsTimeNano_wn +~ 0
    new       = fromIntegral tow
    old       = fromIntegral (t ^. gpsTimeNano_tow)
    increment = old > new && old - new > weekMillis `div` 2
    decrement = new > old && new - old > weekMillis `div` 2

-- | Update GPS time based on GLONASS epoch, handling week rollover.
--
rolloverEpochGpsTime :: Word32 -> GpsTimeNano -> GpsTimeNano
rolloverEpochGpsTime epoch t = rolloverTowGpsTime tow t
  where
    epoch' = fromIntegral epoch - 3 * hourMillis + gpsLeapMillis
    epoch''
      | epoch' < 0 = epoch' + dayMillis
      | otherwise  = epoch'
    dow = fromIntegral (t ^. gpsTimeNano_tow) `div` dayMillis
    tod = fromIntegral (t ^. gpsTimeNano_tow) - dow * dayMillis
    dow'
      | increment = dow + 1 `mod` 7
      | decrement = dow - 1 `mod` 7
      | otherwise = dow
    increment = epoch'' > tod && epoch'' - tod > dayMillis `div` 2
    decrement = tod > epoch'' && tod - epoch'' > dayMillis `div` 2
    tow = fromIntegral $ dow' * dayMillis + epoch''
