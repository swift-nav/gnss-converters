{-# LANGUAGE NoImplicitPrelude #-}
{-# LANGUAGE OverloadedStrings #-}

module Test.Data.RTCM3.SBP.Time
  ( tests
  ) where

import BasicPrelude
import Data.RTCM3.SBP.Time
import Data.Time
import Data.Time.Calendar.WeekDate
import SwiftNav.SBP
import Test.Tasty
import Test.Tasty.HUnit

testRolloverGpsTime :: TestTree
testRolloverGpsTime =
  testGroup "Rollover GPS time tests"
    [ testCase "No rollover" $ do
        rolloverTowGpsTime 0 (GpsTime 0 0 1960) @?= GpsTime 0 0 1960
        rolloverTowGpsTime 302400000 (GpsTime 302400000 0 1960) @?= GpsTime 302400000 0 1960
        rolloverTowGpsTime 604800000 (GpsTime 604800000 0 1960) @?= GpsTime 604800000 0 1960
        rolloverTowGpsTime 0 (GpsTime 302400000 0 1960) @?= GpsTime 0 0 1960
        rolloverTowGpsTime 302400000 (GpsTime 604800000 0 1960) @?= GpsTime 302400000 0 1960
    , testCase "Positive rollover" $ do
        rolloverTowGpsTime 0 (GpsTime 302400001 0 1960) @?= GpsTime 0 0 1961
        rolloverTowGpsTime 0 (GpsTime 604800000 0 1960) @?= GpsTime 0 0 1961
        rolloverTowGpsTime 302399999 (GpsTime 604800000 0 1960) @?= GpsTime 302399999 0 1961
    , testCase "Negative rollover" $ do
        rolloverTowGpsTime 604800000 (GpsTime 0 0 1961) @?= GpsTime 604800000 0 1960
        rolloverTowGpsTime 604800000 (GpsTime 302399999 0 1961) @?= GpsTime 604800000 0 1960
        rolloverTowGpsTime 302400001 (GpsTime 0 0 1961) @?= GpsTime 302400001 0 1960
    ]

testStartDate :: TestTree
testStartDate =
  testGroup "GPS week start date tests"
    [ testCase "Sundays" $ do
        toStartDate (2017, 34, 7) @?= (2017, 34, 7)
        toStartDate (2016, 52, 7) @?= (2016, 52, 7)
        toStartDate (2017,  1, 7) @?= (2017,  1, 7)
    , testCase "Mondays" $ do
        toStartDate (2017, 34, 1) @?= (2017, 33, 7)
        toStartDate (2016, 52, 1) @?= (2016, 51, 7)
        toStartDate (2017,  1, 1) @?= (2016, 52, 7)
    , testCase "Saturdays" $ do
        toStartDate (2017, 34, 6) @?= (2017, 33, 7)
        toStartDate (2016, 52, 6) @?= (2016, 51, 7)
        toStartDate (2017,  1, 6) @?= (2016, 52, 7)
    ]

testToWn :: TestTree
testToWn =
  testGroup "GPS week number tests"
    [ testCase "Sundays" $ do
        toWn (UTCTime (fromWeekDate 2017 34 7) 0) @?= 1964
        toWn (UTCTime (fromWeekDate 2016 52 7) 0) @?= 1930
        toWn (UTCTime (fromWeekDate 2017  1 7) 0) @?= 1931
    , testCase "Mondays" $ do
        toWn (UTCTime (fromWeekDate 2017 34 1) 0) @?= 1963
        toWn (UTCTime (fromWeekDate 2016 52 1) 0) @?= 1929
        toWn (UTCTime (fromWeekDate 2017  1 1) 0) @?= 1930
    , testCase "Saturdays" $ do
        toWn (UTCTime (fromWeekDate 2017 34 6) 0) @?= 1963
        toWn (UTCTime (fromWeekDate 2016 52 6) 0) @?= 1929
        toWn (UTCTime (fromWeekDate 2017  1 6) 0) @?= 1930
    ]

testToTow :: TestTree
testToTow =
  testGroup "GPS time of week tests"
    [ testCase "Sundays" $ do
        toTow (UTCTime (fromWeekDate 2017 34 7) 0) @?= 0
        toTow (UTCTime (fromWeekDate 2016 52 7) 0) @?= 0
        toTow (UTCTime (fromWeekDate 2017  1 7) 0) @?= 0
    , testCase "Mondays" $ do
        toTow (UTCTime (fromWeekDate 2017 34 1) 0) @?= fromIntegral dayMillis
        toTow (UTCTime (fromWeekDate 2016 52 1) 0) @?= fromIntegral dayMillis
        toTow (UTCTime (fromWeekDate 2017  1 1) 0) @?= fromIntegral dayMillis
    , testCase "Saturdays" $ do
        toTow (UTCTime (fromWeekDate 2017 34 6) 0) @?= 6 * fromIntegral dayMillis
        toTow (UTCTime (fromWeekDate 2016 52 6) 0) @?= 6 * fromIntegral dayMillis
        toTow (UTCTime (fromWeekDate 2017  1 6) 0) @?= 6 * fromIntegral dayMillis
    ]

tests :: TestTree
tests =
  testGroup "RTCM3 to SBP time tests"
    [ testRolloverGpsTime
    , testStartDate
    , testToWn
    , testToTow
    ]
