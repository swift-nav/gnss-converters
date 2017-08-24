{-# LANGUAGE NoImplicitPrelude #-}
{-# LANGUAGE OverloadedStrings #-}

module Test.Data.RTCM3.SBP.Time
  ( tests
  ) where

import BasicPrelude
import Test.Tasty
import Test.Tasty.HUnit
import Data.RTCM3.SBP.Time
import SwiftNav.SBP

testRolloverGpsTime :: TestTree
testRolloverGpsTime =
  testGroup "Rollover GPS time tests"
    [ testCase "No rollover" $ do
        rolloverTowGpsTime 0 (GpsTimeNano 0 0 1960) @?= (GpsTimeNano 0 0 1960)
        rolloverTowGpsTime 302400000 (GpsTimeNano 302400000 0 1960) @?= (GpsTimeNano 302400000 0 1960)
        rolloverTowGpsTime 604800000 (GpsTimeNano 604800000 0 1960) @?= (GpsTimeNano 604800000 0 1960)
        rolloverTowGpsTime 0 (GpsTimeNano 302400000 0 1960) @?= (GpsTimeNano 0 0 1960)
        rolloverTowGpsTime 302400000 (GpsTimeNano 604800000 0 1960) @?= (GpsTimeNano 302400000 0 1960)
    , testCase "Positive rollover" $ do
        rolloverTowGpsTime 0 (GpsTimeNano 302400001 0 1960) @?= (GpsTimeNano 0 0 1961)
        rolloverTowGpsTime 0 (GpsTimeNano 604800000 0 1960) @?= (GpsTimeNano 0 0 1961)
        rolloverTowGpsTime 302399999 (GpsTimeNano 604800000 0 1960) @?= (GpsTimeNano 302399999 0 1961)
    , testCase "Negative rollover" $ do
        rolloverTowGpsTime 604800000 (GpsTimeNano 0 0 1961) @?= (GpsTimeNano 604800000 0 1960)
        rolloverTowGpsTime 604800000 (GpsTimeNano 302399999 0 1961) @?= (GpsTimeNano 604800000 0 1960)
        rolloverTowGpsTime 302400001 (GpsTimeNano 0 0 1961) @?= (GpsTimeNano 302400001 0 1960)
    ]

tests :: TestTree
tests =
  testGroup "RTCM3 to SBP time tests"
    [ testRolloverGpsTime
    ]
