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
    [ testCase "No rollover" $
        rolloverTowGpsTime 510191000 (GpsTimeNano 510191000 0 1961) @?= (GpsTimeNano 510191000 0 1961)
    , testCase "Positive rollover" $
        rolloverTowGpsTime 0 (GpsTimeNano 604800000 0 1961) @?= (GpsTimeNano 0 0 1962)
    , testCase "Negative rollover" $
        rolloverTowGpsTime 604800000 (GpsTimeNano 0 0 1961) @?= (GpsTimeNano 604800000 0 1960)
    ]

tests :: TestTree
tests =
  testGroup "RTCM3 to SBP time tests"
    [ testRolloverGpsTime
    ]
