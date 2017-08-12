{-# LANGUAGE NoImplicitPrelude #-}
{-# LANGUAGE OverloadedStrings #-}

-- |
-- Module:      Test
-- Copyright:   (c) 2016 Swift Navigation Inc.
-- License:     AllRightsReserved
-- Maintainer:  Swift Navigation <dev@swift-nav.com>
--
-- Test module for GNSS converters

import           BasicPrelude
import qualified Test.Data.RTCM3.SBP      as SBP
import qualified Test.Data.RTCM3.SBP.Time as Time
import           Test.Tasty

tests :: TestTree
tests = testGroup "Tests"
  [ SBP.tests
  , Time.tests
  ]

main :: IO ()
main = defaultMain tests
