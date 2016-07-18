-- |
-- Module:      Test
-- Copyright:   (c) 2016 Swift Navigation Inc.
-- License:     AllRightsReserved
-- Maintainer:  Skylark Team <skylark@swift-nav.com>
--
-- Test module for GNSS converters

import           BasicPrelude
import qualified Test.Data.RTCM3.SBP as SBP
import           Test.Tasty

tests :: TestTree
tests = testGroup "Tests"
  [ SBP.tests
  ]

main :: IO ()
main = defaultMain tests
