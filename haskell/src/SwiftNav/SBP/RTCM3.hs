-- |
-- Module:      SwiftNav.SBP.RTCM3
-- Copyright:   Copyright (C) 2015 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Mark Fine <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- SBP to RTCMv3 Conversions.

module SwiftNav.SBP.RTCM3
  ( convert
  ) where

import BasicPrelude
import Data.RTCM3
import SwiftNav.SBP

convert :: SBPMsg -> Maybe RTCM3Msg
convert = undefined
