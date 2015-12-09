{-# LANGUAGE LambdaCase #-}

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

fromMsgObs :: MsgObs -> Msg1002
fromMsgObs _obs = Msg1002
  { _msg1002_header = undefined
  , _msg1002_observations = undefined
  }

fromMsgBasePosEcef :: MsgBasePosEcef -> Msg1005
fromMsgBasePosEcef _basePosEcef = Msg1005
  { _msg1005_reference = undefined
  }

convert :: SBPMsg -> Maybe RTCM3Msg
convert = \case
  (SBPMsgObs msg _sbp) -> Just $
    RTCM3Msg1002 msg' $ toRTCM3 msg' where
      msg' = fromMsgObs msg
  (SBPMsgBasePosEcef msg _sbp) -> Just $
    RTCM3Msg1005 msg' $ toRTCM3 msg' where
      msg' = fromMsgBasePosEcef msg
  _sbpMsg -> Nothing
