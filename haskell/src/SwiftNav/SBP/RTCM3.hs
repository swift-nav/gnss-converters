{-# LANGUAGE LambdaCase #-}

-- |
-- Module:      SwiftNav.SBP.RTCM3
-- Copyright:   Copyright (C) 2015 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
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
  (SBPMsgObs m _sbp) -> Just $
    RTCM3Msg1002 m' $ toRTCM3 m' where
      m' = fromMsgObs m
  (SBPMsgBasePosEcef m _sbp) -> Just $
    RTCM3Msg1005 m' $ toRTCM3 m' where
      m' = fromMsgBasePosEcef m
  _sbpMsg -> Nothing
