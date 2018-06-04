{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.Logging
-- Copyright:   Copyright (C) 2018 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Loggings Conversions.

module Data.RTCM3.SBP.Logging
  ( converter
  ) where

import BasicPrelude
import Control.Lens
import Data.Bits
import Data.Conduit
import Data.RTCM3
import Data.Word
import SwiftNav.SBP

{-# ANN module ("HLint: ignore Use head"::String) #-}

-- | Derive sender from station.
--
toSender :: Word16 -> Word16
toSender station = station .|. 61568

-- | Convert an RTCM 1029 text message into an SBP MsgLog message.
--
converter :: Monad m => Msg1029 -> Conduit i m [SBPMsg]
converter m = do
  let message = m ^. msg1029_message
      sender  = toSender $ message ^. textMessage_station
      m'      = MsgLog
        { _msgLog_level = 6
        , _msgLog_text  = message ^. textMessage_text
        }
  yield [SBPMsgLog m' $ toSBP m' sender]
