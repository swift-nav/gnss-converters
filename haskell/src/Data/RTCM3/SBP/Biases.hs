{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.Biases
-- Copyright:   Copyright (C) 2017 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Biases Conversions.

module Data.RTCM3.SBP.Biases
  ( converter
  ) where

import BasicPrelude
import Control.Lens
import Data.Bits
import Data.Conduit
import Data.RTCM3
import Data.Word
import SwiftNav.SBP

-- | Derive sender from station.
--
toSender :: Word16 -> Word16
toSender station = station .|. 61440

-- | Convert an RTCM 1230 GLO biases message into an SBP MsgGloBiases.
--
converter :: Monad m => Msg1230 -> Conduit i m [SBPMsg]
converter m = do
  let bias   = m ^. msg1230_bias
      sender = toSender $ bias ^. glonassBias_station
      scale  = bool 50 (-50) $ bias ^. glonassBias_bias
      m'     = MsgGloBiases
        { _msgGloBiases_mask      = bias ^. glonassBias_mask
        , _msgGloBiases_l1ca_bias = scale * bias ^. glonassBias_l1ca
        , _msgGloBiases_l1p_bias  = scale * bias ^. glonassBias_l1p
        , _msgGloBiases_l2ca_bias = scale * bias ^. glonassBias_l2ca
        , _msgGloBiases_l2p_bias  = scale * bias ^. glonassBias_l2p
        }
  yield [SBPMsgGloBiases m' $ toSBP m' sender]
