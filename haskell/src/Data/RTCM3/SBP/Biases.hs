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

import BasicPrelude hiding (mask)
import Control.Lens
import Data.Bits
import Data.Conduit
import Data.Int
import Data.RTCM3
import Data.Word
import SwiftNav.SBP

{-# ANN module ("HLint: ignore Use head"::String) #-}

-- | Derive sender from station.
--
toSender :: Word16 -> Word16
toSender station = station .|. 61440

-- | Expand out array of biases.
--
toBiases :: Word8 -> Int -> [Int16] -> [Int16]
toBiases mask i (bias:biases)
  | testBit mask i = bias : toBiases mask (i+1) biases
toBiases mask i biases
  | null biases = biases
  | otherwise = 0 : toBiases mask (i+1) biases

-- | Convert an RTCM 1230 GLO biases message into an SBP MsgGloBiases.
--
converter :: Monad m => Msg1230 -> Conduit i m [SBPMsg]
converter m = do
  let bias   = m ^. msg1230_bias
      sender = toSender $ bias ^. glonassBias_station
      scale  = bool 50 (-50) $ bias ^. glonassBias_bias
      biases = toBiases (bias ^. glonassBias_mask) 0 (bias ^. glonassBias_biases) <> repeat 0
      m'     = MsgGloBiases
        { _msgGloBiases_mask      = bias ^. glonassBias_mask
        , _msgGloBiases_l1ca_bias = scale * biases !! 0
        , _msgGloBiases_l1p_bias  = scale * biases !! 1
        , _msgGloBiases_l2ca_bias = scale * biases !! 2
        , _msgGloBiases_l2p_bias  = scale * biases !! 3
        }
  yield [SBPMsgGloBiases m' $ toSBP m' sender]
