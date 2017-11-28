{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.Positions
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Position Conversions.

module Data.RTCM3.SBP.Positions
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

-- | FromPosition produces ecef x, y, x and sender values from RTCMv3 position messages.
--
class FromPosition a where
  ecefX  :: a -> Int64
  ecefY  :: a -> Int64
  ecefZ  :: a -> Int64
  sender :: a -> Word16

instance FromPosition Msg1005 where
  ecefX  = view (msg1005_reference . antennaReference_ecef_x)
  ecefY  = view (msg1005_reference . antennaReference_ecef_y)
  ecefZ  = view (msg1005_reference . antennaReference_ecef_z)
  sender = toSender . view (msg1005_reference . antennaReference_station)

instance FromPosition Msg1006 where
  ecefX = view (msg1006_reference . antennaReference_ecef_x)
  ecefY = view (msg1006_reference . antennaReference_ecef_y)
  ecefZ = view (msg1006_reference . antennaReference_ecef_z)
  sender = toSender . view (msg1006_reference . antennaReference_station)

converter :: (Monad m, FromPosition a) => a -> Conduit i m [SBPMsg]
converter m = do
  let x  = fromIntegral (ecefX m) / 10000
      y  = fromIntegral (ecefY m) / 10000
      z  = fromIntegral (ecefZ m) / 10000
      m' = MsgBasePosEcef x y z
  yield [SBPMsgBasePosEcef m' $ toSBP m' $ sender m]
