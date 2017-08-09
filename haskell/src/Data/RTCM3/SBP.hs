{-# LANGUAGE LambdaCase        #-}
{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Conversions.

module Data.RTCM3.SBP
  ( converter
  , newStore
  ) where

import           BasicPrelude
import           Data.Conduit
import           Data.IORef
import           Data.RTCM3
import qualified Data.RTCM3.SBP.Ephemerides  as Ephemerides
import qualified Data.RTCM3.SBP.Observations as Observations
import qualified Data.RTCM3.SBP.Positions    as Positions
import           Data.RTCM3.SBP.Types
import           SwiftNav.SBP

-- | Convert an RTCM message into possibly multiple SBP messages.
--
converter :: MonadStore e m => RTCM3Msg -> Conduit i m [SBPMsg]
converter = \case
  (RTCM3Msg1002 m _rtcm3) -> Observations.converter m
  (RTCM3Msg1004 m _rtcm3) -> Observations.converter m
  (RTCM3Msg1010 m _rtcm3) -> Observations.converter m
  (RTCM3Msg1012 m _rtcm3) -> Observations.converter m
  (RTCM3Msg1005 m _rtcm3) -> Positions.converter m
  (RTCM3Msg1006 m _rtcm3) -> Positions.converter m
  (RTCM3Msg1019 m _rtcm3) -> Ephemerides.converter m
  _rtcm3Msg               -> mempty

-- | Setup new storage for converter.
--
newStore :: IO Store
newStore = Store <$> newIORef mempty <*> newIORef mempty
