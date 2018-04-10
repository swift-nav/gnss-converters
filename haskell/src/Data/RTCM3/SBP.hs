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
  , runConvertT
  , runConverter
  ) where

import           BasicPrelude
import           Control.Monad.Reader
import           Data.Conduit
import           Data.IORef
import           Data.RTCM3
import qualified Data.RTCM3.SBP.Biases       as Biases
import qualified Data.RTCM3.SBP.Ephemerides  as Ephemerides
import qualified Data.RTCM3.SBP.Logging      as Logging
import qualified Data.RTCM3.SBP.Observations as Observations
import qualified Data.RTCM3.SBP.Positions    as Positions
import qualified Data.RTCM3.SBP.SSR          as SSR
import           Data.RTCM3.SBP.Time
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
  (RTCM3Msg1019 m _rtcm3) -> Ephemerides.gpsConverter m
  (RTCM3Msg1020 m _rtcm3) -> Ephemerides.glonassConverter m
  (RTCM3Msg1029 m _rtcm3) -> Logging.converter m
  (RTCM3Msg1230 m _rtcm3) -> Biases.converter m
  (RTCM3Msg1060 m _rtcm3) -> SSR.gpsOrbitClockConverter m
  (RTCM3Msg1066 m _rtcm3) -> SSR.glonassOrbitClockConverter m
  (RTCM3Msg1059 m _rtcm3) -> SSR.gpsCodeBiasConverter m
  (RTCM3Msg1065 m _rtcm3) -> SSR.glonassCodeBiasConverter m
  (RTCM3Msg1265 m _rtcm3) -> SSR.gpsPhaseBiasConverter m
  (RTCM3Msg1266 m _rtcm3) -> SSR.glonassPhaseBiasConverter m
  _rtcm3Msg               -> mempty

-- | Setup new storage for converter.
--
newStore :: MonadIO m => m Store
newStore = liftIO $ Store <$> pure currentGpsTime <*> newIORef mempty <*> newIORef mempty

-- | Run converter.
--
runConvertT :: HasStore e => e -> ConvertT e m a -> m a
runConvertT e (ConvertT m) = runReaderT m e

-- | Run converter with default store.
--
runConverter :: MonadIO m => ConvertT Store m a -> m a
runConverter action = do
  s <- newStore
  runConvertT s action
