{-# LANGUAGE LambdaCase        #-}
{-# LANGUAGE NoImplicitPrelude #-}

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
  ( converter
  ) where

import BasicPrelude
import Data.Conduit
import Data.RTCM3
import SwiftNav.SBP

converter :: Monad m => SBPMsg -> Conduit i m [RTCM3Msg]
converter = \case
  _sbpMsg -> pure mempty
