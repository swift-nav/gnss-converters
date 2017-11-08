{-# OPTIONS  -fno-warn-orphans          #-}
{-# LANGUAGE ConstraintKinds            #-}
{-# LANGUAGE FlexibleInstances          #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE MultiParamTypeClasses      #-}
{-# LANGUAGE NoImplicitPrelude          #-}
{-# LANGUAGE TemplateHaskell            #-}
{-# LANGUAGE TypeFamilies               #-}
{-# LANGUAGE UndecidableInstances       #-}

-- |
-- Module:      Data.RTCM3.SBP.Types
-- Copyright:   Copyright (C) 2015 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- Types for RTCMv3 to SBP Conversions.

module Data.RTCM3.SBP.Types where

import BasicPrelude
import Control.Lens
import Control.Monad.Base
import Control.Monad.Catch
import Control.Monad.Reader
import Control.Monad.Trans.Control
import Control.Monad.Trans.Resource
import Data.IORef
import Data.Word
import SwiftNav.SBP

type GpsTimeMap = HashMap Word16 GpsTime

instance Ord GpsTime where
  compare (GpsTime tow _ns wn) (GpsTime tow' _ns' wn')
    | wn > wn'   = GT
    | wn < wn'   = LT
    | tow > tow' = GT
    | tow < tow' = LT
    | otherwise  = EQ

newtype ConvertT e m a = ConvertT { unConvertT :: ReaderT e m a }
  deriving
    ( Functor, Applicative, Monad, MonadIO, MonadTrans, MonadThrow, MonadReader e )

instance MonadResource m => MonadResource (ConvertT r m) where
  liftResourceT = lift . liftResourceT
  {-# INLINE liftResourceT #-}

instance MonadBaseControl b m => MonadBaseControl b (ConvertT r m) where
  type StM (ConvertT r m) a = ComposeSt (ConvertT r) m a
  liftBaseWith = defaultLiftBaseWith
  {-# INLINE liftBaseWith #-}
  restoreM = defaultRestoreM
  {-# INLINE restoreM #-}

instance MonadTransControl (ConvertT r) where
  type StT (ConvertT r) a = StT (ReaderT r) a
  liftWith = defaultLiftWith ConvertT unConvertT
  {-# INLINE liftWith #-}
  restoreT = defaultRestoreT ConvertT
  {-# INLINE restoreT #-}

instance MonadBase b m => MonadBase b (ConvertT r m) where
  liftBase = liftBaseDefault
  {-# INLINE liftBase #-}

data Store = Store
  { _storeCurrentGpsTime :: IO GpsTime
  , _storeGpsTimeMap     :: IORef GpsTimeMap
  , _storeObservations   :: IORef (Vector PackedObsContent)
  }

$(makeClassy ''Store)

type MonadStore e m =
  ( MonadIO m
  , MonadReader e m
  , MonadThrow m
  , HasStore e
  )
