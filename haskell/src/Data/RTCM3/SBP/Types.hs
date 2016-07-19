{-# LANGUAGE ConstraintKinds            #-}
{-# LANGUAGE FlexibleInstances          #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE MultiParamTypeClasses      #-}
{-# LANGUAGE TemplateHaskell            #-}
{-# LANGUAGE TypeFamilies               #-}
{-# LANGUAGE UndecidableInstances       #-}

-- |
-- Module:      Data.RTCM3.SBP.Types
-- Copyright:   Copyright (C) 2015 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Mark Fine <dev@swiftnav.com>
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

newtype ConvertT e m a = ConvertT { unConvertT :: ReaderT e m a }
  deriving
    ( Functor
    , Applicative
    , Monad
    , MonadIO
    , MonadTrans
    )

instance MonadThrow m => MonadThrow (ConvertT r m) where
    throwM = lift . throwM

instance MonadResource m => MonadResource (ConvertT r m) where
  liftResourceT = lift . liftResourceT

instance Monad m => MonadReader e (ConvertT e m) where
    ask     = ConvertT ask
    local f = ConvertT . local f . unConvertT
    reader  = ConvertT . reader

instance MonadTransControl (ConvertT r) where
    type StT (ConvertT r) a = StT (ReaderT r) a

    liftWith = defaultLiftWith ConvertT unConvertT
    restoreT = defaultRestoreT ConvertT

instance MonadBase b m => MonadBase b (ConvertT r m) where
    liftBase = liftBaseDefault

data Store = Store
  { _storeWn :: IORef Word16
  } deriving ( Eq )

$(makeClassy ''Store)

runConvertT :: HasStore e => e -> ConvertT e m a -> m a
runConvertT e (ConvertT m) = runReaderT m e

type MonadStore e m =
  ( MonadIO m
  , MonadReader e m
  , MonadThrow m
  , HasStore e
  )
