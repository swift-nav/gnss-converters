{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      RTCM32SBP
-- Copyright:   (c) 2015 Mark Fine
-- License:     BSD3
-- Maintainer:  Mark Fine <mark.fine@gmail.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCM3 to SBP tool.

import           BasicPrelude
import           Control.Monad.Trans.Resource
import           Data.Conduit
import           Data.Conduit.Binary
import qualified Data.Conduit.List                 as CL
import           Data.Conduit.Serialization.Binary
import           Data.RTCM3.SBP
import           Data.RTCM3.SBP.Types
import           System.IO

main :: IO ()
main = do
  s <- newStore
  runResourceT $ runConvertT s $
    sourceHandle stdin    =$=
    conduitDecode         =$=
    CL.concatMapM convert =$=
    conduitEncode         $$
    sinkHandle stdout
