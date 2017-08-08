{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      SBP2RTCM3
-- Copyright:   (c) 2015 Mark Fine
-- License:     BSD3
-- Maintainer:  Mark Fine <mark.fine@gmail.com>
-- Stability:   experimental
-- Portability: portable
--
-- SBP to RTCM3 tool.

import BasicPrelude
import Control.Monad.Trans.Resource
import Data.Conduit
import Data.Conduit.Binary
import Data.Conduit.Serialization.Binary
import SwiftNav.SBP.RTCM3
import System.IO

main :: IO ()
main =
  runResourceT $
    sourceHandle stdin     =$=
    conduitDecode          =$=
    awaitForever converter =$=
    conduitEncode          $$
    sinkHandle stdout
