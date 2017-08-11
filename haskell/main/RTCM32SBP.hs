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

import BasicPrelude
import Data.Conduit
import Data.Conduit.Binary
import Data.Conduit.Serialization.Binary
import Data.RTCM3.SBP
import System.IO

main :: IO ()
main =
  runConverter $ runConduitRes $
    sourceHandle stdin
      =$= conduitDecode
      =$= awaitForever converter
      =$= conduitEncode
      $$  sinkHandle stdout
