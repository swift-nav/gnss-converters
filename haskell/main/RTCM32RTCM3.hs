{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      RTCM32RTCM3
-- Copyright:   (c) 2015 Mark Fine
-- License:     BSD3
-- Maintainer:  Mark Fine <mark.fine@gmail.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCM3 to RTCM3 tool.

import BasicPrelude
import Data.Conduit
import Data.Conduit.Binary
import Data.Conduit.Serialization.Binary
import Data.RTCM3.Replay
import System.IO

main :: IO ()
main =
  runConduitRes $
    sourceHandle stdin
      =$= conduitDecode
      =$= replayer
      =$= conduitEncode
      $$  sinkHandle stdout
