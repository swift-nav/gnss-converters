{-# LANGUAGE NoImplicitPrelude #-}
{-# LANGUAGE OverloadedStrings #-}

module Test.Data.RTCM3.SBP
  ( tests
  ) where

import BasicPrelude                      hiding (map)
import Control.Lens
import Data.Aeson
import Data.Aeson.Encode.Pretty
import Data.ByteString.Lazy              hiding (ByteString, map)
import Data.Conduit
import Data.Conduit.Binary
import Data.Conduit.List
import Data.Conduit.Serialization.Binary
import Data.RTCM3.SBP
import Data.RTCM3.SBP.Types
import Data.Word
import SwiftNav.SBP
import Test.Tasty
import Test.Tasty.Golden

testGpsTime :: Applicative f => Word32 -> Word16 -> f GpsTime
testGpsTime tow wn = pure $ GpsTime tow 0 wn

encodeLine :: ToJSON a => a -> ByteString
encodeLine v = toStrict $ encodePretty' defConfig { confCompare = compare } v <> "\n"

testConverter :: FilePath -> FilePath -> IO ()
testConverter f t = do
  s <- newStore <&> storeCurrentGpsTime .~ testGpsTime 510191000 1961
  runConvertT s $ runConduitRes $
    sourceFile f
      =$= conduitDecode
      =$= awaitForever converter
      =$= map encodeLine
      $$  sinkFile t

goldenFileTest :: TestName -> FilePath -> FilePath -> TestTree
goldenFileTest name s d = do
  let n = ("test/golden" </>)
      f = "from_" <> d
      t = "to_" <> d
  goldenVsFile name (n f) (n t) $ testConverter (n s) (n t)

testObservations :: TestTree
testObservations =
  testGroup "Observations tests"
    [ goldenFileTest "glo-day-rollover" "glo_day_rollover.rtcm" "glo_day_rollover.json"
    ]

tests :: TestTree
tests =
  testGroup "RTCM3 to SBP tests"
    [ testObservations
    ]
