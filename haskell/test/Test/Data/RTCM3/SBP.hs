{-# LANGUAGE FlexibleContexts #-}

-- |
-- Module:      Test.Data.RTCM3.SBP
-- Copyright:   (c) 2016 Swift Navigation Inc.
-- License:     LGPL-3
-- Maintainer:  Skylark Team <skylark@swift-nav.com>
--
-- Test RTCMv3 to SBP Conversions.

module Test.Data.RTCM3.SBP
  ( tests
  ) where

import           BasicPrelude
import           Control.Lens
import           Control.Monad.Trans.Resource
import           Data.Conduit
import           Data.Conduit.Binary
import qualified Data.Conduit.List                 as CL
import           Data.Conduit.Serialization.Binary
import           Data.HashMap.Strict
import           Data.IORef
import           Data.RTCM3.SBP
import           Data.RTCM3.SBP.Types
import           SwiftNav.SBP
import           Test.HUnit.Approx
import           Test.Tasty
import           Test.Tasty.HUnit

decodeRTCMFile :: FilePath -> IO [SBPMsg]
decodeRTCMFile filename = do
  s <- Store <$> newIORef 1906 <*> newIORef mempty
  runResourceT $ runConvertT s $ runConduit  $
    sourceFile filename   =$=
    conduitDecode         =$=
    CL.concatMapM convert $$
    CL.consume

basePosition :: MsgBasePosEcef -> (Double, Double, Double)
basePosition msg' = ( msg' ^. msgBasePosEcef_x
                    , msg' ^. msgBasePosEcef_y
                    , msg' ^. msgBasePosEcef_z
                    )

isL1 :: PackedObsContent -> Bool
isL1 obs = (obs ^. packedObsContent_sid ^. gnssSignal16_code) == l1CSidCode

isL2 :: PackedObsContent -> Bool
isL2 obs = code == l2CMSidCode || code == l2PSidCode
  where
    code = obs ^. packedObsContent_sid ^. gnssSignal16_code

sat :: PackedObsContent -> Word8
sat obs = obs ^. packedObsContent_sid ^. gnssSignal16_sat

cn0 :: PackedObsContent -> Double
cn0 obs = 0.25 * fromIntegral (obs ^. packedObsContent_cn0)

pseudorange :: PackedObsContent -> Double
pseudorange obs = fromIntegral (obs ^. packedObsContent_P) / sbpCMinM

carrierPhase :: PackedObsContent -> Double
carrierPhase obs = whole + fraction
  where
    phase    = obs ^. packedObsContent_L
    whole    = fromIntegral (phase ^. carrierPhase_i)
    fraction = fromIntegral (phase ^. carrierPhase_f) / q32Width

assertExpectedBase :: SBPMsg -> (Double, Double, Double) -> Assertion
assertExpectedBase (SBPMsgBasePosEcef posEcef _) pos =
  assertEqual "Base station position error!" pos $ basePosition posEcef
assertExpectedBase _                             _   = assertFailure "Invalid message type!"

assertObsHeader :: SBPMsg -> ObservationHeader -> Assertion
assertObsHeader (SBPMsgObs obs _) header =
  assertEqual "Observation header is not equal" header $ obs ^. msgObs_header
assertObsHeader _                 _      = assertFailure "Invalid message type!"

assertObs :: HashMap Word8 (Double, Double, Double)
          -> PackedObsContent
          -> Double        -- ^ Pseudorange tolerance (meters)
          -> Double        -- ^ Carrier phase tolerance (cycles)
          -> Word8         -- ^ Satellite band code
          -> Assertion
assertObs truth obs ptol ctol sig = do
  let prn         = sat obs
      def         = (0, 0, 0)
      (p, c, snr) = lookupDefault def prn truth
      code        = obs ^. packedObsContent_sid ^. gnssSignal16_code
      msg'        = textToString $ "PRN=" ++ show prn ++ " CODE=" ++ show code
  -- Pseudorange representation error
  assertApproxEqual ("Incorrect pseudorange" ++ msg')   ptol p $ pseudorange obs
  -- Carrier phase representation error
  assertApproxEqual ("Incorrect carrier phase" ++ msg') ctol c $ carrierPhase obs
  -- SNRs/cn0 should be exact
  assertEqual ("Incorrect SNR" ++ msg')  snr $ cn0 obs
  assertEqual ("Incorrect code" ++ msg') sig   code

assertMsgObs :: SBPMsg -> Assertion
assertMsgObs (SBPMsgObs obs' _) = do
  let ptol    = 0.020 -- 2cm representation errorn
      ctol_L1 = 0.005 -- carrier phase / L1 wavelength = 0.001m/0.190m cycles
      ctol_L2 = 0.004 -- carrier phase / L2 wavelength = 0.001m/0.250m cycles
  forM_ (obs' ^. msgObs_obs) $ \packed ->
    case () of
      _ |  isL1 packed -> assertObs testObs_L1 packed ptol ctol_L1 l1CSidCode
        |  isL2 packed -> assertObs testObs_L2 packed ptol ctol_L2 l2PSidCode
        |  otherwise   -> assertFailure "Not L1 or L2!"
assertMsgObs _                  =  assertFailure "Invalid message type!"

assertMsgObsLength :: SBPMsg -> Int -> Assertion
assertMsgObsLength (SBPMsgObs obs' _) len = length (obs' ^. msgObs_obs) @?= len
assertMsgObsLength _                  _   = assertFailure "Invalid message type!"

-- | L1 observations: PRN => Pseudorange, Carrier Phase, SNR
--
-- From fixtures/rinex/ucsf_bard_four_seconds.obs RINEX
testObs_L1 :: HashMap Word8 (Double, Double, Double)
testObs_L1 = fromList [ (5,  (20982568.242,  110264218.428, 50.500))
                      , (31, (24390286.418,  128171817.525, 43.000))
                      , (25, (20560669.144,  108047101.226, 51.750))
                      , (12, (21678581.616,  113921748.286, 49.500))
                      , (29, (20679317.864,  108670633.460, 50.000))
                      , (21, (23719676.742,  124647833.324, 41.000))
                      , (2,  (22612474.630,  118829414.562, 44.000))
                      , (20, (21815089.336,  114639133.794, 48.000))
                      ]

-- | L2 observations: PRN => Pseudorange, Carrier Phase, SNR
--
-- From fixtures/rinex/ucsf_bard_four_seconds.obs RINEX
testObs_L2 :: HashMap Word8 (Double, Double, Double)
testObs_L2 = fromList [ (5,  (20982564.742, 85920194.806, 40.000))
                      , (31, (24390281.338, 99874145.710, 22.750))
                      , (25, (20560668.244, 84192554.659, 42.750))
                      , (12, (21678578.556, 88770214.224, 38.500))
                      , (29, (20679314.544, 84678442.345, 42.750))
                      , (21, (23719674.482, 97128180.775, 23.250))
                      , (2,  (22612468.690, 92594355.238, 30.750))
                      , (20, (21815086.356, 89329221.086, 34.500))
                      ]

testMsg1004 :: TestTree
testMsg1004 =
  testGroup "Msg1004 conversion to SBP"
    [ testCase "Four seconds of RTCM3" $ do
        msgs <- decodeRTCMFile "fixtures/rtcm3/ucsf_bard_four_seconds.rtcm3"
        -- There are eight messages
        length msgs @?= 8
        -- Check message 0 and 5 against values in RINEX file:
        --   fixtures/rinex/ucsf_bard_four_seconds.obs
        let expectedBasePos = (-2709557.0660, -4260015.9160, 3884773.0630)
        assertExpectedBase (msgs !! 0) expectedBasePos
        assertExpectedBase (msgs !! 5) expectedBasePos
        -- Message 1
        assertObsHeader (msgs !! 1) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86354000 0 1906
          , _observationHeader_n_obs = 32
          }
        assertMsgObsLength (msgs !! 1) 15
        assertMsgObs (msgs !! 1)
        -- Message 2
        assertObsHeader (msgs !! 2) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86354000 0 1906
          , _observationHeader_n_obs = 33
          }
        assertMsgObs (msgs !! 2)
        assertMsgObsLength (msgs !! 2) 1
        -- Message 3
        assertObsHeader (msgs !! 3) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86355000 0 1906
          , _observationHeader_n_obs = 32
          }
        assertMsgObsLength (msgs !! 3) 15
        -- Message 4
        assertObsHeader (msgs !! 4) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86355000 0 1906
          , _observationHeader_n_obs = 33
          }
        assertMsgObsLength (msgs !! 4) 1
        -- Message 6
        assertObsHeader (msgs !! 6) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86356000 0 1906
          , _observationHeader_n_obs = 32
          }
        assertMsgObsLength (msgs !! 6) 15
        -- Message 7
        assertObsHeader (msgs !! 7) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86356000 0 1906
          , _observationHeader_n_obs = 33
          }
        assertMsgObsLength (msgs !! 7) 1
     ]

testToWn :: TestTree
testToWn =
  testGroup "MJD to GPS week number"
    [ testCase "Day 0, Week 0" $
       toWn mjdEpoch @?= 0
    , testCase "Day 4, Week 1906" $
       toWn 57590 @?= 1906
    , testCase "Day 0, Week 1906" $
       toWn 57586 @?= 1906
    , testCase "Day 7, Week 1905" $
       toWn 57585 @?= 1905
    , testCase "Day 7, Week 1906" $
       toWn 57592 @?= 1906
    , testCase "Day 0, Week 1907" $
       toWn 57593 @?= 1907
    ]

testUpdateGpsTime :: TestTree
testUpdateGpsTime =
  testGroup "Update GPS Time"
    [ testCase "old TOW < new TOW" $ do
        let old = GpsTimeNano 1 0 10
            new = updateGpsTime 2 old
        new ^. gpsTimeNano_wn @?= 10
    , testCase "old TOW = new TOW" $ do
        let old = GpsTimeNano 1 0 10
            new = updateGpsTime 1 old
        new ^. gpsTimeNano_wn @?= 10
    , testCase "old TOW > new TOW" $ do
        let old = GpsTimeNano 1 0 10
            new = updateGpsTime 0 old
        new ^. gpsTimeNano_wn @?= 11
    ]

tests :: TestTree
tests =
  testGroup "RTCM3 to SBP conversion tests"
    [ testMsg1004
    , testToWn
    , testUpdateGpsTime
    ]
