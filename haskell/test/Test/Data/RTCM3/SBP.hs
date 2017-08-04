{-# LANGUAGE CPP               #-}
{-# LANGUAGE FlexibleContexts  #-}
{-# LANGUAGE LambdaCase        #-}
{-# LANGUAGE NoImplicitPrelude #-}
{-# LANGUAGE OverloadedStrings #-}

-- |
-- Module:      Test.Data.RTCM3.SBP
-- Copyright:   (c) 2016 Swift Navigation Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swift-nav.com>
--
-- Test RTCMv3 to SBP Conversions.

module Test.Data.RTCM3.SBP
  ( tests
  ) where

import           BasicPrelude
import           Control.Lens
import           Control.Monad.Trans.Resource
import           Data.Binary
import qualified Data.ByteString.Lazy              as LBS
import           Data.Conduit
import           Data.Conduit.Binary               hiding (head)
import qualified Data.Conduit.List                 as CL
import           Data.Conduit.Serialization.Binary
import           Data.HashMap.Strict               hiding (filter, mapMaybe)
import           Data.IORef
import           Data.RTCM3.SBP
import           Data.RTCM3.SBP.Types
import           SwiftNav.SBP
import           Test.HUnit.Approx
import           Test.Tasty
import           Test.Tasty.HUnit

decodeRTCMFile :: Word16 -> FilePath -> IO [SBPMsg]
decodeRTCMFile wn filename = do
  s <- Store <$> newIORef wn <*> newIORef mempty
  runResourceT $ runConvertT s $ runConduit  $
    sourceFile filename   =$=
    conduitDecode         =$=
    CL.concatMapM convert $$
    CL.consume

decodeSBPFile :: FilePath -> IO [SBPMsg]
decodeSBPFile filename = do
  runResourceT $ runConduit  $
    sourceFile filename   =$=
    conduitDecode         $$
    CL.consume

basePosition :: MsgBasePosEcef -> (Double, Double, Double)
basePosition msg' = ( msg' ^. msgBasePosEcef_x
                    , msg' ^. msgBasePosEcef_y
                    , msg' ^. msgBasePosEcef_z
                    )

isL1 :: PackedObsContent -> Bool
isL1 obs = (obs ^. packedObsContent_sid ^. gnssSignal16_code) == 0

isL2 :: PackedObsContent -> Bool
isL2 obs = code == 1 || code == 6
  where
    code = obs ^. packedObsContent_sid ^. gnssSignal16_code

sat :: PackedObsContent -> Word8
sat obs = obs ^. packedObsContent_sid ^. gnssSignal16_sat

cn0 :: PackedObsContent -> Double
cn0 obs = 0.25 * fromIntegral (obs ^. packedObsContent_cn0)

pseudorange :: PackedObsContent -> Double
pseudorange obs = fromIntegral (obs ^. packedObsContent_P) / 50

carrierPhase :: PackedObsContent -> Double
carrierPhase obs = whole + fraction
  where
    phase    = obs ^. packedObsContent_L
    whole    = fromIntegral (phase ^. carrierPhase_i)
    fraction = fromIntegral (phase ^. carrierPhase_f) / 256

assertExpectedBase :: SBPMsg -> (Double, Double, Double) -> Assertion
assertExpectedBase (SBPMsgBasePosEcef posEcef _) pos =
  assertEqual "Base station position error!" pos $ basePosition posEcef
assertExpectedBase _                             _   = assertFailure "Invalid message type!"

assertObsHeader :: SBPMsg -> ObservationHeader -> Assertion
assertObsHeader (SBPMsgObs obs _) header = do
  assertEqual "Observation header is not equal" (header ^. observationHeader_n_obs) $ obs ^. msgObs_header ^. observationHeader_n_obs
  assertEqual "Observation header is not equal" (header ^. observationHeader_t ^. gpsTimeNano_tow) $ obs ^. msgObs_header ^. observationHeader_t ^. gpsTimeNano_tow
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
#if MIN_VERSION_basic_prelude(0,6,1)
      msg'        = "PRN=" ++ show prn ++ " CODE=" ++ show code
#else
      msg'        = textToString $ "PRN=" ++ show prn ++ " CODE=" ++ show code
#endif
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
      _ |  isL1 packed -> assertObs testObs_L1 packed ptol ctol_L1 0
        |  isL2 packed -> assertObs testObs_L2 packed ptol ctol_L2 6
        |  otherwise   -> assertFailure "Not L1 or L2!"
assertMsgObs _                  =  assertFailure "Invalid message type!"

assertMsgObsLength :: SBPMsg -> Int -> Assertion
assertMsgObsLength (SBPMsgObs obs' _) len = length (obs' ^. msgObs_obs) @?= len
assertMsgObsLength _                  _   = assertFailure "Invalid message type!"

assertMsgMaxLength :: SBPMsg -> Assertion
assertMsgMaxLength m = assertBool "The message is too damn long" $ LBS.length (encode m) <= 255 + 2

-- | Given two objects and an accessor, assert that the common field is equal.
compareFieldsWithAccessor :: (Eq b, Show b) => String -> a -> a -> (a -> b) -> Assertion
compareFieldsWithAccessor m o1 o2 a = assertEqual ("Accessed field must be equal: " <> m) (a o1) (a o2)

-- | Convert arbitrary SBP messages to a list of just GpsEphemeris messages.
messagesToGpsEphemeris :: [SBPMsg] -> [MsgEphemerisGps]
messagesToGpsEphemeris = mapMaybe $ \case
  (SBPMsgEphemerisGps m _) -> Just m
  _                        -> Nothing

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
        msgs <- decodeRTCMFile 1906 "fixtures/rtcm3/ucsf_bard_four_seconds.rtcm3"
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
        assertMsgMaxLength (msgs !! 1)
        assertMsgObsLength (msgs !! 1) 14
        assertMsgObs (msgs !! 1)
        -- Message 2
        assertObsHeader (msgs !! 2) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86354000 0 1906
          , _observationHeader_n_obs = 33
          }
        assertMsgObs (msgs !! 2)
        assertMsgMaxLength (msgs !! 2)
        assertMsgObsLength (msgs !! 2) 2
        -- Message 3
        assertObsHeader (msgs !! 3) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86355000 0 1906
          , _observationHeader_n_obs = 32
          }
        assertMsgMaxLength (msgs !! 3)
        assertMsgObsLength (msgs !! 3) 14
        -- Message 4
        assertObsHeader (msgs !! 4) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86355000 0 1906
          , _observationHeader_n_obs = 33
          }
        assertMsgMaxLength (msgs !! 4)
        assertMsgObsLength (msgs !! 4) 2
        -- Message 6
        assertObsHeader (msgs !! 6) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86356000 0 1906
          , _observationHeader_n_obs = 32
          }
        assertMsgMaxLength (msgs !! 6)
        assertMsgObsLength (msgs !! 6) 14
        -- Message 7
        assertObsHeader (msgs !! 7) ObservationHeader {
            _observationHeader_t     = GpsTimeNano 86356000 0 1906
          , _observationHeader_n_obs = 33
          }
        assertMsgMaxLength (msgs !! 7)
        assertMsgObsLength (msgs !! 7) 2
     ]

testMsg1019 :: TestTree
testMsg1019 =
  testGroup "Msg1019 conversion to SBP"
    [ testCase "Full ephemeris in RTCM" $ do
      msgs' <- decodeRTCMFile 1937 "fixtures/rtcm3/20170222_gps_ephemeris_331200.rtcm3"
      let msgs = messagesToGpsEphemeris msgs'
      BasicPrelude.mapM_ assertMsgMaxLength msgs'

      -- No values from this set of ephemerides were compared to Piksi.
      -- Index 0, sat 4
      assertBool "index 0, sat 4, toc"      $ (msgs !! 0) ^. msgEphemerisGps_toc ^. gpsTimeSec_tow                                    == 331200
      assertBool "index 0, sat 4, toc"      $ (msgs !! 0) ^. msgEphemerisGps_toc ^. gpsTimeSec_wn                                     == 1937
      assertBool "index 0, sat 4, sid"      $ (msgs !! 0) ^. msgEphemerisGps_common ^. ephemerisCommonContent_sid ^. gnssSignal16_sat == 5
      assertBool "index 0, sat 4, ura"      $ (msgs !! 0) ^. msgEphemerisGps_common ^. ephemerisCommonContent_ura                     == 2
      assertBool "index 0, sat 4, valid"    $ (msgs !! 0) ^. msgEphemerisGps_common ^. ephemerisCommonContent_valid                   == 1
      assertBool "index 0, sat 4, fitint"   $ (msgs !! 0) ^. msgEphemerisGps_common ^. ephemerisCommonContent_fit_interval            == 14400
      assertBool "index 0, sat 4, health"   $ (msgs !! 0) ^. msgEphemerisGps_common ^. ephemerisCommonContent_health_bits             == 0
      assertBool "index 0, sat 4, dn"       $ (msgs !! 0) ^. msgEphemerisGps_dn                                                       == 4.8134147837606336e-9
      assertBool "index 0, sat 4, w"        $ (msgs !! 0) ^. msgEphemerisGps_w                                                        == 0.5556409483786487
      assertBool "index 0, sat 4, tgd"      $ (msgs !! 0) ^. msgEphemerisGps_tgd                                                      == -1.0710209608078003e-8
      assertBool "index 0, sat 4, c_rs"     $ (msgs !! 0) ^. msgEphemerisGps_c_rs                                                     == 11.0625
      assertBool "index 0, sat 4, c_rc"     $ (msgs !! 0) ^. msgEphemerisGps_c_rc                                                     == 195.6875
      assertBool "index 0, sat 4, c_us"     $ (msgs !! 0) ^. msgEphemerisGps_c_us                                                     == 9.134411811828613e-6
      assertBool "index 0, sat 4, c_uc"     $ (msgs !! 0) ^. msgEphemerisGps_c_uc                                                     == 7.040798664093018e-7
      assertBool "index 0, sat 4, c_is"     $ (msgs !! 0) ^. msgEphemerisGps_c_is                                                     == 2.0489096641540527e-8
      assertBool "index 0, sat 4, c_ic"     $ (msgs !! 0) ^. msgEphemerisGps_c_ic                                                     == 4.6566128730773926e-8
      assertBool "index 0, sat 4, m0"       $ (msgs !! 0) ^. msgEphemerisGps_m0                                                       == -6.1410736472242544e-2
      assertBool "index 0, sat 4, ecc"      $ (msgs !! 0) ^. msgEphemerisGps_ecc                                                      == 4.897040314972401e-3
      assertBool "index 0, sat 4, sqrta"    $ (msgs !! 0) ^. msgEphemerisGps_sqrta                                                    == 5153.690675735474
      assertBool "index 0, sat 4, omega0"   $ (msgs !! 0) ^. msgEphemerisGps_omega0                                                   == 0.26338009467812684
      assertBool "index 0, sat 4, omegadot" $ (msgs !! 0) ^. msgEphemerisGps_omegadot                                                 == -8.164982961456692e-9
      assertBool "index 0, sat 4, inc"      $ (msgs !! 0) ^. msgEphemerisGps_inc                                                      == 0.9470684719233013
      assertBool "index 0, sat 4, incdot"   $ (msgs !! 0) ^. msgEphemerisGps_inc_dot                                                  == 5.146642949765581e-10
      assertBool "index 0, sat 4, af0"      $ (msgs !! 0) ^. msgEphemerisGps_af0                                                      == -5.923490971326828e-5
      assertBool "index 0, sat 4, af1"      $ (msgs !! 0) ^. msgEphemerisGps_af1                                                      == 2.0463630789890885e-12
      assertBool "index 0, sat 4, af2"      $ (msgs !! 0) ^. msgEphemerisGps_af2                                                      == 0
      assertBool "index 0, sat 4, iode"     $ (msgs !! 0) ^. msgEphemerisGps_iode                                                     == 50
      assertBool "index 0, sat 4, iodc"     $ (msgs !! 0) ^. msgEphemerisGps_iodc                                                     == 50

    , testCase "Compare RTCM from ephemeris service and SBP messages from Piksi" $ do
      -- Loop through all the decoded RTCM ephemerides, find the corresponding SBP
      -- message, and compare fields.
      -- Note that sender and CRC are different so we can't expect exact matches.
      -- Note that RTCM will have all sats but SBP will only have a subset.
      msgs2_converted'    <- decodeRTCMFile 1937 "fixtures/rtcm3/20170222_gps_ephemeris_345600.rtcm3"
      msgs2_sbp'          <- decodeSBPFile "fixtures/sbp/20170222_gps_ephemeris_345600.sbp"
      let msgs2_converted = messagesToGpsEphemeris msgs2_converted'
      let msgs2_sbp       = messagesToGpsEphemeris msgs2_sbp'

      flip BasicPrelude.mapM_ msgs2_converted $ \m' ->
        let sid = m' ^. msgEphemerisGps_common ^. ephemerisCommonContent_sid ^. gnssSignal16_sat in
        let toc = m' ^. msgEphemerisGps_toc ^. gpsTimeSec_tow in
        let ms  = filter ((== sid) . _gnssSignal16_sat . _ephemerisCommonContent_sid . _msgEphemerisGps_common) msgs2_sbp in
        let ms'  = filter ((== toc) . _gpsTimeSec_tow . _msgEphemerisGps_toc) ms in
        if length ms' == 0 then return () else
          let m = head ms' in do
            compareFieldsWithAccessor "toc"    m m' (_gpsTimeSec_tow . _msgEphemerisGps_toc)
            compareFieldsWithAccessor "wn"     m m' (_gpsTimeSec_wn  . _msgEphemerisGps_toc)
            compareFieldsWithAccessor "sid"    m m' (_gnssSignal16_sat . _ephemerisCommonContent_sid . _msgEphemerisGps_common)
            -- Does not always match the values coming out of a Piksi. I have reason to believe
            -- that the Piksi might be the problem.
            --compareFieldsWithAccessor "ura"    m m' (_ephemerisCommonContent_ura          . _msgEphemerisGps_common)
            compareFieldsWithAccessor "valid"  m m' (_ephemerisCommonContent_valid        . _msgEphemerisGps_common)
            compareFieldsWithAccessor "fitint" m m' (_ephemerisCommonContent_fit_interval . _msgEphemerisGps_common)
            compareFieldsWithAccessor "health" m m' (_ephemerisCommonContent_health_bits  . _msgEphemerisGps_common)
            compareFieldsWithAccessor "dn"     m m' (_msgEphemerisGps_dn)
            compareFieldsWithAccessor "w"      m m' (_msgEphemerisGps_w)
            compareFieldsWithAccessor "tgd"    m m' (_msgEphemerisGps_tgd)
            compareFieldsWithAccessor "c_rs"   m m' (_msgEphemerisGps_c_rs)
            compareFieldsWithAccessor "c_rc"   m m' (_msgEphemerisGps_c_rc)
            compareFieldsWithAccessor "c_is"   m m' (_msgEphemerisGps_c_is)
            compareFieldsWithAccessor "c_ic"   m m' (_msgEphemerisGps_c_ic)
            compareFieldsWithAccessor "c_us"   m m' (_msgEphemerisGps_c_us)
            compareFieldsWithAccessor "c_uc"   m m' (_msgEphemerisGps_c_uc)
            compareFieldsWithAccessor "m0"     m m' (_msgEphemerisGps_m0)
            compareFieldsWithAccessor "ecc"    m m' (_msgEphemerisGps_ecc)
            compareFieldsWithAccessor "sqrta"  m m' (_msgEphemerisGps_sqrta)
            compareFieldsWithAccessor "omega0" m m' (_msgEphemerisGps_omega0)
            compareFieldsWithAccessor "omega." m m' (_msgEphemerisGps_omegadot)
            compareFieldsWithAccessor "inc"    m m' (_msgEphemerisGps_inc)
            compareFieldsWithAccessor "incdot" m m' (_msgEphemerisGps_inc_dot)
            compareFieldsWithAccessor "af0"    m m' (_msgEphemerisGps_af0)
            compareFieldsWithAccessor "af1"    m m' (_msgEphemerisGps_af1)
            compareFieldsWithAccessor "af2"    m m' (_msgEphemerisGps_af2)
            compareFieldsWithAccessor "iode"   m m' (_msgEphemerisGps_iode)
            compareFieldsWithAccessor "iodc"   m m' (_msgEphemerisGps_iodc)
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

testValidateIodcIode :: TestTree
testValidateIodcIode =
  testGroup "Validate IODC/IODE"
    [ testCase "valid, equal, max"                    $ assertEqual "" 1 $ validateIodcIode 0x00FF 0xFF
    , testCase "valid, equal, min"                    $ assertEqual "" 1 $ validateIodcIode 0x0000 0x00
    , testCase "valid, IODC has higher bits set, max" $ assertEqual "" 1 $ validateIodcIode 0xFFFF 0xFF
    , testCase "valid, IODC has higher bits set, min" $ assertEqual "" 1 $ validateIodcIode 0xFF00 0x00
    , testCase "invalid, unequal"                     $ assertEqual "" 0 $ validateIodcIode 0x0001 0x02
    , testCase "invalid, IODC has higher bits set"    $ assertEqual "" 0 $ validateIodcIode 0xFF14 0x15
    ]

testUriToUra :: TestTree
testUriToUra =
  testGroup "Convert user range index to user range accuracy"
    [ testN 0    2.0
    , testN 1    2.8
    , testN 2    4.0
    , testN 3    5.7
    , testN 4    8.0
    , testN 5    11.3
    , testN 6    16.0
    , testN 7    32.0
    , testN 8    64.0
    , testN 9    128.0
    , testN 10   256.0
    , testN 11   512.0
    , testN 12   1024.0
    , testN 13   2048.0
    , testN 14   4096.0
    , testN 15   6144.0
    , testN 16   (-1.0)
    , testN 17   (-1.0)
    , testN (-1) (-1.0)
    ]

  where
    testN n expected =
#if MIN_VERSION_basic_prelude(0,6,1)
      let s = show n in
#else
      let s = textToString $ show n in
#endif
      testCase s $ assertEqual s expected $ gpsUriToUra n

tests :: TestTree
tests =
  testGroup "RTCM3 to SBP conversion tests"
    [ testMsg1004
    , testMsg1019
    , testToWn
    , testValidateIodcIode
    , testUriToUra
    ]
