{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP.SSR
-- Copyright:   Copyright (C) 2017 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Biases Conversions.

module Data.RTCM3.SBP.SSR
  ( gpsOrbitClockConverter
  , glonassOrbitClockConverter
  , gpsCodeBiasConverter
  , glonassCodeBiasConverter
  , gpsPhaseBiasConverter
  , glonassPhaseBiasConverter
  ) where

import BasicPrelude
import Control.Lens
import Data.Conduit
import Data.RTCM3
import Data.RTCM3.SBP.Time
import Data.RTCM3.SBP.Types
import SwiftNav.SBP

toGpsTimeSec :: MonadStore e m => Word32 -> m GpsTimeSec
toGpsTimeSec epochs = do
  time <- view storeCurrentGpsTime
  wn   <- view gpsTime_wn <$> liftIO time
  pure GpsTimeSec
    { _gpsTimeSec_tow = epochs
    , _gpsTimeSec_wn  = wn
    }

toGlonassTimeSec :: MonadStore e m => Word32 -> m GpsTimeSec
toGlonassTimeSec epochs = do
  time <- view storeCurrentGpsTime
  t    <- liftIO time
  let epochs' = fromIntegral epochs * 1000 - 3 * hourMillis + gpsLeapMillis
      epochs''
        | epochs' < 0 = epochs' + dayMillis
        | otherwise = epochs'
      dow = fromIntegral (t ^. gpsTime_tow) `div` dayMillis
  pure GpsTimeSec
    { _gpsTimeSec_tow = fromIntegral $ (dow * dayMillis + epochs'') `div` 1000
    , _gpsTimeSec_wn  = t ^. gpsTime_wn
    }

gpsOrbitClockConverter :: MonadStore e m => Msg1060 -> Conduit i m [SBPMsg]
gpsOrbitClockConverter m = do
  time <- toGpsTimeSec $ m ^. msg1060_header . gpsOrbitClockCorrectionHeader_epochs
  yield $ flip map (m ^. msg1060_corrections) $ \c -> do
    let m' = MsgSsrOrbitClock
             { _msgSsrOrbitClock_time            = time
             , _msgSsrOrbitClock_sid             = GnssSignal
               { _gnssSignal_sat  = c ^. gpsOrbitClockCorrection_sat
               , _gnssSignal_code = 0
               }
             , _msgSsrOrbitClock_update_interval = m ^. msg1060_header . gpsOrbitClockCorrectionHeader_updateInterval
             , _msgSsrOrbitClock_iod_ssr         = m ^. msg1060_header . gpsOrbitClockCorrectionHeader_iod
             , _msgSsrOrbitClock_iod             = c ^. gpsOrbitClockCorrection_iode
             , _msgSsrOrbitClock_radial          = c ^. gpsOrbitClockCorrection_deltaRadial
             , _msgSsrOrbitClock_along           = c ^. gpsOrbitClockCorrection_deltaAlongTrack
             , _msgSsrOrbitClock_cross           = c ^. gpsOrbitClockCorrection_deltaCrossTrack
             , _msgSsrOrbitClock_dot_radial      = c ^. gpsOrbitClockCorrection_dotDeltaRadial
             , _msgSsrOrbitClock_dot_along       = c ^. gpsOrbitClockCorrection_dotDeltaAlongTrack
             , _msgSsrOrbitClock_dot_cross       = c ^. gpsOrbitClockCorrection_dotDeltaCrossTrack
             , _msgSsrOrbitClock_c0              = c ^. gpsOrbitClockCorrection_deltaClockC0
             , _msgSsrOrbitClock_c1              = c ^. gpsOrbitClockCorrection_deltaClockC1
             , _msgSsrOrbitClock_c2              = c ^. gpsOrbitClockCorrection_deltaClockC2
             }
    SBPMsgSsrOrbitClock m' $ toSBP m' 61440

glonassOrbitClockConverter :: MonadStore e m => Msg1066 -> Conduit i m [SBPMsg]
glonassOrbitClockConverter m = do
  time <- toGlonassTimeSec $ m ^. msg1066_header . glonassOrbitClockCorrectionHeader_epochs
  yield $ flip map (m ^. msg1066_corrections) $ \c -> do
    let m' = MsgSsrOrbitClock
             { _msgSsrOrbitClock_time            = time
             , _msgSsrOrbitClock_sid             = GnssSignal
               { _gnssSignal_sat  = c ^. glonassOrbitClockCorrection_sat
               , _gnssSignal_code = 3
                 }
             , _msgSsrOrbitClock_update_interval = m ^. msg1066_header . glonassOrbitClockCorrectionHeader_updateInterval
             , _msgSsrOrbitClock_iod_ssr         = m ^. msg1066_header . glonassOrbitClockCorrectionHeader_iod
             , _msgSsrOrbitClock_iod             = c ^. glonassOrbitClockCorrection_iode
             , _msgSsrOrbitClock_radial          = c ^. glonassOrbitClockCorrection_deltaRadial
             , _msgSsrOrbitClock_along           = c ^. glonassOrbitClockCorrection_deltaAlongTrack
             , _msgSsrOrbitClock_cross           = c ^. glonassOrbitClockCorrection_deltaCrossTrack
             , _msgSsrOrbitClock_dot_radial      = c ^. glonassOrbitClockCorrection_dotDeltaRadial
             , _msgSsrOrbitClock_dot_along       = c ^. glonassOrbitClockCorrection_dotDeltaAlongTrack
             , _msgSsrOrbitClock_dot_cross       = c ^. glonassOrbitClockCorrection_dotDeltaCrossTrack
             , _msgSsrOrbitClock_c0              = c ^. glonassOrbitClockCorrection_deltaClockC0
             , _msgSsrOrbitClock_c1              = c ^. glonassOrbitClockCorrection_deltaClockC1
             , _msgSsrOrbitClock_c2              = c ^. glonassOrbitClockCorrection_deltaClockC2
             }
    SBPMsgSsrOrbitClock m' $ toSBP m' 61440

gpsCodeBiasConverter :: MonadStore e m => Msg1059 -> Conduit i m [SBPMsg]
gpsCodeBiasConverter m = do
  time <- toGpsTimeSec $ m ^. msg1059_header . gpsCodeBiasCorrectionHeader_epochs
  yield $ flip map (m ^. msg1059_corrections) $ \c -> do
    let m' = MsgSsrCodeBiases
             { _msgSsrCodeBiases_time            = time
             , _msgSsrCodeBiases_sid             = GnssSignal
               { _gnssSignal_sat  = c ^. gpsCodeBiasCorrection_sat
               , _gnssSignal_code = 0
               }
             , _msgSsrCodeBiases_update_interval = m ^. msg1059_header . gpsCodeBiasCorrectionHeader_updateInterval
             , _msgSsrCodeBiases_iod_ssr         = m ^. msg1059_header . gpsCodeBiasCorrectionHeader_iod
             , _msgSsrCodeBiases_biases          = flip map (c ^. gpsCodeBiasCorrection_codeBiases) $ \b -> CodeBiasesContent
               { _codeBiasesContent_code  = b ^. gpsCodeBias_signal
               , _codeBiasesContent_value = b ^. gpsCodeBias_codeBias
               }
             }
    SBPMsgSsrCodeBiases m' $ toSBP m' 61440

glonassCodeBiasConverter :: MonadStore e m => Msg1065 -> Conduit i m [SBPMsg]
glonassCodeBiasConverter m = do
  time <- toGlonassTimeSec $ m ^. msg1065_header . glonassCodeBiasCorrectionHeader_epochs
  yield $ flip map (m ^. msg1065_corrections) $ \c -> do
    let m' = MsgSsrCodeBiases
             { _msgSsrCodeBiases_time            = time
             , _msgSsrCodeBiases_sid             = GnssSignal
               { _gnssSignal_sat  = c ^. glonassCodeBiasCorrection_sat
               , _gnssSignal_code = 3
               }
             , _msgSsrCodeBiases_update_interval = m ^. msg1065_header . glonassCodeBiasCorrectionHeader_updateInterval
             , _msgSsrCodeBiases_iod_ssr         = m ^. msg1065_header . glonassCodeBiasCorrectionHeader_iod
             , _msgSsrCodeBiases_biases          = flip map (c ^. glonassCodeBiasCorrection_codeBiases) $ \b -> CodeBiasesContent
               { _codeBiasesContent_code  = b ^. glonassCodeBias_signal
               , _codeBiasesContent_value = b ^. glonassCodeBias_codeBias
               }
             }
    SBPMsgSsrCodeBiases m' $ toSBP m' 61440

gpsPhaseBiasConverter :: MonadStore e m => Msg1265 -> Conduit i m [SBPMsg]
gpsPhaseBiasConverter m = do
  time <- toGpsTimeSec $ m ^. msg1265_header . gpsPhaseBiasCorrectionHeader_epochs
  yield $ flip map (m ^. msg1265_corrections) $ \c -> do
    let m' = MsgSsrPhaseBiases
             { _msgSsrPhaseBiases_time            = time
             , _msgSsrPhaseBiases_sid             = GnssSignal
               { _gnssSignal_sat  = c ^. gpsPhaseBiasCorrection_sat
               , _gnssSignal_code = 0
               }
             , _msgSsrPhaseBiases_update_interval = m ^. msg1265_header . gpsPhaseBiasCorrectionHeader_updateInterval
             , _msgSsrPhaseBiases_iod_ssr         = m ^. msg1265_header . gpsPhaseBiasCorrectionHeader_iod
             , _msgSsrPhaseBiases_dispersive_bias = bool 0 1 $ m ^. msg1265_header . gpsPhaseBiasCorrectionHeader_dispersive
             , _msgSsrPhaseBiases_mw_consistency  = bool 0 1 $ m ^. msg1265_header . gpsPhaseBiasCorrectionHeader_mw
             , _msgSsrPhaseBiases_yaw             = c ^. gpsPhaseBiasCorrection_yawAngle
             , _msgSsrPhaseBiases_yaw_rate        = c ^. gpsPhaseBiasCorrection_yawRate
             , _msgSsrPhaseBiases_biases          = flip map (c ^. gpsPhaseBiasCorrection_phaseBiases) $ \b -> PhaseBiasesContent
               { _phaseBiasesContent_code                       = b ^. gpsPhaseBias_signal
               , _phaseBiasesContent_integer_indicator          = bool 0 1 $ b ^. gpsPhaseBias_integer
               , _phaseBiasesContent_widelane_integer_indicator = b ^. gpsPhaseBias_wideLaneInteger
               , _phaseBiasesContent_discontinuity_counter      = b ^. gpsPhaseBias_discontinuityCounter
               , _phaseBiasesContent_bias                       = b ^. gpsPhaseBias_phaseBias
               }
             }
    SBPMsgSsrPhaseBiases m' $ toSBP m' 61440

glonassPhaseBiasConverter :: MonadStore e m => Msg1266 -> Conduit i m [SBPMsg]
glonassPhaseBiasConverter m = do
  time <- toGlonassTimeSec $ m ^. msg1266_header . glonassPhaseBiasCorrectionHeader_epochs
  yield $ flip map (m ^. msg1266_corrections) $ \c -> do
    let m' = MsgSsrPhaseBiases
             { _msgSsrPhaseBiases_time            = time
             , _msgSsrPhaseBiases_sid             = GnssSignal
               { _gnssSignal_sat  = c ^. glonassPhaseBiasCorrection_sat
               , _gnssSignal_code = 3
               }
             , _msgSsrPhaseBiases_update_interval = m ^. msg1266_header . glonassPhaseBiasCorrectionHeader_updateInterval
             , _msgSsrPhaseBiases_iod_ssr         = m ^. msg1266_header . glonassPhaseBiasCorrectionHeader_iod
             , _msgSsrPhaseBiases_dispersive_bias = bool 0 1 $ m ^. msg1266_header . glonassPhaseBiasCorrectionHeader_dispersive
             , _msgSsrPhaseBiases_mw_consistency  = bool 0 1 $ m ^. msg1266_header . glonassPhaseBiasCorrectionHeader_mw
             , _msgSsrPhaseBiases_yaw             = c ^. glonassPhaseBiasCorrection_yawAngle
             , _msgSsrPhaseBiases_yaw_rate        = c ^. glonassPhaseBiasCorrection_yawRate
             , _msgSsrPhaseBiases_biases          = flip map (c ^. glonassPhaseBiasCorrection_phaseBiases) $ \b -> PhaseBiasesContent
               { _phaseBiasesContent_code                       = b ^. glonassPhaseBias_signal
               , _phaseBiasesContent_integer_indicator          = bool 0 1 $ b ^. glonassPhaseBias_integer
               , _phaseBiasesContent_widelane_integer_indicator = b ^. glonassPhaseBias_wideLaneInteger
               , _phaseBiasesContent_discontinuity_counter      = b ^. glonassPhaseBias_discontinuityCounter
               , _phaseBiasesContent_bias                       = b ^. glonassPhaseBias_phaseBias
               }
             }
    SBPMsgSsrPhaseBiases m' $ toSBP m' 61440
