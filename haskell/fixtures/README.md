## RTCM3 to SBP test fixtures

The test fixtures in this directory were collected from the UCSF CORS
station. Something like:

```shell
# Collect four seconds of RTCM3
timeout 4 \
  curl -s -N -u <username>:<pwd> -H "Ntrip-Version: Ntrip/2.0" \
  -A "NTRIP curl/7.43.0" http://tiburon.geo.berkeley.edu:2101/UCSF_RTCM3 \
  \> ucsf_bard_four_seconds.rtcm3

# Gold standard RINEX: Convert to RINEX using RTKLIB rtkconv
rtkconv ucsf_bard_four_seconds.rtcm3

# SBP: Convert to SBP using rtcm32sbp from gnss-converters
cat ucsf_bard_four_seconds.rtcm3 | rtcm32sbp > ucsf_bard_four_seconds.rtcm3.sbp

# Inspection: convert SBP to JSON from libsbp Haskell
cat ucsf_bard_four_seconds.rtcm3.sbp | sbp2json | jq .
```
