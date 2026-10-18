# Predicted GPS orbit data: backtest, 2026-10-18

**What was predicted**: `tools/predict/predict_agnss.py` run on the IGS ultra-rapid orbit
`IGS0OPSULT_20262850600_02D_15M_ORB.SP3` (measured 2026-10-12 06:00 to 10-13 06:00 UTC, IGS
prediction to 10-14 06:00) with its ERP file, calibrated against BKG's merged broadcasts of
2026-10-12 and 10-13. Output: 2,720 MSG-GPSEPH frames, 32 satellites, one every 2 h.

**What it was graded against**: what the satellites actually broadcast from 2026-10-13 to 10-17
(BKG `BRDC00WRD_S`, days 286-290), at 15-minute steps through each predicted record's central
two hours, using the broadcast record a receiver would have used at that moment. The frames were
decoded from the output file, so encoding is included.

**Metric**: signal-in-space range error, `sqrt((0.98 R - c dT)^2 + (A^2 + C^2) / 49)` with R, A, C
the radial, along-track and cross-track orbit differences and dT the clock difference: the part of
a range measurement's error that comes from the satellite's data. Position error is roughly this
times the geometry factor (about 2). The broadcasts themselves are good to about 0.5 m, so values
below that measure agreement with the broadcast rather than truth.

| days after the IGS orbit | range error median | 95th percentile | worst | orbit 3D median | URA index sent |
| --- | --- | --- | --- | --- | --- |
| 0.0-0.5 | 0.32 m | 0.97 m | 2.2 m | 0.7 m | 2 |
| 0.5-1.0 | 0.64 m | 2.51 m | 5.7 m | 1.0 m | 2 |
| 1.0-1.5 | 2.38 m | 4.87 m | 7.1 m | 7.8 m | 4 |
| 1.5-2.0 | 3.22 m | 7.02 m | 9.1 m | 12.7 m | 4 |
| 2.0-2.5 | 4.17 m | 9.48 m | 13.1 m | 17.8 m | 6 |
| 2.5-3.0 | 5.26 m | 13.76 m | 20.1 m | 23.7 m | 6 |
| 3.0-3.5 | 6.55 m | 20.58 m | 29.9 m | 29.9 m | 7 |
| 3.5-4.0 | 7.98 m | 29.26 m | 41.9 m | 36.2 m | 7 |
| 4.0-4.5 | 9.46 m | 40.33 m | 57.2 m | 42.6 m | 8 |
| 4.5-5.0 | 10.57 m | 53.55 m | 66.0 m | 46.9 m | 8 |

For comparison, reusing a real broadcast record past its validity reached a median range error of
~58 m after 12 hours and ~120 m after 22 (orbit off by ~390 m).

Notes:

- The force-model fit to the 48 h IGS arc is 2.6-3.6 m RMS for every satellite. A constant
  acceleration along the third ECOM axis comes out at ~80 nm/s², far larger than is usual for GPS,
  so something is still unmodelled; finding it should slow the growth after day 1.
- The first day uses the IGS's own prediction directly (fitted to 0.1 m), which is why it is so
  much better than days 1-5.
- Days 5-7 were not graded: BKG keeps only about a week of files.
- Not measured on a receiver.
