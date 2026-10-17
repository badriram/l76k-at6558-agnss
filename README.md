# l76k-at6558-agnss

Fast cold starts (AGNSS) for the **Quectel L76K** and other GNSS modules on the **AT6558** chip
(CASIC protocol: ATGM336H, L76KB, the Heltec L76K module and the GPS in the Bangle.js 2): inject
**time and position (AID-INI)** together with **CASIC's free orbit-data file**, and a cold start
drops from about 40 s to about 6 s.

Measured on a Quectel L76K (Heltec's L76K GNSS module, AT6558R, firmware URANUS5 V5.3.0.0),
outdoors under a clear sky, three rounds per case, cases run in a different order each round:

| cold start (nothing in the receiver's memory) | runs (s) | typical |
| --- | --- | --- |
| no help | 39.0, 42.3, 42.3 | ~41 s |
| time and position only (AID-INI) | 33.2, 38.2, 38.2 | ~37 s |
| orbit data only (CASIC AGNSS file) | 30.3, 26.3, 34.4 | ~30 s |
| **time and position + orbit data** | **6.3, 6.3, 5.3** | **~6 s** |

Either half alone does little: orbit data is no use without the time and roughly where the
receiver is, and time and position cannot replace the orbit data a cold start spends its ~30 s
downloading. Together they do.

More findings, all on the same receiver, in [`results/`](results/2026-10-17-heltec-l76k.md):

- **Keep the backup supply (V_BCKP) powered and cut the main supply between fixes**: a fix comes
  back in about 2 s after a 60 s power cut, against about 40 s with no backup.
- The STANDBY/WAKEUP pin wakes the receiver in 0.27 s and refixes in about 2 s (20 µA standby, per
  Quectel, against 8 µA in backup).
- **The L76K accepts CASIC orbit-data messages as input** (all 32 MSG-GPSEPH acknowledged outdoors),
  though its documents list them as output only. **It will not output its own orbit data**, so
  capturing and replaying it does not work on this firmware.
- AID-INI is acknowledged only once the receiver has been running about a second; sent straight
  after a restart, it is silently ignored.
- `$PCAS12` timed standby is not supported, and `$PCAS10,1` (warm restart) keeps the orbit data,
  so it behaves like a hot start.

## What is here

| path | what |
| --- | --- |
| `include/casic_agnss.h`, `src/casic_agnss.c` | **the C library**: portable C99, no allocation, no OS, no stdio, byte order explicit. CASIC framing and checksum, AID-INI, CASIC AGNSS file parsing, a MSG-GPSEPH encoder and decoder for RINEX-sourced ephemeris, CFG-MSG, ACK decoding, NMEA commands, a GGA/RMC parser, GPS time, and a stream splitter for the receiver's mixed NMEA-and-binary output |
| `examples/assist_cold_start.c` | the assisted cold start against four platform hooks (UART write/read, milliseconds, sleep): fill them in for Zephyr, ESP-IDF, Arduino or anything else |
| `tests/test_casic.c` | host tests (`make test`), against Quectel's worked examples and independently computed values |
| `micropython/heltec_v4_l76k/assisted_cold_start.py` | the measurement above, on a Heltec WiFi LoRa 32 V4 with its L76K module; logs to `/gnss_assist_log.txt` on the board |
| `micropython/heltec_v4_l76k/hello.py` | power the module and print its NMEA, to check the wiring |
| `tools/fetch_agnss.py` | fetch orbit data from CASIC's file or the IGS broadcast ephemeris (falling back from one to the other), check every frame's checksum, write the raw frames (`casic.bin`) |
| `results/` | measurements, with conditions |

## Using the library

Copy `include/casic_agnss.h` and `src/casic_agnss.c` into your project. Right after the receiver
boots (or after `$PCAS10,2`), wait about a second, then:

```c
uint8_t frame[80];
casic_aid_ini_t aid = {0};
casic_gps_time_from_unix(unix_now, CASIC_GPS_LEAP_SECONDS, &aid.week, &aid.tow_s);
aid.lat_deg = lat;  aid.lon_deg = lon;  aid.height_m = height;   /* roughly right is enough */
aid.pos_acc_m = 100.0f;  aid.time_acc_s = 2.0f;
aid.flags = CASIC_AID_POSITION | CASIC_AID_TIME;
uart_write(frame, casic_aid_ini_frame(&aid, frame, sizeof frame));

/* then CASIC's AGNSS file, decoded from base64: */
long first = casic_agnss_first_frame(agnss, agnss_len);
size_t off = (size_t)first;
casic_frame_t f;
while (first >= 0 && casic_frame_next(agnss, agnss_len, &off, &f) == 1) {
    uart_write(f.frame, f.frame_len);
    delay_ms(25);
}
```

Feed what the receiver sends into a `casic_stream_t` to get whole NMEA sentences and CASIC frames
back (ACKs included); see `examples/assist_cold_start.c` for the whole sequence. Doubles are
assumed to be IEEE 754, as on every common MCU.

## Running it

1. Flash MicroPython (ESP32_GENERIC_S3) to a Heltec WiFi LoRa 32 V4 with the L76K module plugged
   in. **Back up the board's existing firmware first** (`esptool read-flash 0 ALL backup.bin`).
2. `python3 tools/fetch_agnss.py casic.bin`, then copy `casic.bin` and the script to the board
   (`mpremote fs cp casic.bin :casic.bin`, `mpremote run assisted_cold_start.py`). The file is good
   for a few hours.
3. Run it outdoors, antenna up. It takes a first fix, then cold-restarts the receiver (confirming
   the restart by the fix being lost) and injects each kind of aid one second after the restart.

Pins (Heltec V4, from Meshtastic's board definition): GNSS power GPIO 34 (low = on), STANDBY 40
(high = awake), RESET 42, PPS 41, UART TX 38 / RX 39 at 9600 baud.

## The protocol details that cost time

- **The CASIC checksum** is `(ID << 24) + (Class << 16) + Len`, plus the payload summed as
  little-endian 32-bit words. CASIC's own specification prints class and ID the other way round;
  Quectel's L76K document has it right, and every real frame (including CASIC's own file) balances
  only this way. furble found this first.
- **AID-INI** is class 0x0B, ID 0x01, 56 bytes: latitude, longitude, height (doubles, degrees and
  metres, with flag bit 5 set for LLA), GPS time of week (double, s), clock drift (float), position
  accuracy (m), time accuracy (s), drift accuracy, 4 reserved bytes, GPS week (U16), time source
  (U8), flags (U8: bit 0 position valid, bit 1 time valid). GPS time is UTC plus 18 leap seconds,
  counted from 1980-01-06.
- **CASIC's AGNSS file** decodes to a text header and then the frames, ready to write as they are.
- **MSG-GPSEPH** (0x08 0x07, 72 bytes) is the satellite's own LNAV ephemeris at its broadcast
  resolution (IS-GPS-200): angles in semicircles, the week modulo 1024. **The first four bytes,
  "reserved" in the specification, are a check word**: the sum of the payload's other seventeen
  little-endian 32-bit words. All 32 records in CASIC's file, and its MSG-GPSION, obey this.

## Where the orbit data comes from

CASIC's file is the satellites' broadcast ephemeris, and so is the IGS's: the International GNSS
Service merges what its worldwide receivers hear into RINEX navigation files, which NASA CDDIS,
BKG and IGN all publish. The two are interchangeable. Compared field by field on 2026-10-17, all
32 of CASIC's GPS records equal the IGS record with the same toe and IODC in every orbit and clock
term, and the encoded frames are byte for byte identical apart from the URA (accuracy) index,
on which the two sources disagreed for 6 of 32 satellites.

| source | what | size | access |
| --- | --- | --- | --- |
| `casic` (default) | CASIC's AGNSS file via Espruino's mirror: GPS ephemeris + ionosphere, already framed | ~3.6 kB | anonymous; one mirror |
| `igs` (fallback) | BKG's merged broadcast RINEX, refreshed every 15 min; the newest GPS record per satellite is encoded | ~1.3 MB/day gzipped | anonymous HTTPS |
| `--rinex FILE_OR_URL` | any RINEX 3 navigation file: NASA CDDIS (needs an Earthdata login), IGN, or your own | | |

`casic_gpseph_frame()` does the same encoding in C, from a `casic_gps_eph_t` filled with RINEX
values (radians, seconds, metres), so a phone or gateway can convert RINEX itself and send only
the ~2.6 kB of frames. The test suite checks it byte for byte against CASIC's file.

**Measured on the receiver:** IGS-encoded frames are all acknowledged, and an assisted cold start
fixes as fast with them as with CASIC's file: 5.4, 4.3 and 5.4 s against 5.4, 5.3 and 4.3 s
([results](results/2026-10-17-heltec-l76k.md#igs-broadcast-ephemeris-against-casics-file)).

## Credits and sources

- [furble](https://github.com/MaxRink/furble) (MIT): the checksum correction, AID-INI from a cached
  fix, and the capture-and-replay idea (which this firmware turns out not to support).
- [TinyTrackGPSPlus](https://github.com/RafaelReyesCarmona/TinyTrackGPSPlus): pointed to the CASIC
  file for AT6558 receivers.
- [Espruino](https://www.espruino.com/) republishes CASIC's AGNSS data at
  `espruino.com/agps/casic.base64`; Bangle.js 2 uses it with the same AT6558 chip
  ([discussion](https://github.com/orgs/espruino/discussions/7026)).
- [IGS](https://igs.org/) and [BKG](https://igs.bkg.bund.de/) for the merged broadcast ephemeris;
  [NASA CDDIS](https://cddis.nasa.gov/Data_and_Derived_Products/GNSS/broadcast_ephemeris_data.html)
  archives the same files.
- Quectel, *L76K GNSS Protocol Specification* V1.1; Hangzhou Zhongke Microelectronics, *CASIC
  Multimode Satellite Navigation Receiver Protocol Specification*.

CASIC is the China Aerospace Science and Industry Corporation, whose AGNSS service the file
comes from; Espruino republishes it. The data is the satellites' own broadcast orbit data, so
fetching it reveals nothing about the receiver. This repository does not redistribute it, it
fetches it. If that source goes away, `tools/fetch_agnss.py` falls back to the IGS files.

## Licence

MIT. See [LICENSE](LICENSE).
