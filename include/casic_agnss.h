/* SPDX-License-Identifier: MIT */
/*
 * casic_agnss: assisted cold starts for CASIC GNSS receivers (Quectel L76K and other AT6558-based
 * modules). Portable C99: no allocation, no OS, no stdio, byte order handled explicitly. Doubles
 * are assumed to be IEEE 754 binary64, as on every common MCU and host.
 *
 * The sequence that cuts a cold start from ~40 s to ~6 s (see results/):
 *   1. after the receiver boots or restarts, wait about one second (earlier, AID-INI is ignored);
 *   2. send AID-INI with the time and an approximate position (casic_aid_ini_frame);
 *   3. send the frames of CASIC's AGNSS file (casic_agnss_first_frame + casic_frame_next).
 *
 * MIT licence. See LICENSE.
 */
#ifndef CASIC_AGNSS_H
#define CASIC_AGNSS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ CASIC binary frames */

#define CASIC_SYNC1 0xBA
#define CASIC_SYNC2 0xCE
#define CASIC_OVERHEAD 10u        /* sync (2) + length (2) + class + id + checksum (4) */
#define CASIC_MAX_PAYLOAD 1024u

/* Message classes and ids used here. */
#define CASIC_CLS_ACK 0x05
#define CASIC_ID_ACK_NACK 0x00
#define CASIC_ID_ACK_ACK 0x01
#define CASIC_CLS_CFG 0x06
#define CASIC_ID_CFG_MSG 0x01
#define CASIC_CLS_MSG 0x08
#define CASIC_ID_MSG_BDSUTC 0x00
#define CASIC_ID_MSG_BDSION 0x01
#define CASIC_ID_MSG_BDSEPH 0x02
#define CASIC_ID_MSG_GPSUTC 0x05
#define CASIC_ID_MSG_GPSION 0x06
#define CASIC_ID_MSG_GPSEPH 0x07
#define CASIC_CLS_AID 0x0B
#define CASIC_ID_AID_INI 0x01
#define CASIC_AID_INI_LEN 56u

/*
 * The CASIC checksum: (id << 24) + (class << 16) + length, plus the payload summed as little-endian
 * 32-bit words (a trailing partial word is ignored). CASIC's own specification prints class and id
 * the other way round; real frames, CASIC's AGNSS file included, balance only this way.
 */
uint32_t casic_checksum(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

/* Build a frame into out. Returns its length, or 0 if out is too small or len too large. */
size_t casic_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                   uint8_t *out, size_t out_cap);

/* A frame found in a buffer: points into that buffer. */
typedef struct {
    uint8_t cls, id;
    const uint8_t *payload;
    uint16_t len;
    const uint8_t *frame;       /* the whole frame, sync to checksum */
    size_t frame_len;
} casic_frame_t;

/* Check one frame starting at buf. Returns 1 and fills f if it is whole and its checksum holds. */
int casic_frame_parse(const uint8_t *buf, size_t len, casic_frame_t *f);

/*
 * Iterate over consecutive frames in a buffer, such as CASIC's AGNSS file after its header.
 * *offset is advanced past each frame returned. Returns 1 for a good frame, 0 at the end, -1 at a
 * malformed frame (stop there rather than send a desynchronised stream).
 */
int casic_frame_next(const uint8_t *buf, size_t len, size_t *offset, casic_frame_t *f);

/*
 * CASIC's AGNSS file (as served base64 at espruino.com/agps/casic.base64, once decoded) is a short
 * text header followed by frames: typically 32 MSG-GPSEPH and a MSG-GPSION, GPS only. Returns the
 * offset of the first frame, or -1 if there is none.
 */
long casic_agnss_first_frame(const uint8_t *buf, size_t len);

/* Decode an ACK or NACK frame: returns 1 for ACK, 0 for NACK, -1 if f is neither. */
int casic_ack(const casic_frame_t *f, uint8_t *acked_cls, uint8_t *acked_id);

/* CFG-MSG to set a message's rate, or with rate 0xFFFF to poll it once. Returns frame length. */
size_t casic_cfg_msg(uint8_t msg_cls, uint8_t msg_id, uint16_t rate, uint8_t *out, size_t out_cap);

/* ------------------------------------------------------------------ AID-INI */

/* GPS time is UTC plus the leap seconds since 1980 (18 since 2017), counted from 1980-01-06. */
#define CASIC_GPS_LEAP_SECONDS 18
void casic_gps_time_from_unix(int64_t unix_seconds, int leap_seconds, uint16_t *week, double *tow);

typedef struct {
    double lat_deg, lon_deg, height_m;   /* used when CASIC_AID_POSITION is set */
    uint16_t week;                       /* GPS week and time of week, when CASIC_AID_TIME is set */
    double tow_s;
    float pos_acc_m;                     /* how far off the position may be */
    float time_acc_s;                    /* how far off the time may be */
    uint8_t flags;
} casic_aid_ini_t;

#define CASIC_AID_POSITION 0x01          /* position valid */
#define CASIC_AID_TIME 0x02              /* time valid */
#define CASIC_AID_LLA 0x20               /* position is latitude/longitude/height (set for you) */

/* The 56-byte AID-INI payload. */
void casic_aid_ini_payload(const casic_aid_ini_t *aid, uint8_t out[CASIC_AID_INI_LEN]);

/* The whole AID-INI frame (66 bytes). Returns its length, or 0 if out_cap is too small. */
size_t casic_aid_ini_frame(const casic_aid_ini_t *aid, uint8_t *out, size_t out_cap);

/* ------------------------------------------------------------------ MSG-GPSEPH */

/*
 * One GPS satellite's broadcast ephemeris (LNAV), in the units RINEX navigation files use: angles
 * in radians, rates in radians per second, so a RINEX record copies straight in. The encoder
 * scales each field to the satellite's own broadcast resolution (IS-GPS-200), which is also what
 * CASIC's 72-byte MSG-GPSEPH carries; values taken from RINEX therefore encode exactly. Checked
 * field by field against CASIC's own AGNSS file: every orbit and clock term identical.
 */
typedef struct {
    double sqrt_a;                       /* square root of the semi-major axis, m^1/2 */
    double e;                            /* eccentricity */
    double omega, m0, i0, omega0;        /* argument of perigee, mean anomaly, inclination, RAAN: rad */
    double omega_dot, delta_n, idot;     /* rad/s */
    double cuc, cus, cic, cis;           /* rad */
    double crc, crs;                     /* m */
    double toe, toc;                     /* ephemeris and clock reference times, seconds of GPS week */
    double af0, af1, af2;                /* clock: s, s/s, s/s^2 */
    double tgd;                          /* group delay, s */
    uint16_t week;                       /* GPS week of toe; full or 10-bit, stored modulo 1024 */
    uint16_t iodc;
    uint8_t ura;                         /* URA index 0-15 (casic_ura_index converts from metres) */
    uint8_t health;                      /* 0 = healthy */
    uint8_t svid;                        /* PRN 1-32 */
    uint8_t valid;                       /* 3 = usable, 2 = expired, 1 = unhealthy, 0 = invalid */
} casic_gps_eph_t;

#define CASIC_GPSEPH_LEN 72u
#define CASIC_GPSEPH_VALID 3
#define CASIC_GPS_PI 3.1415926535898     /* the value IS-GPS-200 defines semicircles with */

/*
 * The first four bytes of MSG-GPSEPH and MSG-GPSION, "reserved" in CASIC's specification, hold the
 * sum of the payload's remaining little-endian 32-bit words. Every record in CASIC's own AGNSS file
 * obeys this; the encoders here fill it in, for MSG-BDSEPH too (which CASIC's file never carries).
 */
uint32_t casic_msg_word_sum(const uint8_t *payload, uint16_t len);

/* The URA index for a RINEX "SV accuracy" in metres (IS-GPS-200 table 20-I). */
uint8_t casic_ura_index(double metres);

/* The 72-byte payload. Returns 1, or 0 if a field is out of its broadcast range. */
int casic_gpseph_payload(const casic_gps_eph_t *eph, uint8_t out[CASIC_GPSEPH_LEN]);

/* The whole frame (82 bytes). Returns its length, or 0 if out_cap is too small or a field is out of range. */
size_t casic_gpseph_frame(const casic_gps_eph_t *eph, uint8_t *out, size_t out_cap);

/* Decode a MSG-GPSEPH payload (from CASIC's file, say) back into physical units. Returns 1, or 0 if len is wrong. */
int casic_gpseph_decode(const uint8_t *payload, uint16_t len, casic_gps_eph_t *eph);

/* ------------------------------------------------------------------ MSG-BDSEPH */

/*
 * One BeiDou satellite's broadcast ephemeris (B1I D1/D2), in RINEX units like casic_gps_eph_t.
 * Times are BeiDou time (BDT: GPS time minus 14 s, weeks counted from 2006-01-01). The 92-byte
 * MSG-BDSEPH carries each field at its broadcast resolution, so RINEX values encode exactly (all
 * 849 BeiDou records of a day's IGS file land on the grid). The CASIC specification numbers BeiDou
 * satellites 1-37; newer BeiDou-3 satellites (C38 and up) are refused.
 */
typedef struct {
    double sqrt_a, e;
    double omega, m0, i0, omega0;        /* rad */
    double omega_dot, delta_n, idot;     /* rad/s */
    double cuc, cus, cic, cis;           /* rad */
    double crc, crs;                     /* m */
    double toe, toc;                     /* seconds of BDT week */
    double af0, af1, af2;
    double tgd1;                         /* B1I group delay, s */
    uint16_t week;                       /* BDT week */
    uint8_t aodc, aode;                  /* ages of clock and ephemeris data, as broadcast */
    uint8_t ura;                         /* URA index (casic_ura_index) */
    uint8_t health;                      /* SatH1: 0 = healthy */
    uint8_t svid;                        /* PRN 1-37 */
    uint8_t valid;                       /* as casic_gps_eph_t */
} casic_bds_eph_t;

#define CASIC_BDSEPH_LEN 92u
#define CASIC_BDS_MAX_SVID 37

/* The 92-byte payload. Returns 1, or 0 if a field is out of its broadcast range. */
int casic_bdseph_payload(const casic_bds_eph_t *eph, uint8_t out[CASIC_BDSEPH_LEN]);

/* The whole frame (102 bytes). Returns its length, or 0. */
size_t casic_bdseph_frame(const casic_bds_eph_t *eph, uint8_t *out, size_t out_cap);

/* Decode a MSG-BDSEPH payload. Returns 1, or 0 if len is wrong. */
int casic_bdseph_decode(const uint8_t *payload, uint16_t len, casic_bds_eph_t *eph);

/* ------------------------------------------------------------------ NMEA */

/*
 * Build "$<body>*<checksum>\r\n" into out, for the receiver's text commands: "PCAS10,2" (cold
 * restart), "PCAS10,0" (hot), "PCAS02,1000" (fix interval, ms), "PCAS04,3" (GPS + BeiDou).
 * Returns the length, or 0 if out is too small.
 */
size_t casic_nmea_command(const char *body, char *out, size_t out_cap);

/* What the GGA and RMC sentences say. Fields are only updated by the sentences that carry them. */
typedef struct {
    int fix_quality;                     /* GGA: 0 none, 1 GPS, 2 differential, ... */
    int satellites;                      /* GGA: satellites used */
    int have_position;
    double lat_deg, lon_deg, height_m;
    int have_time;                       /* RMC with status A: a valid UTC date and time */
    int year, month, day, hour, minute;
    double second;
} casic_nmea_fix_t;

/*
 * Parse one NMEA sentence (with or without the leading '$', with or without CR/LF), checking its
 * checksum. Returns 1 for a GGA, 2 for an RMC, 0 for another valid sentence, -1 if malformed.
 */
int casic_nmea_parse(const char *sentence, size_t len, casic_nmea_fix_t *fix);

/* Seconds since 1970 from a valid RMC time (have_time set), for casic_gps_time_from_unix. */
int64_t casic_nmea_unix_time(const casic_nmea_fix_t *fix);

/* ------------------------------------------------------------------ the receiver's byte stream */

/*
 * Split the receiver's output, which mixes NMEA text and CASIC frames, into whole sentences and
 * frames. Feed it bytes as they arrive; it calls back for each complete item. No allocation: the
 * caller owns the struct and its buffer.
 */
typedef void (*casic_on_nmea)(void *ctx, const char *sentence, size_t len);
typedef void (*casic_on_frame)(void *ctx, const casic_frame_t *frame);

typedef struct {
    uint8_t buf[CASIC_MAX_PAYLOAD + CASIC_OVERHEAD];
    size_t n;
    casic_on_nmea on_nmea;
    casic_on_frame on_frame;
    void *ctx;
} casic_stream_t;

void casic_stream_init(casic_stream_t *s, casic_on_nmea on_nmea, casic_on_frame on_frame, void *ctx);
void casic_stream_feed(casic_stream_t *s, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* CASIC_AGNSS_H */
