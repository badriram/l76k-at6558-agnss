/* Host tests for casic_agnss. Expected values come from Quectel's L76K protocol specification
 * (worked examples) or were computed independently. All positions are made up. */
#include "casic_agnss.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures, checks;
#define CHECK(cond) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

static int hexeq(const uint8_t *got, size_t n, const char *hex)
{
    size_t i;
    for (i = 0; i < n; i++) {
        unsigned b;
        if (sscanf(hex + i * 3, "%2x", &b) != 1 || b != got[i]) return 0;
    }
    return strlen(hex) == n * 3 - 1;
}

static void test_frames(void)
{
    uint8_t out[80];
    casic_frame_t f;
    uint8_t c, i;
    /* Quectel L76K spec, CFG-MSG examples: an empty Get, and setting the GGA rate. */
    size_t n = casic_frame(0x06, 0x01, NULL, 0, out, sizeof out);
    CHECK(n == 10 && hexeq(out, n, "ba ce 00 00 06 01 00 00 06 01"));
    n = casic_cfg_msg(0x4E, 0x00, 1, out, sizeof out);
    CHECK(n == 14 && hexeq(out, n, "ba ce 04 00 06 01 4e 00 01 00 52 00 07 01"));
    CHECK(casic_frame_parse(out, n, &f) == 1 && f.cls == 0x06 && f.id == 0x01 && f.len == 4);
    out[7] ^= 1;                                         /* corrupt it */
    CHECK(casic_frame_parse(out, n, &f) == 0);
    CHECK(casic_frame(0x06, 0x01, out, 8, out, 12) == 0); /* too small */

    /* Quectel's ACK example. */
    {
        static const uint8_t ack[] = {0xBA, 0xCE, 0x04, 0x00, 0x05, 0x01, 0x06, 0x01, 0x00, 0x00,
                                      0x0A, 0x01, 0x05, 0x01};
        CHECK(casic_frame_parse(ack, sizeof ack, &f) == 1);
        CHECK(casic_ack(&f, &c, &i) == 1 && c == 0x06 && i == 0x01);
    }
}

static void test_gps_time_and_aid(void)
{
    uint16_t week;
    double tow;
    uint8_t out[80];
    casic_frame_t f;
    casic_aid_ini_t a;
    size_t n;
    double lat;
    uint64_t u;
    int k;

    casic_gps_time_from_unix(1700000000, CASIC_GPS_LEAP_SECONDS, &week, &tow);
    CHECK(week == 2288 && tow == 252818.0);

    memset(&a, 0, sizeof a);
    a.lat_deg = 12.575;
    a.lon_deg = 12.575;
    a.height_m = 100.5;
    a.week = week;
    a.tow_s = tow;
    a.pos_acc_m = 100.0f;
    a.time_acc_s = 2.0f;
    a.flags = CASIC_AID_POSITION | CASIC_AID_TIME;
    n = casic_aid_ini_frame(&a, out, sizeof out);
    CHECK(n == 66);
    CHECK(casic_frame_parse(out, n, &f) == 1 && f.cls == 0x0B && f.id == 0x01 && f.len == 56);
    CHECK(f.payload[55] == 0x23);                        /* position + time + LLA */
    CHECK((f.payload[52] | f.payload[53] << 8) == 2288);
    for (u = 0, k = 7; k >= 0; k--) u = u << 8 | f.payload[k];
    memcpy(&lat, &u, sizeof lat);
    CHECK(lat == 12.575);
}

static void test_nmea(void)
{
    char out[40];
    casic_nmea_fix_t fix;
    static const char gga[] = "$GNGGA,221320.000,1234.5000,N,01234.5000,E,1,08,1.2,100.5,M,0.0,M,,*7D\r\n";
    static const char rmc[] = "$GNRMC,221320.000,A,1234.5000,N,01234.5000,E,0.00,0.00,141123,,,A,V*0E";
    static const char sw[] = "GPGGA,221320.000,1234.5000,S,01234.5000,W,1,05,1.2,7.0,M,0.0,M,,*62";
    static const char nofix[] = "$GNGGA,,,,,,0,00,25.5,,,,,,*64";

    CHECK(casic_nmea_command("PCAS10,2", out, sizeof out) == 14 && strcmp(out, "$PCAS10,2*1E\r\n") == 0);
    CHECK(casic_nmea_command("PCAS10,2", out, 10) == 0);

    memset(&fix, 0, sizeof fix);
    CHECK(casic_nmea_parse(gga, strlen(gga), &fix) == 1);
    CHECK(fix.fix_quality == 1 && fix.satellites == 8 && fix.have_position);
    CHECK(fabs(fix.lat_deg - 12.575) < 1e-9 && fabs(fix.lon_deg - 12.575) < 1e-9 && fix.height_m == 100.5);
    CHECK(casic_nmea_parse(rmc, strlen(rmc), &fix) == 2 && fix.have_time);
    CHECK(fix.year == 2023 && fix.month == 11 && fix.day == 14 && fix.hour == 22 && fix.minute == 13);
    CHECK(casic_nmea_unix_time(&fix) == 1700000000);

    memset(&fix, 0, sizeof fix);
    CHECK(casic_nmea_parse(sw, strlen(sw), &fix) == 1 && fix.lat_deg < 0 && fix.lon_deg < 0);
    memset(&fix, 0, sizeof fix);
    CHECK(casic_nmea_parse(nofix, strlen(nofix), &fix) == 1 && fix.fix_quality == 0 && !fix.have_position);
    {
        char bad[sizeof gga];
        memcpy(bad, gga, sizeof gga);
        bad[20] = '9';                                   /* checksum no longer holds */
        CHECK(casic_nmea_parse(bad, strlen(bad), &fix) == -1);
    }
}

/* A synthetic AGNSS file: a text header and three made-up frames of the right kinds. */
static size_t make_agnss(uint8_t *buf, size_t cap)
{
    static const char hdr[] = "AGNSS data from CASIC.\nDataLength: 0.\nLimitation: 1.\n";
    uint8_t eph[72], ion[16];
    size_t n = strlen(hdr);
    int k;
    memcpy(buf, hdr, n);
    for (k = 0; k < 72; k++) eph[k] = (uint8_t)(k * 7);
    for (k = 0; k < 16; k++) ion[k] = (uint8_t)(k * 3);
    n += casic_frame(CASIC_CLS_MSG, CASIC_ID_MSG_GPSEPH, eph, 72, buf + n, cap - n);
    eph[0] = 1;
    n += casic_frame(CASIC_CLS_MSG, CASIC_ID_MSG_GPSEPH, eph, 72, buf + n, cap - n);
    n += casic_frame(CASIC_CLS_MSG, CASIC_ID_MSG_GPSION, ion, 16, buf + n, cap - n);
    return n;
}

static void test_agnss_file(void)
{
    uint8_t buf[400];
    size_t len = make_agnss(buf, sizeof buf), off;
    long first = casic_agnss_first_frame(buf, len);
    casic_frame_t f;
    int count = 0, r;
    CHECK(first == (long)strlen("AGNSS data from CASIC.\nDataLength: 0.\nLimitation: 1.\n"));
    off = (size_t)first;
    while ((r = casic_frame_next(buf, len, &off, &f)) == 1) count++;
    CHECK(r == 0 && count == 3 && off == len);

    buf[first + 20] ^= 0xFF;                             /* corrupt the first frame */
    off = (size_t)first;
    CHECK(casic_frame_next(buf, len, &off, &f) == -1);
}

struct counts { int nmea, frames, gga, acks; };
static void on_nmea(void *ctx, const char *s, size_t n)
{
    struct counts *c = ctx;
    casic_nmea_fix_t fix;
    memset(&fix, 0, sizeof fix);
    c->nmea++;
    if (casic_nmea_parse(s, n, &fix) == 1) c->gga++;
}
static void on_frame(void *ctx, const casic_frame_t *f)
{
    struct counts *c = ctx;
    c->frames++;
    if (casic_ack(f, NULL, NULL) == 1) c->acks++;
}

static void test_stream(void)
{
    static casic_stream_t s;
    struct counts c = {0, 0, 0, 0};
    uint8_t buf[600];
    size_t n = 0, i;
    static const char gga[] = "$GNGGA,221320.000,1234.5000,N,01234.5000,E,1,08,1.2,100.5,M,0.0,M,,*7D\r\n";
    static const uint8_t ack[] = {0xBA, 0xCE, 0x04, 0x00, 0x05, 0x01, 0x06, 0x01, 0x00, 0x00,
                                  0x0A, 0x01, 0x05, 0x01};
    memcpy(buf + n, "\x01\x02junk", 6); n += 6;           /* noise first */
    memcpy(buf + n, gga, strlen(gga)); n += strlen(gga);
    memcpy(buf + n, ack, sizeof ack); n += sizeof ack;
    memcpy(buf + n, "$GNGGA,,,", 9); n += 9;               /* a sentence cut short */
    memcpy(buf + n, ack, sizeof ack); n += sizeof ack;
    n += make_agnss(buf + n, sizeof buf - n);             /* header text, then three frames */
    memcpy(buf + n, gga, strlen(gga)); n += strlen(gga);

    casic_stream_init(&s, on_nmea, on_frame, &c);
    for (i = 0; i < n; i++)                               /* a byte at a time, as a UART delivers */
        casic_stream_feed(&s, buf + i, 1);
    CHECK(c.gga == 2 && c.nmea == 2);
    CHECK(c.acks == 2 && c.frames == 5);

    memset(&c, 0, sizeof c);
    casic_stream_init(&s, on_nmea, on_frame, &c);
    casic_stream_feed(&s, buf, n);                        /* and all at once */
    CHECK(c.gga == 2 && c.acks == 2 && c.frames == 5);
}

int main(void)
{
    test_frames();
    test_gps_time_and_aid();
    test_nmea();
    test_agnss_file();
    test_stream();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
