/*
 * The assisted cold start, written against four platform hooks so it ports anywhere. Fill in the
 * hooks for your board (UART at 9600 baud to the receiver, a millisecond clock, a sleep). With
 * -DCASIC_EXAMPLE_HOST it builds on a desktop with stub hooks that only print what would be sent.
 *
 *   1. after the receiver boots or restarts, wait about a second (AID-INI sent earlier is ignored);
 *   2. send AID-INI: the time, and a position roughly right (the last fix, or the phone's);
 *   3. send CASIC's AGNSS file, frame by frame, counting ACKs;
 *   4. wait for a fix. Measured on an L76K: ~6 s, against ~40 s unassisted.
 *
 * MIT licence.
 */
#include "casic_agnss.h"

#include <string.h>

/* ---- platform hooks: provide these ---- */
void plat_uart_write(const uint8_t *data, size_t len);
size_t plat_uart_read(uint8_t *data, size_t cap);     /* non-blocking; returns bytes read */
uint32_t plat_millis(void);
void plat_sleep_ms(uint32_t ms);

/* ---- state gathered from the receiver's output ---- */
typedef struct {
    casic_nmea_fix_t fix;
    unsigned acks, nacks, lines;
} rx_state_t;

static void on_nmea(void *ctx, const char *s, size_t n)
{
    rx_state_t *rx = ctx;
    rx->lines++;
    (void)casic_nmea_parse(s, n, &rx->fix);
}

static void on_frame(void *ctx, const casic_frame_t *f)
{
    rx_state_t *rx = ctx;
    int r = casic_ack(f, NULL, NULL);
    if (r == 1) rx->acks++;
    else if (r == 0) rx->nacks++;
}

static casic_stream_t stream;

static void pump(rx_state_t *rx, uint32_t ms)
{
    uint32_t t0 = plat_millis();
    uint8_t b[64];
    (void)rx;
    do {
        size_t n = plat_uart_read(b, sizeof b);
        if (n) casic_stream_feed(&stream, b, n);
        else plat_sleep_ms(2);
    } while ((uint32_t)(plat_millis() - t0) < ms);
}

/*
 * Assist the receiver right after it has booted or been restarted. unix_now: the current UTC time
 * (from the phone, an RTC, or the receiver's own last RMC). lat/lon/height: a position good to
 * within pos_acc_m. agnss/agnss_len: CASIC's AGNSS file, base64-decoded. Returns the number of
 * AGNSS frames the receiver acknowledged.
 */
unsigned casic_assist(rx_state_t *rx, int64_t unix_now, double lat, double lon, double height,
                      float pos_acc_m, const uint8_t *agnss, size_t agnss_len)
{
    uint8_t frame[80];
    casic_aid_ini_t aid;
    casic_frame_t f;
    size_t n, off;
    long first;
    unsigned acks0;

    pump(rx, 1000);                                   /* 1: let it settle */

    memset(&aid, 0, sizeof aid);                      /* 2: time and position */
    casic_gps_time_from_unix(unix_now, CASIC_GPS_LEAP_SECONDS, &aid.week, &aid.tow_s);
    aid.lat_deg = lat;
    aid.lon_deg = lon;
    aid.height_m = height;
    aid.pos_acc_m = pos_acc_m;
    aid.time_acc_s = 2.0f;
    aid.flags = CASIC_AID_POSITION | CASIC_AID_TIME;
    n = casic_aid_ini_frame(&aid, frame, sizeof frame);
    plat_uart_write(frame, n);
    pump(rx, 300);

    acks0 = rx->acks;                                 /* 3: orbit data, paced */
    first = casic_agnss_first_frame(agnss, agnss_len);
    if (first < 0)
        return 0;
    off = (size_t)first;
    while (casic_frame_next(agnss, agnss_len, &off, &f) == 1) {
        plat_uart_write(f.frame, f.frame_len);
        pump(rx, 25);
    }
    pump(rx, 600);
    return rx->acks - acks0;
}

/* Cold-restart the receiver, assist it, and return the seconds to a fix (0 if none in timeout_s). */
unsigned casic_assisted_cold_start(int64_t unix_now, double lat, double lon, double height,
                                   const uint8_t *agnss, size_t agnss_len, unsigned timeout_s)
{
    static rx_state_t rx;
    char cmd[24];
    uint32_t t0;

    memset(&rx, 0, sizeof rx);
    casic_stream_init(&stream, on_nmea, on_frame, &rx);
    plat_uart_write((const uint8_t *)cmd, casic_nmea_command("PCAS10,2", cmd, sizeof cmd));
    t0 = plat_millis();
    casic_assist(&rx, unix_now, lat, lon, height, 100.0f, agnss, agnss_len);

    rx.fix.fix_quality = 0;                           /* 4: wait for a fix */
    while ((uint32_t)(plat_millis() - t0) < timeout_s * 1000u) {
        pump(&rx, 50);
        if (rx.fix.fix_quality > 0 && rx.fix.satellites >= 4)
            return (unsigned)((plat_millis() - t0 + 999u) / 1000u);
    }
    return 0;
}

#ifdef CASIC_EXAMPLE_HOST
/* Desktop stubs: print what would go to the receiver; nothing ever comes back. */
#include <stdio.h>
#include <time.h>

static uint32_t fake_ms;
void plat_uart_write(const uint8_t *data, size_t len)
{
    printf("-> %zu bytes%s\n", len, (len && data[0] == '$') ? " (NMEA)" : " (CASIC)");
}
size_t plat_uart_read(uint8_t *data, size_t cap) { (void)data; (void)cap; return 0; }
uint32_t plat_millis(void) { return fake_ms; }
void plat_sleep_ms(uint32_t ms) { fake_ms += ms; }

int main(void)
{
    /* A one-frame stand-in for CASIC's file; fetch the real one with tools/fetch_casic.py. */
    uint8_t agnss[100], eph[72] = {0};
    size_t n = casic_frame(CASIC_CLS_MSG, CASIC_ID_MSG_GPSEPH, eph, sizeof eph, agnss, sizeof agnss);
    unsigned s = casic_assisted_cold_start((int64_t)time(NULL), 0.0, 0.0, 0.0, agnss, n, 2);
    printf("fix after %u s (always 0 here: the stubs never answer)\n", s);
    return 0;
}
#endif
