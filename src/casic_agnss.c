/* SPDX-License-Identifier: MIT */
/* casic_agnss: see include/casic_agnss.h. MIT licence. */
#include "casic_agnss.h"

#include <string.h>

/* ------------------------------------------------------------------ little-endian helpers */

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void wrf32(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof u);
    wr32(p, u);
}
static void wrf64(uint8_t *p, double v)
{
    uint64_t u;
    memcpy(&u, &v, sizeof u);
    wr32(p, (uint32_t)u);
    wr32(p + 4, (uint32_t)(u >> 32));
}

/* ------------------------------------------------------------------ frames */

uint32_t casic_checksum(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint32_t ck = ((uint32_t)id << 24) + ((uint32_t)cls << 16) + len;
    uint16_t i;
    for (i = 0; i + 4 <= len; i += 4)
        ck += rd32(payload + i);
    return ck;
}

size_t casic_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len,
                   uint8_t *out, size_t out_cap)
{
    size_t total = (size_t)len + CASIC_OVERHEAD;
    if (len > CASIC_MAX_PAYLOAD || out_cap < total || (len && !payload))
        return 0;
    out[0] = CASIC_SYNC1;
    out[1] = CASIC_SYNC2;
    wr16(out + 2, len);
    out[4] = cls;
    out[5] = id;
    if (len)
        memcpy(out + 6, payload, len);
    wr32(out + 6 + len, casic_checksum(cls, id, payload, len));
    return total;
}

int casic_frame_parse(const uint8_t *buf, size_t len, casic_frame_t *f)
{
    uint16_t plen;
    if (len < CASIC_OVERHEAD || buf[0] != CASIC_SYNC1 || buf[1] != CASIC_SYNC2)
        return 0;
    plen = rd16(buf + 2);
    if (plen > CASIC_MAX_PAYLOAD || len < (size_t)plen + CASIC_OVERHEAD)
        return 0;
    if (rd32(buf + 6 + plen) != casic_checksum(buf[4], buf[5], buf + 6, plen))
        return 0;
    f->cls = buf[4];
    f->id = buf[5];
    f->payload = buf + 6;
    f->len = plen;
    f->frame = buf;
    f->frame_len = (size_t)plen + CASIC_OVERHEAD;
    return 1;
}

int casic_frame_next(const uint8_t *buf, size_t len, size_t *offset, casic_frame_t *f)
{
    if (*offset >= len)
        return 0;
    if (!casic_frame_parse(buf + *offset, len - *offset, f))
        return -1;
    *offset += f->frame_len;
    return 1;
}

long casic_agnss_first_frame(const uint8_t *buf, size_t len)
{
    size_t i;
    casic_frame_t f;
    for (i = 0; i + 1 < len; i++)
        if (buf[i] == CASIC_SYNC1 && buf[i + 1] == CASIC_SYNC2 && casic_frame_parse(buf + i, len - i, &f))
            return (long)i;
    return -1;
}

int casic_ack(const casic_frame_t *f, uint8_t *acked_cls, uint8_t *acked_id)
{
    if (f->cls != CASIC_CLS_ACK || f->len < 2)
        return -1;
    if (acked_cls) *acked_cls = f->payload[0];
    if (acked_id) *acked_id = f->payload[1];
    if (f->id == CASIC_ID_ACK_ACK) return 1;
    if (f->id == CASIC_ID_ACK_NACK) return 0;
    return -1;
}

size_t casic_cfg_msg(uint8_t msg_cls, uint8_t msg_id, uint16_t rate, uint8_t *out, size_t out_cap)
{
    uint8_t p[4];
    p[0] = msg_cls;
    p[1] = msg_id;
    wr16(p + 2, rate);
    return casic_frame(CASIC_CLS_CFG, CASIC_ID_CFG_MSG, p, sizeof p, out, out_cap);
}

/* ------------------------------------------------------------------ AID-INI */

void casic_gps_time_from_unix(int64_t unix_seconds, int leap_seconds, uint16_t *week, double *tow)
{
    int64_t g = unix_seconds - 315964800 + leap_seconds;   /* 1980-01-06T00:00:00Z */
    if (g < 0)
        g = 0;
    *week = (uint16_t)(g / 604800);
    *tow = (double)(g % 604800);
}

void casic_aid_ini_payload(const casic_aid_ini_t *aid, uint8_t out[CASIC_AID_INI_LEN])
{
    uint8_t flags = aid->flags;
    if (flags & CASIC_AID_POSITION)
        flags |= CASIC_AID_LLA;
    memset(out, 0, CASIC_AID_INI_LEN);
    wrf64(out + 0, aid->lat_deg);           /* ecefXOrLat */
    wrf64(out + 8, aid->lon_deg);           /* ecefYOrLon */
    wrf64(out + 16, aid->height_m);         /* ecefZOrAlt */
    wrf64(out + 24, aid->tow_s);            /* GPS time of week, s */
    wrf32(out + 32, 0.0f);                  /* clock frequency drift */
    wrf32(out + 36, aid->pos_acc_m);        /* position accuracy, m */
    wrf32(out + 40, aid->time_acc_s);       /* time accuracy, s */
    wrf32(out + 44, 0.0f);                  /* drift accuracy */
    wr32(out + 48, 0);                      /* reserved */
    wr16(out + 52, aid->week);              /* GPS week */
    out[54] = 0;                            /* time source */
    out[55] = flags;
}

size_t casic_aid_ini_frame(const casic_aid_ini_t *aid, uint8_t *out, size_t out_cap)
{
    uint8_t p[CASIC_AID_INI_LEN];
    casic_aid_ini_payload(aid, p);
    return casic_frame(CASIC_CLS_AID, CASIC_ID_AID_INI, p, sizeof p, out, out_cap);
}

/* ------------------------------------------------------------------ MSG-GPSEPH */

uint32_t casic_msg_word_sum(const uint8_t *payload, uint16_t len)
{
    uint32_t sum = 0;
    uint16_t i;
    for (i = 4; i + 4 <= len; i += 4)
        sum += rd32(payload + i);
    return sum;
}

uint8_t casic_ura_index(double metres)
{
    static const double upper[15] = {2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24, 48, 96, 192, 384, 768,
                                     1536, 3072, 6144};
    uint8_t n;
    for (n = 0; n < 15; n++)
        if (metres <= upper[n]) return n;
    return 15;
}

/* Round v / lsb to the nearest integer; clear *ok if it falls outside [lo, hi]. */
static int64_t quant(double v, double lsb, int64_t lo, int64_t hi, int *ok)
{
    double q = v / lsb;
    int64_t r;
    if (!(q > (double)lo - 1.0 && q < (double)hi + 1.0)) { *ok = 0; return 0; }
    r = (int64_t)(q >= 0 ? q + 0.5 : q - 0.5);
    if (r < lo || r > hi) { *ok = 0; return 0; }
    return r;
}
#define Q32S(v, lsb) ((uint32_t)(int32_t)quant(v, lsb, INT32_MIN, INT32_MAX, &ok))
#define Q32U(v, lsb) ((uint32_t)quant(v, lsb, 0, UINT32_MAX, &ok))
#define Q16S(v, lsb) ((uint16_t)(int16_t)quant(v, lsb, INT16_MIN, INT16_MAX, &ok))
#define Q16U(v, lsb) ((uint16_t)quant(v, lsb, 0, UINT16_MAX, &ok))
#define Q8S(v, lsb) ((uint8_t)(int8_t)quant(v, lsb, INT8_MIN, INT8_MAX, &ok))

int casic_gpseph_payload(const casic_gps_eph_t *e, uint8_t out[CASIC_GPSEPH_LEN])
{
    const double sc = 1.0 / CASIC_GPS_PI;    /* radians to semicircles */
    int ok = 1;
    memset(out, 0, CASIC_GPSEPH_LEN);
    wr32(out + 4, Q32U(e->sqrt_a, 0x1p-19));
    wr32(out + 8, Q32U(e->e, 0x1p-33));
    wr32(out + 12, Q32S(e->omega * sc, 0x1p-31));
    wr32(out + 16, Q32S(e->m0 * sc, 0x1p-31));
    wr32(out + 20, Q32S(e->i0 * sc, 0x1p-31));
    wr32(out + 24, Q32S(e->omega0 * sc, 0x1p-31));
    wr32(out + 28, Q32S(e->omega_dot * sc, 0x1p-43));
    wr16(out + 32, Q16S(e->delta_n * sc, 0x1p-43));
    wr16(out + 34, Q16S(e->idot * sc, 0x1p-43));
    wr16(out + 36, Q16S(e->cuc, 0x1p-29));
    wr16(out + 38, Q16S(e->cus, 0x1p-29));
    wr16(out + 40, Q16S(e->crc, 0x1p-5));
    wr16(out + 42, Q16S(e->crs, 0x1p-5));
    wr16(out + 44, Q16S(e->cic, 0x1p-29));
    wr16(out + 46, Q16S(e->cis, 0x1p-29));
    wr16(out + 48, Q16U(e->toe, 16.0));
    wr16(out + 50, (uint16_t)(e->week % 1024u));
    wr32(out + 52, Q32U(e->toc, 16.0));
    wr32(out + 56, Q32S(e->af0, 0x1p-31));
    wr16(out + 60, Q16S(e->af1, 0x1p-43));
    out[62] = Q8S(e->af2, 0x1p-55);
    out[63] = Q8S(e->tgd, 0x1p-31);
    wr16(out + 64, e->iodc);
    out[66] = e->ura;
    out[67] = e->health;
    out[68] = e->svid;
    out[69] = e->valid;
    if (e->svid < 1 || e->svid > 32) ok = 0;
    wr32(out, casic_msg_word_sum(out, CASIC_GPSEPH_LEN));
    return ok;
}

size_t casic_gpseph_frame(const casic_gps_eph_t *eph, uint8_t *out, size_t out_cap)
{
    uint8_t p[CASIC_GPSEPH_LEN];
    if (!casic_gpseph_payload(eph, p)) return 0;
    return casic_frame(CASIC_CLS_MSG, CASIC_ID_MSG_GPSEPH, p, sizeof p, out, out_cap);
}

static double s32(const uint8_t *p) { return (double)(int32_t)rd32(p); }
static double s16(const uint8_t *p) { return (double)(int16_t)rd16(p); }

int casic_gpseph_decode(const uint8_t *p, uint16_t len, casic_gps_eph_t *e)
{
    const double pi = CASIC_GPS_PI;
    if (len != CASIC_GPSEPH_LEN) return 0;
    e->sqrt_a = rd32(p + 4) * 0x1p-19;
    e->e = rd32(p + 8) * 0x1p-33;
    e->omega = s32(p + 12) * 0x1p-31 * pi;
    e->m0 = s32(p + 16) * 0x1p-31 * pi;
    e->i0 = s32(p + 20) * 0x1p-31 * pi;
    e->omega0 = s32(p + 24) * 0x1p-31 * pi;
    e->omega_dot = s32(p + 28) * 0x1p-43 * pi;
    e->delta_n = s16(p + 32) * 0x1p-43 * pi;
    e->idot = s16(p + 34) * 0x1p-43 * pi;
    e->cuc = s16(p + 36) * 0x1p-29;
    e->cus = s16(p + 38) * 0x1p-29;
    e->crc = s16(p + 40) * 0x1p-5;
    e->crs = s16(p + 42) * 0x1p-5;
    e->cic = s16(p + 44) * 0x1p-29;
    e->cis = s16(p + 46) * 0x1p-29;
    e->toe = rd16(p + 48) * 16.0;
    e->week = rd16(p + 50);
    e->toc = rd32(p + 52) * 16.0;
    e->af0 = s32(p + 56) * 0x1p-31;
    e->af1 = s16(p + 60) * 0x1p-43;
    e->af2 = (int8_t)p[62] * 0x1p-55;
    e->tgd = (int8_t)p[63] * 0x1p-31;
    e->iodc = rd16(p + 64);
    e->ura = p[66];
    e->health = p[67];
    e->svid = p[68];
    e->valid = p[69];
    return 1;
}

/* ------------------------------------------------------------------ MSG-BDSEPH */

int casic_bdseph_payload(const casic_bds_eph_t *e, uint8_t out[CASIC_BDSEPH_LEN])
{
    const double sc = 1.0 / CASIC_GPS_PI;    /* BeiDou defines semicircles with the same pi */
    int ok = 1;
    memset(out, 0, CASIC_BDSEPH_LEN);
    wr32(out + 4, Q32U(e->sqrt_a, 0x1p-19));
    wr32(out + 8, Q32U(e->e, 0x1p-33));
    wr32(out + 12, Q32S(e->omega * sc, 0x1p-31));
    wr32(out + 16, Q32S(e->m0 * sc, 0x1p-31));
    wr32(out + 20, Q32S(e->i0 * sc, 0x1p-31));
    wr32(out + 24, Q32S(e->omega0 * sc, 0x1p-31));
    wr32(out + 28, Q32S(e->omega_dot * sc, 0x1p-43));
    wr16(out + 32, Q16S(e->delta_n * sc, 0x1p-43));
    wr16(out + 34, Q16S(e->idot * sc, 0x1p-43));
    wr32(out + 36, Q32S(e->cuc, 0x1p-31));
    wr32(out + 40, Q32S(e->cus, 0x1p-31));
    wr32(out + 44, Q32S(e->crc, 0x1p-6));
    wr32(out + 48, Q32S(e->crs, 0x1p-6));
    wr32(out + 52, Q32S(e->cic, 0x1p-31));
    wr32(out + 56, Q32S(e->cis, 0x1p-31));
    wr32(out + 60, Q32U(e->toe, 8.0));
    wr16(out + 64, e->week);
    wr32(out + 68, Q32U(e->toc, 8.0));
    wr32(out + 72, Q32S(e->af0, 0x1p-33));
    wr32(out + 76, Q32S(e->af1, 0x1p-50));
    wr16(out + 80, Q16S(e->af2, 0x1p-66));
    wr16(out + 82, Q16S(e->tgd1, 1e-10));
    out[84] = e->aodc;
    out[85] = e->aode;
    out[86] = e->ura;
    out[87] = e->health;
    out[88] = e->svid;
    out[89] = e->valid;
    if (e->svid < 1 || e->svid > CASIC_BDS_MAX_SVID) ok = 0;
    wr32(out, casic_msg_word_sum(out, CASIC_BDSEPH_LEN));
    return ok;
}

size_t casic_bdseph_frame(const casic_bds_eph_t *eph, uint8_t *out, size_t out_cap)
{
    uint8_t p[CASIC_BDSEPH_LEN];
    if (!casic_bdseph_payload(eph, p)) return 0;
    return casic_frame(CASIC_CLS_MSG, CASIC_ID_MSG_BDSEPH, p, sizeof p, out, out_cap);
}

int casic_bdseph_decode(const uint8_t *p, uint16_t len, casic_bds_eph_t *e)
{
    const double pi = CASIC_GPS_PI;
    if (len != CASIC_BDSEPH_LEN) return 0;
    e->sqrt_a = rd32(p + 4) * 0x1p-19;
    e->e = rd32(p + 8) * 0x1p-33;
    e->omega = s32(p + 12) * 0x1p-31 * pi;
    e->m0 = s32(p + 16) * 0x1p-31 * pi;
    e->i0 = s32(p + 20) * 0x1p-31 * pi;
    e->omega0 = s32(p + 24) * 0x1p-31 * pi;
    e->omega_dot = s32(p + 28) * 0x1p-43 * pi;
    e->delta_n = s16(p + 32) * 0x1p-43 * pi;
    e->idot = s16(p + 34) * 0x1p-43 * pi;
    e->cuc = s32(p + 36) * 0x1p-31;
    e->cus = s32(p + 40) * 0x1p-31;
    e->crc = s32(p + 44) * 0x1p-6;
    e->crs = s32(p + 48) * 0x1p-6;
    e->cic = s32(p + 52) * 0x1p-31;
    e->cis = s32(p + 56) * 0x1p-31;
    e->toe = rd32(p + 60) * 8.0;
    e->week = rd16(p + 64);
    e->toc = rd32(p + 68) * 8.0;
    e->af0 = s32(p + 72) * 0x1p-33;
    e->af1 = s32(p + 76) * 0x1p-50;
    e->af2 = s16(p + 80) * 0x1p-66;
    e->tgd1 = s16(p + 82) * 1e-10;
    e->aodc = p[84];
    e->aode = p[85];
    e->ura = p[86];
    e->health = p[87];
    e->svid = p[88];
    e->valid = p[89];
    return 1;
}

/* ------------------------------------------------------------------ NMEA */

static const char HEX[] = "0123456789ABCDEF";

size_t casic_nmea_command(const char *body, char *out, size_t out_cap)
{
    size_t n = strlen(body), i;
    uint8_t c = 0;
    if (out_cap < n + 7)                   /* '$' body '*' hh CR LF NUL */
        return 0;
    out[0] = '$';
    for (i = 0; i < n; i++) {
        out[1 + i] = body[i];
        c ^= (uint8_t)body[i];
    }
    out[1 + n] = '*';
    out[2 + n] = HEX[c >> 4];
    out[3 + n] = HEX[c & 15];
    out[4 + n] = '\r';
    out[5 + n] = '\n';
    out[6 + n] = '\0';
    return n + 6;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* A small, locale-free decimal parser for NMEA fields ("[-]digits[.digits]"). */
static int parse_num(const char *s, size_t n, double *out)
{
    double v = 0, scale = 1;
    int neg = 0, digits = 0, frac = 0;
    size_t i = 0;
    if (n && (s[0] == '-' || s[0] == '+')) { neg = s[0] == '-'; i = 1; }
    for (; i < n; i++) {
        if (s[i] == '.' && !frac) { frac = 1; continue; }
        if (s[i] < '0' || s[i] > '9') return 0;
        digits++;
        if (frac) { scale /= 10; v += (s[i] - '0') * scale; }
        else v = v * 10 + (s[i] - '0');
    }
    if (!digits) return 0;
    *out = neg ? -v : v;
    return 1;
}

static int parse_2(const char *s) { return (s[0] - '0') * 10 + (s[1] - '0'); }

/* ddmm.mmmm (lat) or dddmm.mmmm (lon) and a hemisphere letter, to signed degrees. */
static int parse_coord(const char *s, size_t n, char hemi, int deg_digits, double *out)
{
    double minutes, deg = 0;
    int i;
    if ((int)n < deg_digits + 2) return 0;
    for (i = 0; i < deg_digits; i++) {
        if (s[i] < '0' || s[i] > '9') return 0;
        deg = deg * 10 + (s[i] - '0');
    }
    if (!parse_num(s + deg_digits, n - (size_t)deg_digits, &minutes)) return 0;
    deg += minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') deg = -deg;
    else if (hemi != 'N' && hemi != 'E') return 0;
    *out = deg;
    return 1;
}

#define NMEA_MAX_FIELDS 24

int casic_nmea_parse(const char *sentence, size_t len, casic_nmea_fix_t *fix)
{
    const char *f[NMEA_MAX_FIELDS];
    size_t fl[NMEA_MAX_FIELDS];
    size_t i, start, star;
    int nf = 0;
    uint8_t c = 0;

    while (len && (sentence[len - 1] == '\r' || sentence[len - 1] == '\n'))
        len--;
    if (len && sentence[0] == '$') { sentence++; len--; }
    for (star = 0; star < len && sentence[star] != '*'; star++)
        c ^= (uint8_t)sentence[star];
    if (star + 3 != len || hexval(sentence[star + 1]) < 0 || hexval(sentence[star + 2]) < 0)
        return -1;
    if (c != (uint8_t)(hexval(sentence[star + 1]) << 4 | hexval(sentence[star + 2])))
        return -1;

    for (i = 0, start = 0; i <= star && nf < NMEA_MAX_FIELDS; i++) {
        if (i == star || sentence[i] == ',') {
            f[nf] = sentence + start;
            fl[nf] = i - start;
            nf++;
            start = i + 1;
        }
    }
    if (nf < 1 || fl[0] < 5)
        return 0;

    if (memcmp(f[0] + fl[0] - 3, "GGA", 3) == 0 && nf >= 10) {
        double q = 0, sats = 0, lat, lon, h = 0;
        parse_num(f[6], fl[6], &q);
        parse_num(f[7], fl[7], &sats);
        fix->fix_quality = (int)q;
        fix->satellites = (int)sats;
        if (fix->fix_quality > 0 && fl[3] == 1 && fl[5] == 1 &&
            parse_coord(f[2], fl[2], f[3][0], 2, &lat) && parse_coord(f[4], fl[4], f[5][0], 3, &lon)) {
            parse_num(f[9], fl[9], &h);
            fix->lat_deg = lat;
            fix->lon_deg = lon;
            fix->height_m = h;
            fix->have_position = 1;
        }
        return 1;
    }
    if (memcmp(f[0] + fl[0] - 3, "RMC", 3) == 0 && nf >= 10) {
        if (fl[2] == 1 && f[2][0] == 'A' && fl[1] >= 6 && fl[9] == 6) {
            double sec;
            if (!parse_num(f[1] + 4, fl[1] - 4, &sec))
                return 2;
            fix->hour = parse_2(f[1]);
            fix->minute = parse_2(f[1] + 2);
            fix->second = sec;
            fix->day = parse_2(f[9]);
            fix->month = parse_2(f[9] + 2);
            fix->year = 2000 + parse_2(f[9] + 4);
            fix->have_time = 1;
        }
        return 2;
    }
    return 0;
}

static int64_t days_from_civil(int y, int m, int d)
{
    int64_t era, yoe, doy, doe;
    y -= m <= 2;
    era = (y >= 0 ? y : y - 399) / 400;
    yoe = y - era * 400;
    doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

int64_t casic_nmea_unix_time(const casic_nmea_fix_t *fix)
{
    if (!fix->have_time)
        return 0;
    return days_from_civil(fix->year, fix->month, fix->day) * 86400 +
           fix->hour * 3600 + fix->minute * 60 + (int64_t)fix->second;
}

/* ------------------------------------------------------------------ the byte stream */

void casic_stream_init(casic_stream_t *s, casic_on_nmea on_nmea, casic_on_frame on_frame, void *ctx)
{
    s->n = 0;
    s->on_nmea = on_nmea;
    s->on_frame = on_frame;
    s->ctx = ctx;
}

static void drop(casic_stream_t *s, size_t k)
{
    if (k >= s->n) { s->n = 0; return; }
    memmove(s->buf, s->buf + k, s->n - k);
    s->n -= k;
}

/* Consume whatever whole items the buffer holds. */
static void drain(casic_stream_t *s)
{
    for (;;) {
        size_t i;
        if (s->n == 0)
            return;
        if (s->buf[0] == CASIC_SYNC1) {
            uint16_t plen;
            casic_frame_t f;
            if (s->n < 2) return;
            if (s->buf[1] != CASIC_SYNC2) { drop(s, 1); continue; }
            if (s->n < 4) return;
            plen = rd16(s->buf + 2);
            if (plen > CASIC_MAX_PAYLOAD) { drop(s, 2); continue; }
            if (s->n < (size_t)plen + CASIC_OVERHEAD) return;
            if (casic_frame_parse(s->buf, s->n, &f)) {
                if (s->on_frame) s->on_frame(s->ctx, &f);
                drop(s, f.frame_len);
            } else {
                drop(s, 2);                 /* bad checksum: resynchronise */
            }
            continue;
        }
        if (s->buf[0] == '$') {
            for (i = 1; i < s->n; i++) {
                if (s->buf[i] == '\n') break;
                if (s->buf[i] == '$' || s->buf[i] == CASIC_SYNC1) { i = 0; break; }
            }
            if (i == 0) { drop(s, 1); continue; }   /* a sentence cut short by another item */
            if (i == s->n) {
                if (s->n == sizeof s->buf) s->n = 0;
                return;
            }
            if (s->on_nmea) s->on_nmea(s->ctx, (const char *)s->buf, i + 1);
            drop(s, i + 1);
            continue;
        }
        for (i = 1; i < s->n && s->buf[i] != '$' && s->buf[i] != CASIC_SYNC1; i++)
            ;
        drop(s, i);                         /* noise before the next item */
    }
}

void casic_stream_feed(casic_stream_t *s, const uint8_t *data, size_t len)
{
    while (len) {
        size_t room = sizeof s->buf - s->n, k = len < room ? len : room;
        memcpy(s->buf + s->n, data, k);
        s->n += k;
        data += k;
        len -= k;
        drain(s);
        if (s->n == sizeof s->buf)
            drop(s, 1);                     /* full with no whole item: give up on the oldest byte */
    }
}
