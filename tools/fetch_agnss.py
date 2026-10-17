#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Fetch GPS orbit data for an AT6558 / L76K and write it as raw CASIC frames, ready to send.

    python3 tools/fetch_agnss.py casic.bin                  # CASIC first, IGS if that fails
    python3 tools/fetch_agnss.py --source igs casic.bin     # IGS broadcast ephemeris only
    python3 tools/fetch_agnss.py --rinex BRDC.rnx casic.bin # any RINEX 3 navigation file or URL

Sources, tried in the order given (default: casic, then igs):

  casic  CASIC's AGNSS file as republished by Espruino: 32 MSG-GPSEPH and a MSG-GPSION, already
         framed. Small (~3.6 kB of base64). One mirror, run by a hobby project.
  igs    The IGS merged broadcast ephemeris (RINEX 3, all constellations) from BKG's data centre,
         anonymous HTTPS, refreshed every 15 minutes from the IGS real-time network. ~1.3 MB
         gzipped per day. Each GPS satellite's newest ephemeris is encoded as MSG-GPSEPH. The
         same files are at NASA CDDIS (needs an Earthdata login) and IGN; pass one with --rinex.

Both carry the satellites' own broadcast data, so they agree: checked field by field, every orbit
and clock term of CASIC's file equals the IGS record with the same toe and IODC, and the frames
come out byte for byte the same apart from the URA index, where the two sources sometimes disagree.
The "reserved" first word is the sum of the payload's other words, filled in here as CASIC does. No ionosphere message is written for igs unless the RINEX header carries one.
"""
import argparse, base64, datetime as dt, gzip, io, math, struct, sys, urllib.request

CASIC_URL = "https://www.espruino.com/agps/casic.base64"
IGS_URL = "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/{y}/{doy:03d}/BRDC00WRD_S_{y}{doy:03d}0000_01D_MN.rnx.gz"
GPS_EPOCH = dt.datetime(1980, 1, 6, tzinfo=dt.timezone.utc)
LEAP = 18
PI = 3.1415926535898                       # IS-GPS-200's pi, which semicircles are defined with
FIT_S = 2 * 3600                           # an ephemeris is good for about 2 h either side of toe


def checksum(cls, mid, payload):
    ck = (mid << 24) + (cls << 16) + len(payload)
    for i in range(0, len(payload) - len(payload) % 4, 4):
        ck += struct.unpack("<I", payload[i:i + 4])[0]
    return ck & 0xFFFFFFFF


def word_sum(payload):
    """MSG-GPSEPH and MSG-GPSION's "reserved" first word: the sum of the payload's other U4 words."""
    return sum(struct.unpack_from("<%dI" % (len(payload) // 4 - 1), payload, 4)) & 0xFFFFFFFF


def with_word_sum(payload):
    return struct.pack("<I", word_sum(payload)) + payload[4:]


def frame(cls, mid, payload):
    return struct.pack("<BBHBB", 0xBA, 0xCE, len(payload), cls, mid) + payload + \
        struct.pack("<I", checksum(cls, mid, payload))


def split_frames(buf):
    """Check every frame's checksum; return {(class, id): count}."""
    j, kinds = 0, {}
    while j + 6 <= len(buf):
        ln = buf[j + 2] | buf[j + 3] << 8
        f = buf[j:j + 10 + ln]
        if f[:2] != b"\xba\xce" or struct.unpack("<I", f[-4:])[0] != checksum(f[4], f[5], f[6:-4]):
            raise ValueError(f"bad frame at offset {j}")
        kinds[(f[4], f[5])] = kinds.get((f[4], f[5]), 0) + 1
        j += len(f)
    return kinds


# ---------------------------------------------------------------- casic

def from_casic(now):
    raw = base64.b64decode(urllib.request.urlopen(CASIC_URL, timeout=30).read())
    start = raw.index(b"\xba\xce")
    print(raw[:start].decode(errors="replace").strip(), file=sys.stderr)
    return raw[start:]


# ---------------------------------------------------------------- igs (RINEX 3)

def ura_index(m):
    for n, top in enumerate((2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24, 48, 96, 192, 384, 768, 1536, 3072, 6144)):
        if m <= top:
            return n
    return 15


def q(v, lsb, fmt):
    r = round(v / lsb)
    struct.pack("<" + fmt, r)              # raises struct.error if out of range
    return r


def gpseph_payload(r, sv):
    s = 1 / PI
    healthy = int(r["health"]) == 0
    return with_word_sum(struct.pack(
        "<IIIiiiiihhhhhhhhHHIihbbHBBBBH",
        0,
        q(r["sqrtA"], 2**-19, "I"), q(r["e"], 2**-33, "I"),
        q(r["omega"] * s, 2**-31, "i"), q(r["M0"] * s, 2**-31, "i"), q(r["i0"] * s, 2**-31, "i"),
        q(r["Omega0"] * s, 2**-31, "i"), q(r["OmegaDot"] * s, 2**-43, "i"),
        q(r["deltaN"] * s, 2**-43, "h"), q(r["IDOT"] * s, 2**-43, "h"),
        q(r["Cuc"], 2**-29, "h"), q(r["Cus"], 2**-29, "h"), q(r["Crc"], 2**-5, "h"),
        q(r["Crs"], 2**-5, "h"), q(r["Cic"], 2**-29, "h"), q(r["Cis"], 2**-29, "h"),
        q(r["toe"], 16, "H"), int(r["week"]) % 1024, q(r["toc"], 16, "I"),
        q(r["af0"], 2**-31, "i"), q(r["af1"], 2**-43, "h"), q(r["af2"], 2**-55, "b"),
        q(r["TGD"], 2**-31, "b"), int(r["IODC"]), ura_index(r["svacc"]), int(r["health"]),
        sv, 3 if healthy else 1, 0))


NAMES = ["af0", "af1", "af2", "IODE", "Crs", "deltaN", "M0", "Cuc", "e", "Cus", "sqrtA", "toe", "Cic",
         "Omega0", "Cis", "i0", "Crc", "omega", "OmegaDot", "IDOT", "L2codes", "week", "L2P", "svacc",
         "health", "TGD", "IODC"]


def num(s):
    s = s.strip().replace("D", "E")
    return float(s) if s else 0.0


def parse_rinex(text):
    """GPS LNAV records from a RINEX 3 navigation file, and the GPSA/GPSB ionosphere header lines."""
    lines = text.splitlines()
    k = next(n for n, l in enumerate(lines) if "END OF HEADER" in l) + 1
    ion = {}
    for l in lines[:k]:
        if "IONOSPHERIC CORR" in l and l[:4] in ("GPSA", "GPSB"):
            ion[l[:4]] = [num(l[5 + 12 * j:17 + 12 * j]) for j in range(4)]
    recs = []
    while k < len(lines):
        l = lines[k]
        if not l.startswith("G") or k + 7 >= len(lines):
            k += 1
            continue
        vals = [num(l[23 + 19 * j:42 + 19 * j]) for j in range(3)]
        for m in range(1, 8):
            b = lines[k + m]
            vals += [num(b[4 + 19 * j:23 + 19 * j]) for j in range(4)]
        r = dict(zip(NAMES, vals))
        r["sv"] = int(l[1:3])
        t = dt.datetime(*map(int, l[4:23].split()), tzinfo=dt.timezone.utc)
        r["toc"] = (t - GPS_EPOCH).total_seconds() % (7 * 86400)
        recs.append(r)
        k += 8
    return recs, ion


def encode_rinex(text, now):
    recs, ion = parse_rinex(text)
    gps_now = (now - GPS_EPOCH).total_seconds() + LEAP
    best = {}
    for r in recs:
        # The record whose toe is nearest now: the one the satellite is most likely broadcasting,
        # and the most accurate. (Merged files often lack a usable transmission time.)
        r["age"] = abs(gps_now - (r["week"] * 7 * 86400 + r["toe"]))
        if r["age"] <= FIT_S and (r["sv"] not in best or r["age"] < best[r["sv"]]["age"]):
            best[r["sv"]] = r
    if not best:
        raise ValueError("no GPS ephemeris in the file is within 2 h of now")
    out = b"".join(frame(0x08, 0x07, gpseph_payload(best[sv], sv)) for sv in sorted(best))
    if "GPSA" in ion and "GPSB" in ion:
        a, b = ion["GPSA"], ion["GPSB"]
        out += frame(0x08, 0x06, with_word_sum(struct.pack(
            "<I8bBBH", 0,
            *(q(v, s, "b") for v, s in zip(a, (2**-30, 2**-27, 2**-24, 2**-24))),
            *(q(v, s, "b") for v, s in zip(b, (2**11, 2**14, 2**16, 2**16))), 3, 0, 0)))
    return out


def read_rinex(where):
    if "://" in where:
        data = urllib.request.urlopen(where, timeout=60).read()
    else:
        data = open(where, "rb").read()
    if data[:2] == b"\x1f\x8b":
        data = gzip.decompress(data)
    return data.decode("ascii", errors="replace")


def from_igs(now):
    # Today's file (UTC), or yesterday's in the first hours of a day before today's has filled in.
    for day in (now, now - dt.timedelta(days=1)):
        url = IGS_URL.format(y=day.year, doy=day.timetuple().tm_yday)
        try:
            return encode_rinex(read_rinex(url), now)
        except Exception as e:             # noqa: BLE001 - try the next day, then give up
            print(f"igs: {url}: {e}", file=sys.stderr)
    raise RuntimeError("no usable IGS file")


SOURCES = {"casic": from_casic, "igs": from_igs}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out", nargs="?", default="casic.bin")
    ap.add_argument("--source", action="append", choices=sorted(SOURCES),
                    help="try this source (repeat for a fallback order; default: casic, then igs)")
    ap.add_argument("--rinex", help="encode this RINEX 3 navigation file (path or URL) instead")
    ap.add_argument("--at", help="encode for this UTC time (ISO 8601) instead of now")
    a = ap.parse_args()
    now = dt.datetime.fromisoformat(a.at).replace(tzinfo=dt.timezone.utc) if a.at else \
        dt.datetime.now(dt.timezone.utc)

    if a.rinex:
        frames, used = encode_rinex(read_rinex(a.rinex), now), "rinex"
    else:
        frames = used = None
        for name in a.source or ["casic", "igs"]:
            try:
                frames, used = SOURCES[name](now), name
                break
            except Exception as e:         # noqa: BLE001 - fall through to the next source
                print(f"{name}: failed: {e}", file=sys.stderr)
        if frames is None:
            sys.exit("every source failed")
    kinds = split_frames(frames)
    open(a.out, "wb").write(frames)
    print(f"{used}: {len(frames)} bytes, frames by (class, id): {kinds} -> {a.out}")


if __name__ == "__main__":
    main()
