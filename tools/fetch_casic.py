#!/usr/bin/env python3
"""Fetch CASIC's AGNSS file (as republished by Espruino), check it, and write the raw CASIC frames.

    python3 tools/fetch_casic.py casic.bin

The file is base64: a short text header ("AGNSS data from CASIC.", "DataLength: N.", a download
counter) and then CASIC binary frames, typically 32 MSG-GPSEPH (0x08 0x07) and one MSG-GPSION
(0x08 0x06), sometimes MSG-GPSUTC (0x08 0x05). GPS only. Espruino refreshes it every few hours;
it is good for a few hours. Write the frames to the receiver after AID-INI, a second after boot.
"""
import base64, struct, sys, urllib.request

URL = "https://www.espruino.com/agps/casic.base64"

def checksum(cls, mid, payload):
    # Quectel L76K protocol spec 3.1: (ID << 24) + (Class << 16) + Len, then the payload as U4 words.
    # (CASIC's own spec prints the class and ID the other way round; the frames do not balance that way.)
    ck = (mid << 24) + (cls << 16) + len(payload)
    for i in range(0, len(payload) - len(payload) % 4, 4):
        ck += struct.unpack("<I", payload[i:i + 4])[0]
    return ck & 0xFFFFFFFF

def main(out):
    raw = base64.b64decode(urllib.request.urlopen(URL, timeout=30).read())
    start = raw.index(b"\xba\xce")
    print(raw[:start].decode(errors="replace").strip())
    frames, j, kinds = raw[start:], 0, {}
    while j + 6 <= len(frames):
        ln = frames[j + 2] | frames[j + 3] << 8
        f = frames[j:j + 6 + ln + 4]
        if struct.unpack("<I", f[-4:])[0] != checksum(f[4], f[5], f[6:-4]):
            sys.exit(f"bad checksum in frame at offset {j}")
        kinds[(f[4], f[5])] = kinds.get((f[4], f[5]), 0) + 1
        j += len(f)
    open(out, "wb").write(frames)
    print(f"{len(frames)} bytes, frames by (class, id): {kinds} -> {out}")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "casic.bin")
