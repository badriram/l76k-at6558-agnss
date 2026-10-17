# Assisted cold starts on a Quectel L76K (CASIC / AT6558), on a Heltec WiFi LoRa 32 V4 (MicroPython).
# Takes a first fix, then cold-restarts the receiver ($PCAS10,2) again and again: with no help, with
# AID-INI (time and position from that first fix, held in memory only), with CASIC's AGNSS orbit data
# (/casic.bin, from tools/fetch_casic.py), and with both. Logs to /gnss_assist_log.txt. The white LED
# blinks while searching and stays on when done. Pins from Meshtastic's heltec_v4 variant: EN 34
# (low = on), STANDBY 40 (high = awake), RESET 42 (low = reset), PPS 41, UART TX 38 / RX 39, 9600 baud.
# MIT licence; see LICENSE.
from machine import Pin, UART, RTC
import time, struct

EN, STBY, RST = Pin(34, Pin.OUT, value=0), Pin(40, Pin.OUT, value=1), Pin(42, Pin.OUT, value=1)
LED = Pin(35, Pin.OUT, value=0)
U = UART(1, baudrate=9600, tx=38, rx=39, rxbuf=8192)
LOG = "/gnss_assist_log.txt"
OFF_S = 60          # how long each sleep or power cut lasts
FIX_TIMEOUT = 300
REPEAT = 3

def log(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    with open(LOG, "a") as f:
        f.write(s + "\n")

# ---------------------------------------------------------------- CASIC framing
def checksum(cls, mid, payload):
    # Quectel L76K spec 3.1: (ID << 24) + (Class << 16) + Len, then the payload as U4 words
    ck = (mid << 24) + (cls << 16) + len(payload)
    for i in range(0, len(payload) - len(payload) % 4, 4):
        ck += struct.unpack("<I", payload[i:i + 4])[0]
    return ck & 0xFFFFFFFF

def frame(cls, mid, payload=b""):
    return b"\xba\xce" + struct.pack("<HBB", len(payload), cls, mid) + payload + \
        struct.pack("<I", checksum(cls, mid, payload))

def nmea(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return ("$%s*%02X\r\n" % (body, c)).encode()

# ---------------------------------------------------------------- receiver state
class Rx:
    def __init__(self):
        self.buf = b""; self.fix = False; self.sats = 0; self.lat = None; self.lon = None
        self.alt = 0.0; self.lines = 0; self.acks = 0; self.nacks = 0; self.frames = []
        self.capture = False; self.date = None; self.last_byte = time.ticks_ms()
        self.gga_q = 0; self.gga_n = 0; self.last_ack = None

    def pump(self):
        d = U.read()
        if not d:
            return
        self.last_byte = time.ticks_ms()
        self.buf += d
        while True:
            i_n, i_c = self.buf.find(b"$"), self.buf.find(b"\xba\xce")
            if i_c != -1 and (i_n == -1 or i_c < i_n):
                if len(self.buf) < i_c + 6:
                    break
                ln = self.buf[i_c + 2] | self.buf[i_c + 3] << 8
                end = i_c + 6 + ln + 4
                if ln > 2048:
                    self.buf = self.buf[i_c + 2:]; continue
                if len(self.buf) < end:
                    break
                f = self.buf[i_c:end]; self.buf = self.buf[end:]
                self.casic(f)
            elif i_n != -1:
                e = self.buf.find(b"\r\n", i_n)
                if e == -1:
                    if len(self.buf) > 4096:
                        self.buf = b""
                    break
                line = self.buf[i_n:e]; self.buf = self.buf[e + 2:]
                self.sentence(line)
            else:
                self.buf = self.buf[-1:]
                break

    def casic(self, f):
        cls, mid = f[4], f[5]
        SEEN[(cls, mid)] = SEEN.get((cls, mid), 0) + 1
        payload = f[6:-4]
        if struct.unpack("<I", f[-4:])[0] != checksum(cls, mid, payload):
            return
        if cls == 0x05:
            if mid == 0x01: self.acks += 1
            else: self.nacks += 1
            self.last_ack = (mid, payload[0], payload[1]) if len(payload) >= 2 else (mid, 0, 0)
            k = (mid, self.last_ack[1], self.last_ack[2]); ACKS[k] = ACKS.get(k, 0) + 1
        elif cls == 0x08 and mid in (0x05, 0x06, 0x07) and self.capture:
            self.frames.append(f)

    def sentence(self, line):
        self.lines += 1
        try:
            body = line[1:line.index(b"*")].decode()
        except Exception:
            return
        p = body.split(",")
        if p[0].endswith("GGA") and len(p) > 9:
            q = int(p[6] or 0); self.sats = int(p[7] or 0)
            self.gga_q = q; self.gga_n += 1
            self.fix = q > 0 and self.sats >= 4
            if self.fix and p[2] and p[4]:
                self.lat = (int(p[2][:2]) + float(p[2][2:]) / 60) * (1 if p[3] == "N" else -1)
                self.lon = (int(p[4][:3]) + float(p[4][3:]) / 60) * (1 if p[5] == "E" else -1)
                self.alt = float(p[9] or 0)
        elif p[0].endswith("RMC") and len(p) > 9 and p[2] == "A" and p[1] and p[9]:
            hh, mm, ss = int(p[1][0:2]), int(p[1][2:4]), int(float(p[1][4:]))
            dd, mo, yy = int(p[9][0:2]), int(p[9][2:4]), 2000 + int(p[9][4:6])
            RTC().datetime((yy, mo, dd, 0, hh, mm, ss, 0))
            self.date = (yy, mo, dd)

SEEN = {}
ACKS = {}
R = Rx()

def run_for(ms):
    t = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t) < ms:
        R.pump(); time.sleep_ms(5)

def wait_fix(timeout_s=FIX_TIMEOUT):
    t = time.ticks_ms(); R.fix = False
    while time.ticks_diff(time.ticks_ms(), t) < timeout_s * 1000:
        R.pump()
        if R.fix:
            return time.ticks_diff(time.ticks_ms(), t) / 1000
        time.sleep_ms(5)
        LED.value((time.ticks_ms() // 250) & 1)
    return None

def wait_output(timeout_s=10):
    """Time until the receiver talks again (it boots, or leaves standby)."""
    t = time.ticks_ms(); n = R.lines
    while time.ticks_diff(time.ticks_ms(), t) < timeout_s * 1000:
        R.pump()
        if R.lines > n:
            return time.ticks_diff(time.ticks_ms(), t) / 1000
        time.sleep_ms(5)
    return None

def silent_for(ms, limit_ms=10000):
    """True once no byte has arrived for ms; False if it is still talking after limit_ms."""
    run_for(300); t = t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t) < ms:
        if time.ticks_diff(time.ticks_ms(), t0) > limit_ms:
            return False
        if U.any():
            U.read(); t = time.ticks_ms()
        time.sleep_ms(10)
    return True

# ---------------------------------------------------------------- aiding
def days_from_civil(y, m, d):
    y -= m <= 2
    era = y // 400; yoe = y - era * 400
    doy = (153 * (m + (-3 if m > 2 else 9)) + 2) // 5 + d - 1
    doe = yoe * 365 + yoe // 4 - yoe // 100 + doy
    return era * 146097 + doe - 719468

def gps_time():
    y, mo, d, _, h, mi, s, _ = RTC().datetime()
    unix = days_from_civil(y, mo, d) * 86400 + h * 3600 + mi * 60 + s
    g = unix - 315964800 + 18                     # GPS epoch 1980-01-06, 18 leap seconds
    return g // 604800, g % 604800

def aid_ini(lat, lon, alt, pacc=100.0):
    wn, tow = gps_time()
    flags = 0x01 | 0x02 | 0x20                    # position valid, time valid, position is LLA
    payload = struct.pack("<ddddffffIHBB", lat, lon, alt, float(tow), 0.0, pacc, 2.0, 0.0,
                          0, wn, 0, flags)
    U.write(frame(0x0B, 0x01, payload))
    return wn, tow

def capture_eph():
    R.frames = []; R.capture = True
    for rate in (0xFFFF, 1):                      # one copy each; then periodic, if a poll gives nothing
        for mid in (0x07, 0x06, 0x05):            # GPSEPH, GPSION, GPSUTC
            U.write(frame(0x06, 0x01, struct.pack("<BBH", 0x08, mid, rate)))
            run_for(1500)
        run_for(4000)
        if R.frames:
            break
    for mid in (0x07, 0x06, 0x05):
        U.write(frame(0x06, 0x01, struct.pack("<BBH", 0x08, mid, 0)))
    run_for(1000); R.capture = False
    log("casic messages seen so far (class, id): count", sorted(SEEN.items()))
    with open("/eph.bin", "wb") as f:
        for fr in R.frames:
            f.write(fr)
    return len(R.frames)

def replay_eph():
    a0, n0 = R.acks, R.nacks; sent = 0
    for fr in R.frames:
        U.write(fr); sent += 1
        run_for(30)
    run_for(500)
    return sent, R.acks - a0, R.nacks - n0

def power_cut():
    EN.value(1); time.sleep(OFF_S); EN.value(0)
    return wait_output()




# ---------------------------------------------------------------- the assisted cold-start run
# Can a cold start, with nothing in memory, be cut below ~30 s by giving the receiver time and
# position (AID-INI) and fresh orbit data (CASIC's AGNSS file, /casic.bin: 32 MSG-GPSEPH and a
# MSG-GPSION, fetched from espruino.com/agps/casic.base64) straight away? Each cold restart is
# confirmed by the receiver reporting no fix before the clock starts; injection goes in 1 s later.
REPEAT = 3
CASIC = open("/casic.bin", "rb").read()

def casic_frames():
    out, j = [], 0
    while j + 6 <= len(CASIC):
        ln = CASIC[j + 2] | CASIC[j + 3] << 8
        out.append(CASIC[j:j + 6 + ln + 4]); j += 6 + ln + 4
    return out
EPH = casic_frames()

def wait_fix_from(t0, timeout_s=FIX_TIMEOUT):
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_s * 1000:
        R.pump()
        if R.fix:
            return time.ticks_diff(time.ticks_ms(), t0) / 1000
        time.sleep_ms(5)
        LED.value((time.ticks_ms() // 250) & 1)
    return None

def send_aid(lat, lon, alt):
    for attempt in range(1, 6):
        R.last_ack = None
        aid_ini(lat, lon, alt, 10.0)
        t = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t) < 600:
            R.pump()
            if R.last_ack and R.last_ack[1:] == (0x0B, 0x01):
                return attempt, "ack" if R.last_ack[0] == 1 else "nack"
            time.sleep_ms(5)
    return 5, "none"

def cold_restart():
    for attempt in range(1, 4):
        U.write(nmea("PCAS10,2"))
        t = time.ticks_ms(); n = R.gga_n
        while time.ticks_diff(time.ticks_ms(), t) < 5000:
            R.pump()
            if R.gga_n > n and R.gga_q == 0:
                R.fix = False
                return attempt, time.ticks_ms()
            time.sleep_ms(5)
    return None, time.ticks_ms()

def send_eph():
    a0 = dict(ACKS)
    for f in EPH:
        U.write(f); run_for(25)
    run_for(600)
    d = {}
    for k, v in ACKS.items():
        if k[1] == 0x08 and v - a0.get(k, 0):
            d["%s%02x" % ("ack" if k[0] == 1 else "nack", k[2])] = v - a0.get(k, 0)
    return len(EPH), d

def main():
    log("==== L76K assisted cold-start test; casic frames", len(EPH))
    t = wait_fix(600)
    log("first fix", t, "sats", R.sats)
    if t is None:
        log("no fix: move to open sky and power-cycle to rerun"); return
    lat, lon, alt = R.lat, R.lon, R.alt           # held in memory only, never logged
    run_for(20000)
    cases = ["none", "aid+eph", "eph", "aid"]
    for rep in range(REPEAT):
        log("---- round", rep + 1)
        order = cases[rep:] + cases[:rep]         # a different order each round
        for name in order:
            attempts, t0 = cold_restart()
            if attempts is None:
                log(name, "cold restart NOT confirmed; skipped"); run_for(5000); continue
            while time.ticks_diff(time.ticks_ms(), t0) < 1000:
                R.pump(); time.sleep_ms(5)
            info = ""
            if "aid" in name:
                info += " aid=%s" % (send_aid(lat, lon, alt),)
            if "eph" in name:
                info += " eph(sent,acks)=%s" % (send_eph(),)
            log(name, "restart_attempts", attempts, "ttff", wait_fix_from(t0), "sats", R.sats, info)
            run_for(20000)
    log("==== done")

try:
    main()
except Exception as e:
    import sys
    with open(LOG, "a") as f:
        sys.print_exception(e, f)
    sys.print_exception(e)
LED.value(1)
