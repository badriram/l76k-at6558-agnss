from machine import Pin, UART
import time
en = Pin(34, Pin.OUT, value=0)      # active low: GNSS power on
stby = Pin(40, Pin.OUT, value=1)    # high: force awake
rst = Pin(42, Pin.OUT, value=1)     # high: not in reset
pps = Pin(41, Pin.IN)
u = UART(1, baudrate=9600, tx=38, rx=39, rxbuf=4096)
t0 = time.ticks_ms(); buf = b""; edges = 0; last = pps.value()
while time.ticks_diff(time.ticks_ms(), t0) < 6000:
    d = u.read()
    if d: buf += d
    v = pps.value()
    if v and not last: edges += 1
    last = v
print("bytes", len(buf), "pps edges", edges)
for line in buf.split(b"\r\n")[:14]:
    print(line[:100])
