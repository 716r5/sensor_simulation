"""Auto-start logger. Ctrl+C stops. Raw values are intentionally retained."""
from sensors import bus, Sensors
from records import FlashLog, FramLog, pack
import time
import os
import binascii

MODE = 'flash'  # 'flash': short prototype; 'stream': long PC capture; 'fram': added hardware
PERIOD_MS = 100  # target 10 Hz. Sequential sensors; actual timestamps are logged.
MAX_SECONDS = 60 if MODE == 'flash' else 0  # protect flash from continuous bench wear

def run():
    i = bus()
    s = Sensors(i)
    log = FlashLog() if MODE == 'flash' else FramLog(i) if MODE == 'fram' else None
    if MODE not in ('flash', 'fram', 'stream'):
        raise ValueError('Unknown MODE')
    boot = int.from_bytes(os.urandom(4), 'little')
    print('# mode=%s boot=%d BMP180_coefficients=%s' % (MODE, boot, str(s.c)))
    print('# READY: records are prefixed DATA; host must verify CRC')
    previous = time.ticks_us()
    elapsed = 0
    seq = 0
    deadline = time.ticks_ms()
    while True:
        now = time.ticks_us()
        elapsed += time.ticks_diff(now, previous)
        previous = now
        values = s.read()
        record = pack(boot, seq, elapsed, values)
        if log is not None:
            log.append(record)  # persist first; print only after storage call succeeds
        print('DATA,' + binascii.hexlify(record).decode())
        seq += 1
        if MAX_SECONDS and elapsed >= MAX_SECONDS * 1000000:
            print('# STOP: demo duration reached; old logs retained')
            return
        deadline = time.ticks_add(deadline, PERIOD_MS)
        remaining = time.ticks_diff(deadline, time.ticks_ms())
        if remaining < 0:
            print('# STOP: deadline missed; lower sample rate or inspect storage')
            return
        time.sleep_ms(remaining)

try:
    run()
except KeyboardInterrupt:
    print('# STOP: user stopped acquisition')
except Exception as e:
    # Never delete logs, reformat, or retry destructive recovery on an error.
    print('# STOP ERROR:', repr(e))
